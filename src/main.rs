#![no_main]
#![no_std]
#![feature(macro_metavar_expr)]
#![feature(alloc_error_handler)]

mod config;
mod drivers;
//mod protobuf;
mod utils;

use defmt_rtt as _; // global logger
use panic_abort as _;

use stm32f1xx_hal::adc::{self, Adc, AdcPayload, ChannelTimeSequence, SampleTime, SetChannels};
use stm32f1xx_hal::dma::{CircBuffer, CircReadDma, DmaExt, RxDma};
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{Analog, GpioExt, PA0, PA1};

use stm32f1xx_hal::pac::{adc1, ADC1};
use stm32f1xx_hal::rcc::RccExt;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};

use embedded_hal::adc::OneShot;

use fugit::RateExtU32;

use usb_device::prelude::{UsbDevice, UsbDeviceBuilder};

use rtic::app;
use rtic_monotonics::systick::prelude::*;

//-----------------------------------------------------------------------------

systick_monotonic!(Mono, config::SYST_CLOCK_HZ);

//-----------------------------------------------------------------------------

defmt::timestamp!("[{=u64:ms}]", Mono::now().ticks());

//-----------------------------------------------------------------------------

const AUDIO_EP_SIZE: usize =
    utils::ep_size(usbd_audio::Format::S16le, 2, config::DISCRETISATION_RATE); // 44 семпла -> 176 байт
const AUDIO_EP_SIZE_MAX: usize = AUDIO_EP_SIZE * 2;

pub struct AdcPins<const MS: usize>(PA0<Analog>, PA1<Analog>);

impl<const MS: usize> AdcPins<MS> {
    pub const fn cahnnels() -> [u8; 2] {
        [0, 1]
    }

    pub const fn buf_size() -> usize {
        Self::chank_size() * MS
    }

    pub const fn chank_size() -> usize {
        Self::cahnnels().len()
    }

    pub const fn multy_sampled_count() -> usize {
        MS
    }
}

impl<const MS: usize> SetChannels<AdcPins<MS>> for Adc<ADC1> {
    fn set_samples(&mut self) {
        AdcPins::<MS>::cahnnels().iter().for_each(|&ch| {
            self.set_channel_sample_time(ch, SampleTime::T_239);
        });
    }

    fn set_sequence(&mut self) {
        self.set_regular_sequence(&AdcPins::<MS>::cahnnels());
        self.set_continuous_mode(true);
    }
}

// use multisempling 128
type AdcPinsMS = AdcPins<128>;

//-----------------------------------------------------------------------------

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM, CAN_SCE, TIM1_TRG_COM])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        sample_buffer: heapless::Vec<u8, AUDIO_EP_SIZE_MAX>,
    }

    #[local]
    struct Local {
        usb_audio: usbd_audio::AudioClass<'static, UsbBus<Peripheral>>,
        usb_device: UsbDevice<'static, UsbBusType>,
        adc_transfer: CircBuffer<
            [u16; AdcPinsMS::buf_size()],
            RxDma<
                AdcPayload<stm32f1xx_hal::pac::ADC1, AdcPinsMS, stm32f1xx_hal::adc::Scan>,
                stm32f1xx_hal::dma::dma1::C1,
            >,
        >,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        defmt::info!("Init...");

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        defmt::info!("\tDWT");

        let mut flash = ctx.device.FLASH.constrain();
        let mut rcc = ctx.device.RCC.freeze(
            stm32f1xx_hal::rcc::Config::hse(config::XTAL_FREQ.Hz())
                .sysclk(config::SYST_CLOCK_HZ.Hz())
                .pclk1(config::SYST_CLOCK_HZ.Hz() / 2)
                .pclk2(config::SYST_CLOCK_HZ.Hz() / 2)
                .adcclk(config::SYST_CLOCK_HZ.Hz() / (2 * 8)),
            &mut flash.acr,
        );

        let clocks = rcc.clocks.clone();

        defmt::info!("\tClocks: {}", defmt::Debug2Format(&clocks));

        // Initialize the systick interrupt & obtain the token to prove that we did
        Mono::start(ctx.core.SYST, clocks.sysclk().to_Hz());

        let dma_channels = ctx.device.DMA1.split(&mut rcc); // for defmt

        let mut gpioa = ctx.device.GPIOA.split(&mut rcc);

        let (usb_pull_up, usb_dp) = if let Some(usb_pull_up_lvl) = config::USB_PULLUP_UNACTVE_LEVEL
        {
            // pa10 or replace to your pin
            let usb_pull_up = gpioa
                .pa10
                .into_push_pull_output_with_state(&mut gpioa.crh, usb_pull_up_lvl);
            (
                Some(usb_pull_up),
                gpioa.pa12.into_push_pull_output(&mut gpioa.crh),
            )
        } else {
            // https://github.com/will-hart/pedalrs/blob/dd33bf753c9d482c38a8365cc925822f105b12cd/src/configure/stm32f103.rs#L77
            // BluePill board has a pull-up resistor on the D+ line.
            // Pull the D+ pin down to send a RESET condition to the USB bus.
            // This forced reset is needed only for development, without it host
            // will not reset your device when you upload new firmware.
            let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
            usb_dp.set_low();
            cortex_m::asm::delay(10000);

            (None, usb_dp)
        };

        //---------------------------------------------------------------------

        let usb = stm32f1xx_hal::usb::Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        };

        let usb_bus = unsafe {
            cortex_m::singleton!(
                : usb_device::bus::UsbBusAllocator<UsbBus<Peripheral>> = UsbBus::new(usb)
            )
            .unwrap_unchecked()
        };

        let usb_audio = {
            use drivers::AudioStreamConfigExt;

            usbd_audio::AudioClassBuilder::new()
                .input(
                    usbd_audio::StreamConfig::new_discrete(
                        usbd_audio::Format::S16le,
                        2,
                        &[config::DISCRETISATION_RATE],
                        // or ExtSpdifConnector, (ExtLineConnector, ExtLegacyAudioConnector, Ext1394*)
                        usbd_audio::TerminalType::ExtSpdifConnector,
                    )
                    .unwrap()
                    .set_ep_size(AUDIO_EP_SIZE_MAX as u16), // это костыль!
                )
                .build(usb_bus)
                .unwrap()
        };

        // fixme
        let usb_dev =
            UsbDeviceBuilder::new(usb_bus, usb_device::prelude::UsbVidPid(0x0483, 0x3917))
                .strings(&[
                    usb_device::device::StringDescriptors::new(usb_device::LangID::RU_RU)
                        .manufacturer("SCTBElpa")
                        .product("rk-meter-agc-pll")
                        .serial_number(stm32_device_signature::device_id_hex()),
                ])
                .unwrap()
                .build();

        defmt::info!("\tUSB");

        let adc_transfer = {
            let mut adc_rx_ch = gpioa.pa0.into_analog(&mut gpioa.crl);
            let mut adc_v_pll_ch = gpioa.pa1.into_analog(&mut gpioa.crl);

            let mut adc1 = adc::Adc::new(ctx.device.ADC1, &mut rcc);

            let adc_rx_ch_v: u16 = adc1.read(&mut adc_rx_ch).unwrap();
            let adc_v_pll_ch_v: u16 = adc1.read(&mut adc_v_pll_ch).unwrap();
            defmt::info!("ADC test: {} {}", adc_rx_ch_v, adc_v_pll_ch_v);

            let pins = AdcPins(adc_rx_ch, adc_v_pll_ch);

            adc1.set_align(adc::Align::Right);
            adc1.set_external_trigger(adc1::cr2::EXTSEL::Swstart);

            let mut dma_ch1 = dma_channels.1;
            dma_ch1.listen(stm32f1xx_hal::dma::Event::HalfTransfer);
            dma_ch1.listen(stm32f1xx_hal::dma::Event::TransferComplete);
            let adc_dma = adc1.with_scan_dma(pins, dma_ch1);

            let ad_value = unsafe {
                cortex_m::singleton!(:
                    [[u16; AdcPinsMS::buf_size()]; 2] = [[0; AdcPinsMS::buf_size()]; 2]
                )
                .unwrap_unchecked()
            };

            // кольцевой режим АЦП
            adc_dma.circ_read(ad_value)
        };

        defmt::info!("\tADC");

        let sample_buffer = heapless::Vec::<u8, AUDIO_EP_SIZE_MAX>::new();

        defmt::info!("Buffers ready");

        //---------------------------------------------------------------------

        if let Some(mut usb_pull_up) = usb_pull_up {
            usb_pull_up.toggle(); // enable USB
            defmt::info!("USB enabled");
        }

        //---------------------------------------------------------------------

        defmt::info!("Init done");

        //---------------------------------------------------------------------

        (
            Shared { sample_buffer },
            Local {
                usb_audio,
                usb_device: usb_dev,
                adc_transfer,
            },
        )
    }

    //-------------------------------------------------------------------------

    #[task(binds = USB_HP_CAN_TX, priority = 4)]
    fn usb_tx(_ctx: usb_tx::Context) {
        usb::spawn().ok();
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 4)]
    fn usb_rx0(_ctx: usb_rx0::Context) {
        usb::spawn().ok();
    }

    #[task(shared = [sample_buffer], local = [usb_audio, usb_device], priority = 4)]
    async fn usb(ctx: usb::Context) {
        let usb_device = ctx.local.usb_device;
        let usb_audio = ctx.local.usb_audio;

        let mut sample_buffer = ctx.shared.sample_buffer;

        sample_buffer.lock(|sample_buffer| {
            let send_buf = if sample_buffer.is_empty() {
                &[0, 0, 0, 0] // получив это, хост следующий опрос устройство зажержит на ~4 мс.
            } else {
                sample_buffer.as_slice()
            };

            if usb_audio.write(send_buf).is_ok() || sample_buffer.is_full() {
                sample_buffer.clear();
            }
        });

        while usb_device.poll(&mut [usb_audio]) {}
    }

    #[task(binds = DMA1_CHANNEL1, shared = [sample_buffer], local = [adc_transfer], priority = 2)]
    fn adc_dma_half_complete(ctx: adc_dma_half_complete::Context) {
        let mut sample_buffer = ctx.shared.sample_buffer;
        let adc_transfer = ctx.local.adc_transfer;

        let result = sample_buffer.lock(|sample_buffer| {
            adc_transfer.peek(|buff, _half| {
                sample_buffer
                    .extend_from_slice(unsafe {
                        core::slice::from_raw_parts(buff.as_ptr() as *const u8, buff.len())
                    })
                    .ok();
            })
        });

        if let Err(e) = result {
            defmt::error!("ADC DMA irq error: {}", defmt::Debug2Format(&e));
        }
    }
}
