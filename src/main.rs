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
use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::dma::{CircBuffer, CircReadDma, DmaExt, RxDma};
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{Analog, GpioExt, Output, PushPull, PA0, PA1, PB12, PB13, PB14, PB15};

use stm32f1xx_hal::pac::{adc1, ADC1};
use stm32f1xx_hal::rcc::RccExt;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};

use embedded_hal::adc::OneShot;

use fugit::RateExtU32;

use usb_device::prelude::{UsbDevice, UsbDeviceBuilder};

use rtic::app;
use rtic_monotonics::systick::prelude::*;

//-----------------------------------------------------------------------------

systick_monotonic!(Mono, config::SYST_TIMER_HZ);

//-----------------------------------------------------------------------------

defmt::timestamp!("[{=u64:ms}]", Mono::now().ticks());

//-----------------------------------------------------------------------------

const AUDIO_EP_SIZE: usize =
    utils::ep_size(usbd_audio::Format::S16le, 2, config::DISCRETISATION_RATE); // 44 семпла -> 176 байт
const AUDIO_EP_SIZE_MAX: usize = AUDIO_EP_SIZE * 2;

const TEST_TABLE: [i16; 96] = [
    0, 32700, 4268, 32420, 8463, 31586, 12514, 30211, 16350, 28319, 19906, 25943, 23122, 23122,
    25943, 19906, 28319, 16350, 30211, 12514, 31586, 8463, 32420, 4268, 32700, 0, 32420, -4268,
    31586, -8463, 30211, -12514, 28319, -16350, 25943, -19906, 23122, -23122, 19906, -25943, 16350,
    -28319, 12514, -30211, 8463, -31586, 4268, -32420, 0, -32700, -4268, -32420, -8463, -31586,
    -12514, -30211, -16350, -28319, -19906, -25943, -23122, -23122, -25943, -19906, -28319, -16350,
    -30211, -12514, -31586, -8463, -32420, -4268, -32700, 0, -32420, 4268, -31586, 8463, -30211,
    12514, -28319, 16350, -25943, 19906, -23122, 23122, -19906, 25943, -16350, 28319, -12514,
    30211, -8463, 31586, -4268, 32420,
];

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
            self.set_channel_sample_time(ch, SampleTime::T_7);
        });
    }

    fn set_sequence(&mut self) {
        self.set_regular_sequence(&AdcPins::<MS>::cahnnels());
        // на 1 старт свполнить MS конвертаций и остановиться
        self.set_discontinuous_mode(Some(1));
    }
}

// use multisampling 1
type AdcPinsMS = AdcPins<{ config::DISCRETISATION_RATE as usize / 1000 }>;

//-----------------------------------------------------------------------------

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM, CAN_SCE, TIM1_TRG_COM])]
mod app {
    use super::*;

    type AdcTransferBuffer = CircBuffer<
        [u16; AdcPinsMS::buf_size()],
        RxDma<
            AdcPayload<stm32f1xx_hal::pac::ADC1, AdcPinsMS, stm32f1xx_hal::adc::Scan>,
            stm32f1xx_hal::dma::dma1::C1,
        >,
    >;

    #[shared]
    struct Shared {
        usb_audio: usbd_audio::AudioClass<'static, UsbBus<Peripheral>>,
        usb_device: UsbDevice<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        adc_transfer: AdcTransferBuffer,
        t1: PB12<Output<PushPull>>,
        t2: PB13<Output<PushPull>>,
        t3: PB14<Output<PushPull>>,
        t4: PB15<Output<PushPull>>,
    }

    #[init(local = [usb_bus: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None])]
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
        let mut afio = ctx.device.AFIO.constrain(&mut rcc);

        defmt::info!("\tClocks: {}", defmt::Debug2Format(&clocks));

        // Initialize the systick interrupt & obtain the token to prove that we did
        Mono::start(ctx.core.SYST, clocks.sysclk().to_Hz());

        let dma_channels = ctx.device.DMA1.split(&mut rcc); // for defmt

        let mut gpioa = ctx.device.GPIOA.split(&mut rcc);
        let mut gpiob = ctx.device.GPIOB.split(&mut rcc);

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
            cortex_m::asm::delay(50000);

            (None, usb_dp)
        };

        //---------------------------------------------------------------------

        let usb = stm32f1xx_hal::usb::Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        };

        ctx.local.usb_bus.replace(UsbBus::new(usb));
        let usb_bus = ctx.local.usb_bus.as_ref().unwrap();

        let usb_audio = {
            use drivers::AudioStreamConfigExt;

            usbd_audio::AudioClassBuilder::new()
                .input(
                    usbd_audio::StreamConfig::new_discrete(
                        usbd_audio::Format::S16le,
                        2,
                        &[config::DISCRETISATION_RATE],
                        // or ExtSpdifConnector, (ExtLineConnector, ExtLegacyAudioConnector, Ext1394*)
                        usbd_audio::TerminalType::ExtLineConnector,
                    )
                    .unwrap()
                    .set_ep_size(AUDIO_EP_SIZE_MAX as u16), // это костыль!
                )
                .build(usb_bus)
                .unwrap()
        };

        let usb_dev =
            UsbDeviceBuilder::new(usb_bus, usb_device::prelude::UsbVidPid(0x0483, 0x5F23))
                .composite_with_iads()
                .strings(&[usb_device::device::StringDescriptors::default()
                    .manufacturer("Lolka_097")
                    .product("fast-ampermeter2")
                    .serial_number(stm32_device_signature::device_id_hex())])
                .unwrap()
                .build();

        defmt::info!("\tUSB");

        let adc_transfer = {
            // use timer1 ch1 as external trigger for ADC
            let ch1 = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
            let mut pwm = stm32f1xx_hal::timer::Timer::new(ctx.device.TIM1, &mut rcc)
                .pwm_hz(ch1, &mut afio.mapr, config::DISCRETISATION_RATE.Hz())
                .split();
            pwm.set_duty(pwm.get_max_duty() / 2); // 50% duty cycle
            pwm.enable();

            let mut adc_rx_ch = gpioa.pa0.into_analog(&mut gpioa.crl);
            let mut adc_v_pll_ch = gpioa.pa1.into_analog(&mut gpioa.crl);

            let mut adc1 = adc::Adc::new(ctx.device.ADC1, &mut rcc);

            let adc_rx_ch_v: u16 = adc1.read(&mut adc_rx_ch).unwrap();
            let adc_v_pll_ch_v: u16 = adc1.read(&mut adc_v_pll_ch).unwrap();
            defmt::info!("ADC test: {} {}", adc_rx_ch_v, adc_v_pll_ch_v);

            let pins = AdcPins(adc_rx_ch, adc_v_pll_ch);

            adc1.set_align(adc::Align::Right);
            adc1.set_external_trigger(adc1::cr2::EXTSEL::Tim1cc1);

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

        let mut t1 = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
        let mut t2 = gpiob.pb13.into_push_pull_output(&mut gpiob.crh);
        let mut t3 = gpiob.pb14.into_push_pull_output(&mut gpiob.crh);
        let mut t4 = gpiob.pb15.into_push_pull_output(&mut gpiob.crh);

        //---------------------------------------------------------------------

        if let Some(mut usb_pull_up) = usb_pull_up {
            usb_pull_up.toggle(); // enable USB
            defmt::info!("USB enabled");
        }

        //---------------------------------------------------------------------

        defmt::info!("Init done");

        //---------------------------------------------------------------------

        (
            Shared {
                usb_audio,
                usb_device: usb_dev,
            },
            Local {
                adc_transfer,
                t1,
                t2,
                t3,
                t4,
            },
        )
    }

    //-------------------------------------------------------------------------

    #[task(binds = USB_HP_CAN_TX, shared = [usb_device, usb_audio], local = [t1], priority = 4)]
    fn usb_tx(ctx: usb_tx::Context) {
        let usb_device = ctx.shared.usb_device;
        let usb_audio = ctx.shared.usb_audio;
        let t1 = ctx.local.t1;

        t1.set_high();
        (usb_device, usb_audio).lock(|usb_device, usb_audio| usb_device.poll(&mut [usb_audio]));
        t1.set_low();
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_device, usb_audio], local = [t2], priority = 4)]
    fn usb_rx0(ctx: usb_rx0::Context) {
        let usb_device = ctx.shared.usb_device;
        let usb_audio = ctx.shared.usb_audio;
        let t2 = ctx.local.t2;

        t2.set_high();
        (usb_device, usb_audio).lock(|usb_device, usb_audio| usb_device.poll(&mut [usb_audio]));
        t2.set_low();
    }

    #[task(binds = DMA1_CHANNEL1, shared = [usb_audio], local = [adc_transfer, t3, t4], priority = 2)]
    fn adc_dma_half_complete(ctx: adc_dma_half_complete::Context) {
        let mut usbd_audio = ctx.shared.usb_audio;

        let adc_transfer = ctx.local.adc_transfer;
        let t3 = ctx.local.t3;
        let t4 = ctx.local.t4;

        match adc_transfer.peek(|buff, _half| unsafe {
            /*
            core::slice::from_raw_parts(
                buff.as_ptr() as *const u8,
                buff.len() * core::mem::size_of::<u16>(),
            )
            */
            core::slice::from_raw_parts(
                TEST_TABLE.as_ptr() as *const u8,
                TEST_TABLE.len() * core::mem::size_of::<i16>(),
            )
        }) {
            Ok(data_ptr) => {
                match usbd_audio.lock(move |usbd_audio| {
                    usbd_audio.write(data_ptr)
                }) {
                    Ok(_n) => {
                        t4.set_high();
                        t4.set_low();
                    }
                    Err(e) => {
                        t3.set_high();
                        t3.set_low();
                    }
                }
            }
            Err(e) => {
                defmt::error!("ADC DMA irq error: {}", defmt::Debug2Format(&e));
                unsafe {
                    #[allow(invalid_value)]
                    let mut fake_transfer =
                        core::mem::MaybeUninit::<AdcTransferBuffer>::zeroed().assume_init();
                    core::mem::swap(&mut fake_transfer, adc_transfer);
                    let (data, channel) = fake_transfer.stop();
                    let mut new_transfer = channel.circ_read(data);
                    core::mem::swap(adc_transfer, &mut new_transfer);
                    core::mem::forget(new_transfer);
                }
            }
        }
    }
}
