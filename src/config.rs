#![allow(unused)]


pub const XTAL_FREQ: u32 = 8_000_000;

// System clock trate
pub const SYST_CLOCK_HZ: u32 = 72_000_000;

// System timer rate
pub const SYST_TIMER_HZ: u32 = 1_000;

//-----------------------------------------------------------------------------

// usb pull up
pub const USB_PULLUP_UNACTVE_LEVEL: Option<stm32f1xx_hal::gpio::PinState> = Some(stm32f1xx_hal::gpio::PinState::High);

//-----------------------------------------------------------------------------

// heap size
pub const HEAP_SIZE: usize = 2048;

//-----------------------------------------------------------------------------

pub const DISCRETISATION_RATE: u32 = 48_000;