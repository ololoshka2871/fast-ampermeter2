use usbd_audio::{Format, Rates, StreamConfig, TerminalType};

pub trait AudioStreamConfigExt {
    fn set_ep_size(self, size: u16) -> Self;
}

// Это полная копия usbd_audio::StreamConfig
// Будем добираться до поля ep_size
#[derive(Debug)]
#[allow(unused)]
struct MyStreamConfig<'a> {
    format: Format,
    channels: u8,
    rates: Rates<'a>,
    terminal_type: TerminalType,
    /// ISO endpoint size calculated from format, channels and rates (may be
    /// removed in future)
    ep_size: u16, // В оригинальной структуре это поле приватное, поэтому будем использовать доступ через трансмутацию
}

impl<'a> AudioStreamConfigExt for StreamConfig<'a> {
    fn set_ep_size(self, size: u16) -> Self {
        let mut my_config = unsafe { core::mem::transmute::<StreamConfig, MyStreamConfig>(self) };
        my_config.ep_size = size;
        unsafe { core::mem::transmute::<MyStreamConfig, StreamConfig>(my_config) }
    }
}
