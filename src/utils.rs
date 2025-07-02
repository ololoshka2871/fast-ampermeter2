/// from usb_audio/src/lib.rs
/// calculate ISO endpoint size from format, channels and rates
pub const fn ep_size(format: usbd_audio::Format, channels: u8, max_rate: u32) -> usize {
    const MAX_ISO_EP_SIZE: u32 = 1023; // not a public constant in usbd_audio

    let octets_per_frame = channels as u32
        * match format {
            usbd_audio::Format::S16le => 2,
            _ => unimplemented!(),
        };
    let ep_size = octets_per_frame * max_rate / 1000;
    if ep_size > MAX_ISO_EP_SIZE {
        panic!("ISO endpoint size too large");
    }
    ep_size as usize
}
