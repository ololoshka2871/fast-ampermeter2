use prost::{bytes::BufMut, Message};

pub fn encode_md_message_to(resp: super::messages::Response, buf: &mut impl BufMut) -> Result<(), prost::EncodeError> {
    buf.put_u8(super::messages::Info::Magick as u8);
    resp.encode_length_delimited(buf)
}