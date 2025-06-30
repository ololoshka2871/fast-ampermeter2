use prost::bytes::{BufMut, BytesMut};
use prost::DecodeError;

use super::messages::Request;

#[derive(Clone)]
pub enum ParseError {
    Overflow,
    InsuficientData,
    InvalidMagick,
    InvalidSize,
    InvalidMessage(DecodeError),
}

impl defmt::Format for ParseError {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            ParseError::Overflow => defmt::write!(fmt, "Overflow"),
            ParseError::InsuficientData => defmt::write!(fmt, "InsuficientData"),
            ParseError::InvalidMagick => defmt::write!(fmt, "InvalidMagick"),
            ParseError::InvalidSize => defmt::write!(fmt, "InvalidSize"),
            ParseError::InvalidMessage(_e) => defmt::write!(fmt, "InvalidMessage"),
        }
    }
}

pub struct InputPocketCollector {
    pocket: BytesMut,
}

impl InputPocketCollector {
    pub fn new(max_size: usize) -> Self {
        Self {
            pocket: BytesMut::with_capacity(max_size),
        }
    }

    pub fn reset(&mut self) {
        self.pocket.clear();
    }

    pub fn feed(&mut self, data: u8) -> Result<Request, ParseError> {
        use prost::Message;

        if self.pocket.len() >= self.pocket.capacity() {
            Err(ParseError::Overflow)?;
        }

        self.pocket.put_u8(data);

        if self.pocket.len() < 5 {
            Err(ParseError::InsuficientData)?;
        }

        if self.pocket[0] != crate::protobuf::messages::Info::Magick as u8 {
            Err(ParseError::InvalidMagick)?;
        }

        let (size, data_offset) = {
            let mut size_end = 5;
            for n in 1usize..=5 {
                if self.pocket[n] < 0x80 {
                    size_end = n + 1;
                    break;
                }
            }
            let size = prost::decode_length_delimiter(&self.pocket[1..size_end])
                .map_err(|_| ParseError::InvalidSize)?;

            if size > self.pocket.capacity() - size_end {
                Err(ParseError::Overflow)?;
            }

            (size, size_end)
        };

        if size + data_offset > self.pocket.len() {
            Err(ParseError::InsuficientData)?;
        }

        crate::protobuf::messages::Request::decode(&self.pocket[data_offset..])
            .map_err(|e| ParseError::InvalidMessage(e))
    }
}
