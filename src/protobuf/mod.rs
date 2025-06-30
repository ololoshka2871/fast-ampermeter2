mod process_requiest;
mod encode_md_message;
mod process_control;
mod process_output;
mod input_pocket_collector;

pub mod messages;

pub use process_requiest::process_request;
pub use encode_md_message::encode_md_message_to;
pub use input_pocket_collector::{InputPocketCollector, ParseError};
pub use process_output::OutputData;

#[allow(unused)]
pub use process_control::{Control, ControlAction, ControlState, CalibrationStatus};

pub fn default_response(id: u32, now: u64) -> messages::Response {
    use messages::{Info, Status};
    messages::Response {
        id,
        device_id: Info::RkMeter3Id as u32,
        protocol_version: Info::ProtocolVersion as u32,
        global_status: Status::Ok as i32,
        timestamp: now,

        ..Default::default()
    }
}
