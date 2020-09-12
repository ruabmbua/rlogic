use core::convert::TryInto;
use stm32f1xx_hal::{prelude::*, time::Hertz};

#[repr(u8)]
#[allow(dead_code)]
enum CommandOp {
    Stop = 0x00,
    Start,
    _End,
}

pub enum Command {
    Stop,
    /// .0 Frequency in hertz
    Start(Hertz),
}

impl Command {
    pub fn parse(bytes: &[u8]) -> Option<Self> {
        if bytes[0] >= CommandOp::_End as u8 {
            return None;
        }
        let op: CommandOp = unsafe { core::mem::transmute(bytes[0]) };

        match op {
            CommandOp::Stop => Some(Command::Stop),
            CommandOp::Start => {
                let val = bytes[1..5].try_into().unwrap();
                Some(Command::Start(u32::from_le_bytes(val).hz()))
            }
            _ => unreachable!(),
        }
    }
}
