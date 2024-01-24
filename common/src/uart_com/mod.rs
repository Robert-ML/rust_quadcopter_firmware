use crc::{Crc, CRC_8_LTE};
use serde::{Deserialize, Serialize};

// MACROS that make life easyer
macro_rules! check_Error {
    ($val: expr, $error: expr) => {
        match $val {
            Ok(_) => {}
            Err(_) => {
                return Err($error);
            }
        }
    };
}

macro_rules! check_OCS {
    ($val: expr) => {
        check_Error!($val, Error::ENOMEM);
    };
}

macro_rules! check_LibBug {
    ($val: expr) => {
        check_Error!($val, Error::LibBug);
    };
}

// constants
pub const START_BYTE: u8 = '<' as u8;
pub const END_BYTE: u8 = '>' as u8;
const ESCAPE_BYTE: u8 = '\\' as u8;
const MASK: u8 = { 1 << 3 } as u8;

// CRC calculation object
const LTE: Crc<u8> = Crc::<u8>::new(&CRC_8_LTE);

// ERROR codes
#[derive(Debug, PartialEq)]
pub enum Error {
    CRC,    // CRC error in the packet
    Des,    // failed the deserialize opperation
    ENOMEM, // out of space (CAP too small most probably)

    LibBug, // shoud not happen, but who knows
}

// packet structure to be sent over UART
#[derive(Serialize, Deserialize)]
struct HeaderT {
    crc: u8,
}

#[derive(Serialize, Deserialize)]
struct PacketT<const CAP: usize> {
    header: HeaderT,
    payload: heapless::Vec<u8, CAP>,
}

/// Creates a frame ready to be sent over the wire.
/// @input data - byte vector
/// @return byte stream ready to be sent over the wire (a frame) OR error
#[inline]
pub fn frame<const CAP: usize>(
    data: heapless::Vec<u8, CAP>,
) -> Result<heapless::Vec<u8, CAP>, Error> {
    let packet: PacketT<CAP> = PacketT::<CAP> {
        header: HeaderT {
            crc: LTE.checksum(&data),
        },
        payload: data,
    };

    match postcard::to_vec::<PacketT<CAP>, CAP>(&packet) {
        Ok(serialized) => {
            return prepare_frame::<CAP>(serialized);
        }
        Err(_) => {
            return Err(Error::ENOMEM);
        }
    };
}

/// Unwraps the frame that was sent over the wire.
/// @input frame - must start with START_BYTE and end in END_BYTE
/// @return data in byte format OR error
#[inline]
pub fn unframe<const CAP: usize>(
    frame: heapless::Vec<u8, CAP>,
) -> Result<heapless::Vec<u8, CAP>, Error> {
    let serialized: heapless::Vec<u8, CAP> = extract_data::<CAP>(frame)?;

    match postcard::from_bytes::<PacketT<CAP>>(&serialized) {
        Ok(packet) => {
            if packet.header.crc != LTE.checksum(&packet.payload) {
                return Err(Error::CRC);
            }

            return Ok(packet.payload);
        }
        Err(_) => {
            return Err(Error::Des);
        }
    };
}

/// Encapsulate the "data" between START_BYTE and END_BYTE and uniquifyes the
/// middle part no not contain them
/// @input data - byte vector
///
/// @return byte stream ready to be sent over the wire (a frame) OR error
#[inline]
fn prepare_frame<const CAP: usize>(
    data: heapless::Vec<u8, CAP>,
) -> Result<heapless::Vec<u8, CAP>, Error> {
    let mut ret: heapless::Vec<u8, CAP> = heapless::Vec::<u8, CAP>::new();

    // insert the starting byte of the stream
    check_OCS!(ret.push(START_BYTE));

    // make sure the special bytes do not appear in the transmitted byte stream
    for data_byte in data {
        if data_byte == START_BYTE || data_byte == END_BYTE || data_byte == ESCAPE_BYTE {
            check_OCS!(ret.push(ESCAPE_BYTE as u8));
            check_OCS!(ret.push(data_byte ^ MASK));
        } else {
            check_OCS!(ret.push(data_byte));
        }
    }

    // insert the ending byte of the stream
    check_OCS!(ret.push(END_BYTE));

    return Ok(ret);
}

/// Extract the "data" from between START_BYTE and END_BYTE dis-uniquifying the
/// middle part to get the original
/// @input frame - must start with START_BYTE and end in END_BYTE

/// @return byte stream ready to be deserialized OR error
#[inline]
fn extract_data<const CAP: usize>(
    frame: heapless::Vec<u8, CAP>,
) -> Result<heapless::Vec<u8, CAP>, Error> {
    // TODO: make more memory efficient by repurposing "frame" instead of "ret"
    let mut ret: heapless::Vec<u8, CAP> = heapless::Vec::<u8, CAP>::new();

    let mut escape: bool = false;
    // i range takes care of START_BYTE and END_BYTE
    for i in 1..(frame.len() - 1) {
        if escape {
            check_LibBug!(ret.push(frame[i] ^ MASK));
            escape = false;
        } else if frame[i] == ESCAPE_BYTE as u8 {
            escape = true;
        } else {
            check_LibBug!(ret.push(frame[i]));
        }
    }

    return Ok(ret);
}

// --------------- TESTING ---------------

// TODO: maybe is better to test with hex values, not frame - unframe
#[test]
fn frame_and_unframe() {
    let data: heapless::Vec<u8, 64> =
        heapless::Vec::<u8, 64>::from_slice(&[0, 1, 'a' as u8, 'b' as u8]).unwrap();
    let reference = data.clone();

    assert_eq!(
        unframe::<64>(frame::<64>(data).unwrap()).unwrap(),
        reference
    );
}

#[test]
fn frame_and_unframe_escape_bytes_only() {
    let data: heapless::Vec<u8, 64> =
        heapless::Vec::<u8, 64>::from_slice(&[START_BYTE, END_BYTE, ESCAPE_BYTE, MASK]).unwrap();

    assert_eq!(
        unframe::<64>(frame::<64>(data.clone()).unwrap()).unwrap(),
        data
    );
}

#[test]
fn unframe_bad_data() {
    let data = heapless::Vec::<u8, 64>::from_slice(&[100, 22, 30, 30, 30, 30, 30, 20]).unwrap();
    assert_eq!(unframe::<64>(data).err().unwrap(), Error::Des);
}

#[test]
// We have to make sure that none of the tokens become any of the other tokens when they are masked in the protocol.
fn token_exclusivity() {
    assert_ne!(ESCAPE_BYTE ^ MASK, START_BYTE);
    assert_ne!(ESCAPE_BYTE ^ MASK, END_BYTE);
    assert_ne!(START_BYTE ^ MASK, ESCAPE_BYTE);
    assert_ne!(START_BYTE ^ MASK, END_BYTE);
    assert_ne!(END_BYTE ^ MASK, START_BYTE);
    assert_ne!(END_BYTE ^ MASK, ESCAPE_BYTE);
}
