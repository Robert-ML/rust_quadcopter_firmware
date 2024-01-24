use ringbuffer::{
    ConstGenericRingBuffer, RingBuffer, RingBufferExt, RingBufferRead, RingBufferWrite,
};

use crate::protocol::DataT;
use crate::uart_com;

const BUFFER_SIZE: usize = 8;

#[derive(Debug)]
pub enum ComErr {
    Empty,       // no hole packet to be extracted
    InvalPacket, // found invalid pair START_BYTE - END_BYTE (maybe corruption)
    ENOMEM,      // passed capacity is too small
    Busy,        // send_f returned false and could not send data

    UartError(uart_com::Error),

    Bug, // something wrong happened
}

/// Structure to reuse the code in both dronecode and runner.
///
/// `BUF_CAP` is the underlying size of the receiving buffer, `read_f` is the
/// funciton used to populate the buffer, `send_f` is the function used to send
/// serialized data.
pub struct ComT<const BUF_CAP: usize> {
    recv_buffer: ConstGenericRingBuffer<u8, BUF_CAP>,

    read_f: fn(&mut [u8]) -> usize,
    send_f: fn(&[u8]) -> bool,
}

impl<const BUF_CAP: usize> ComT<BUF_CAP> {
    pub fn new(read_f: fn(&mut [u8]) -> usize, send_f: fn(&[u8]) -> bool) -> Self {
        Self {
            recv_buffer: ConstGenericRingBuffer::default(),
            read_f,
            send_f,
        }
    }

    /// Serializes data and sends it over the UART. `CAP` is used for internal
    /// buffers and is configurable.
    ///
    /// Returns `ComErr::ENOMEM` or `ComErr::UartError::ENOMEM` depending on where
    /// `CAP` became too small.
    ///
    /// Returns `ComErr::Busy` when `send_f` returned `false`.
    ///
    /// Returns also other errors from `uart_com::Error`.
    pub fn send_data<const CAP: usize>(&mut self, data: DataT) -> Result<(), ComErr> {
        match data.to_packet::<CAP>() {
            Ok(frame) => {
                // send the frame
                let ret: bool = (self.send_f)(&frame);

                if ret == true {
                    return Ok(());
                } else {
                    return Err(ComErr::Busy);
                }
            }
            Err(e) => {
                return Err(ComErr::UartError(e));
            }
        }
    }

    /// Populates the internal buffer using `read_f`. Looks for a data frame
    /// (between `uart_com::START_BYTE` and `uart_com::END_BYTE`).
    ///
    /// Returns `ComErr::Empty` if no data is ready yet.
    ///
    /// `CAP` is used for internal buffers and is configurable. Returns
    /// `ComErr::ENOMEM` or `ComErr::UartError::ENOMEM` depending on where `CAP`
    /// became too small.
    ///
    /// If an invalid frame is found, it returns `ComErr::InvalPacket`.
    ///
    /// Returns also other errors from `uart_com::Error`.
    pub fn read_data<const CAP: usize>(&mut self) -> Result<DataT, ComErr> {
        self.fill_recv_buffer::<CAP>();

        let frame: heapless::Vec<u8, CAP> = self.extract_frame_from_recv_buf::<CAP>()?;

        // extract the payload
        match DataT::from_packet::<CAP>(frame) {
            Ok(val) => {
                return Ok(val);
            }
            Err(err) => {
                return Err(ComErr::UartError(err));
            }
        };
    }

    // reads from the uart RX buffer and fills in the buffer passed as argument
    #[inline]
    fn fill_recv_buffer<const CAP: usize>(&mut self) -> usize {
        let mut written: usize = 0;

        loop {
            let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

            let remaining_space: usize = self.recv_buffer.capacity() - self.recv_buffer.len();

            if BUFFER_SIZE > remaining_space {
                break;
            }

            let written_now: usize = (self.read_f)(&mut buffer);
            written += written_now;

            for i in 0..written_now {
                self.recv_buffer.enqueue(buffer[i])
            }

            if written_now != BUFFER_SIZE || self.recv_buffer.is_full() == true {
                break;
            }
        }

        written
    }

    // tries to extract an entire Protocol packet
    #[inline]
    fn extract_frame_from_recv_buf<const CAP: usize>(
        &mut self,
    ) -> Result<heapless::Vec<u8, CAP>, ComErr> {
        self.align_to_packet();

        if self.recv_buffer.is_empty() {
            return Err(ComErr::Empty);
        }

        let end_byte_pos: usize = self.get_end_of_packet()?;

        // the raw byte escaped packet
        let mut data: heapless::Vec<u8, CAP> = heapless::Vec::<u8, CAP>::new();

        for _ in 0..=end_byte_pos {
            match self.recv_buffer.dequeue() {
                Some(val) => {
                    match data.push(val) {
                        Ok(_) => {}
                        Err(_) => {
                            return Err(ComErr::ENOMEM);
                        }
                    };
                }
                None => {
                    return Err(ComErr::Bug);
                }
            };
        }

        Ok(data)
    }

    // skip bytes untill the top value is the start byte in the protocol
    #[inline]
    fn align_to_packet(&mut self) {
        while (self.recv_buffer.is_empty() == false)
            && (*self.recv_buffer.peek().unwrap() != uart_com::START_BYTE)
        {
            self.recv_buffer.skip();
        }
    }

    #[inline]
    fn get_end_of_packet(&mut self) -> Result<usize, ComErr> {
        for (i, it) in self.recv_buffer.iter().enumerate() {
            if *it == uart_com::END_BYTE {
                return Ok(i);
            } else if (i != 0) && (*it == uart_com::START_BYTE) {
                // delete packet with no END_BYTE
                for _ in 0..i {
                    self.recv_buffer.skip();
                }
                return Err(ComErr::InvalPacket);
            }
        }

        Err(ComErr::Empty)
    }
}
