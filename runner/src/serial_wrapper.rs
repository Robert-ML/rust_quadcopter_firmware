use std::cell::OnceCell;
use std::sync::Mutex;
use std::time::Duration;

use serial2::SerialPort;

pub static SERIAL: Mutex<OnceCell<SerialPort>> = Mutex::new(OnceCell::new());

pub(crate) fn init_global_serial(
    name: impl AsRef<std::path::Path>,
    settings: impl serial2::IntoSettings,
) {
    match SerialPort::open(name, settings) {
        Ok(mut serial) => {
            let lock = SERIAL.lock().unwrap();

            if let Err(e) = serial.set_write_timeout(Duration::from_millis(15)) {
                log::error!(
                    "[ERROR]: Failed to initialize the serial when setting write timeout: {}\n",
                    e
                );
            }

            if let Err(e) = serial.set_read_timeout(Duration::from_millis(10)) {
                log::error!(
                    "[ERROR]: Failed to initialize the serial when setting read timeout: {}\n",
                    e
                );
            }

            match lock.set(serial) {
                Ok(_) => {
                    log::info!("Serial initialized\n");
                }
                Err(_) => {
                    log::error!("[ERROR]: Tried to reinitialize the serial\n");
                }
            }
        }
        Err(_) => {
            log::error!("[ERROR]: Could not open the serial\n");
            std::process::exit(-1);
        }
    }
}

pub fn send_bytes(bytes: &[u8]) -> bool {
    let lock = SERIAL.lock().unwrap();

    match lock.get() {
        Option::None => {
            log::error!("[ERROR]: Usage of SERIAL before initialization.\n");
            assert!(false);
            return false;
        }

        Option::Some(serial) => match serial.write(bytes) {
            Ok(written) => {
                if written == bytes.len() {
                    return true;
                }

                log::error!(
                    "[ERROR]: Could not write the entire array on the \
                    serial, the wrapper at the moment does not implement \
                    fragmented writing on the serial\n"
                );
                return false;
            }
            Err(_) => {
                log::error!("[ERROR]: failed to send on the serial\n");
                return false;
            }
        },
    };
}

pub fn receive_bytes(bytes: &mut [u8]) -> usize {
    let lock = SERIAL.lock().unwrap();

    match lock.get() {
        Option::None => {
            log::error!("[ERROR]: Usage of SERIAL before initialization.\n");
            assert!(false);
            return 0;
        }

        Option::Some(serial) => match serial.read(bytes) {
            Ok(read) => {
                return read;
            }
            Err(_) => {
                return 0;
            }
        },
    };
}
