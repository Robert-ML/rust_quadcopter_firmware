// Rust libraries
use core::time::Duration;

// Our libraries
use crate::calibrationdata::CalibrationData;
use crate::sensors_dmp::SensorsDMP;
use crate::sensors_raw::SensorsRaw;
use common::io::{ComErr, ComT};
use common::protocol::{ControlDT, DataT, SensorLogDT};
use common::DroneMode;

// TUDelft library
use tudelft_quadrupel::flash::FlashError;

// This crate imports
use crate::state_machine::*;

// This module imports
use super::config::DroneConfig;

const PIPE_SIZE: usize = 128;
const COM_BUF_SIZE: usize = 64;

const LOG_DATA_FIELD_NO: usize = 9;

const ADDRESS_OF_LOG_REPORT_EOF: u32 = 0x00;

type FP = fixed::types::I16F16;

/// Main structure that keeps track of the drone state (communication pipe,
/// drone mode, configuration parameters, keep alive etc.).
///
/// Call `tick` method in order to update the drone state according to the
/// mode. We statically dispatch according to the mode what state in the
/// `state_machine` module we use.
///
/// Note: Only one instance of this object should be created.
pub struct DroneState {
    // communication double way pipe
    pipe: ComT<PIPE_SIZE>,

    // the mode the drone is at the moment
    mode: DroneMode,

    // internal variables representing the state of the drone
    received_command: ControlDT, // received control command
    motor_command: [u16; 4],     // last command given to motors

    // configurations
    pub config: DroneConfig,

    // misc
    ticks_since_last_ka: u32,

    // To be used by Yaw control and stable mode
    pub calibrated_data: CalibrationData,

    pub sensors_dmp: SensorsDMP,
    pub sensors_raw: SensorsRaw,
    pub P: FP,
    pub P1: FP,
    pub P2: FP,

    // logging utility variables
    flash_iterator: u32, // it is the memory address where the cursor is
    log_report_eof: u32, // address at which the log report ends
    log_on: bool,
    log_report_on: bool,

    // debug stuff
    pub debug_info: DataT,
}

impl DroneState {
    pub fn new() -> Self {
        let pipe = ComT::<PIPE_SIZE>::new(
            tudelft_quadrupel::uart::receive_bytes,
            tudelft_quadrupel::uart::send_bytes,
        );

        Self {
            pipe: pipe,

            mode: DroneMode::Safe,

            received_command: ControlDT {
                // set controls to maximum by default
                lift: 2048,
                roll: 2048,
                pitch: 2048,
                yaw: 2048,
            },
            motor_command: [0; 4],

            config: DroneConfig::default(),

            ticks_since_last_ka: 0,

            calibrated_data: CalibrationData::new(),
            sensors_dmp: SensorsDMP::new(),
            sensors_raw: SensorsRaw::new(),
            P: FP::from_num(5),
            P1: FP::from_num(5),
            P2: FP::from_num(5),

            flash_iterator: ADDRESS_OF_LOG_REPORT_EOF + 0x04, // the first address is for storing the last address (EOF)
            log_report_eof: ADDRESS_OF_LOG_REPORT_EOF + 0x04,
            log_on: false,
            log_report_on: false,

            debug_info: DataT::KeepAlive,
        }
    }

    /// Perform "tick" and update the state of the drone according to the mode.
    pub fn tick(&mut self, iter_count: u32, delta_t: Duration) {
        // check keep alive
        self.tick_time_since_keep_alive();
        if (self.check_alive() == false)
            && (self.mode != DroneMode::Safe)
            && (self.mode != DroneMode::Panic)
        {
            self.mode = DroneMode::Panic
        }

        self.log_if_enabled();
        self.log_report_if_enabled(iter_count);

        self.dispatch_mode(iter_count, delta_t);
    }

    // TODO: do this with function pointer as parameter or something generic
    fn dispatch_mode(&mut self, iter_count: u32, delta_t: Duration) {
        match self.mode {
            DroneMode::Safe => self.internal_tick::<safemode::SafeMode>(iter_count, delta_t),
            DroneMode::Manual => self.internal_tick::<manualmode::ManualMode>(iter_count, delta_t),
            DroneMode::Panic => self.internal_tick::<panicmode::PanicMode>(iter_count, delta_t),
            DroneMode::Calibrate => {
                self.internal_tick::<calibratemode::CalibrateMode>(iter_count, delta_t)
            }
            DroneMode::YawControl => {
                self.internal_tick::<yawcontrolmode::YawControlMode>(iter_count, delta_t)
            }
            DroneMode::FullControl => {
                self.internal_tick::<fullcontrolmode::FullControlMode>(iter_count, delta_t)
            }
            DroneMode::RawMode => self.internal_tick::<rawmode::RawMode>(iter_count, delta_t),
        }
    }

    fn internal_tick<MODE: ModeTrait>(&mut self, iter_count: u32, delta_t: Duration) {
        if self.log_on {
            self.sensors_raw
                .update_sensor_readings_raw(&self.calibrated_data);
            self.sensors_dmp.update_sensor_readings_dmp();
        } else if self.mode == DroneMode::RawMode {
            self.sensors_raw
                .update_sensor_readings_raw(&self.calibrated_data);
            let roll_val = self.sensors_raw.get_roll_value();
            self.debug_info = DataT::Message(heapless::String::from(
                alloc::format!("E r {:#?}", roll_val).as_str(),
            ));
        } else {
            self.sensors_dmp.update_sensor_readings_dmp();
        }

        let new_mode = MODE::operate(self, iter_count, delta_t);

        // TODO: decide if this approach is good. I do the tick immediatly
        // because I do not consume the recv_buffer when we change the mode
        // tick immediatly when mode changes
        if self.mode != new_mode {
            self.mode = new_mode;

            // inform the PC about the mode change
            self.send_data(common::protocol::DataT::Mode(self.mode));

            // perform the remaining operations (if any) in accordance with the
            // new mode
            self.dispatch_mode(iter_count, delta_t);
        }
    }

    /// set received control command
    pub fn set_cc(&mut self, data: ControlDT) {
        self.received_command = data;
    }

    /// get last received control command
    pub fn get_cc(&self) -> ControlDT {
        return self.received_command;
    }

    /// get received control command as a vector
    pub fn get_cc_as_vec(&self) -> [u16; 4] {
        [
            self.received_command.lift,
            self.received_command.roll,
            self.received_command.pitch,
            self.received_command.yaw,
        ]
    }

    /// Tries to send `data` and returns if it succedded or not
    pub fn send_data(&mut self, data: DataT) -> bool {
        // TODO: do more error handling
        if let Err(err) = self.pipe.send_data::<COM_BUF_SIZE>(data) {
            #[allow(unused_must_use)]
            {
                // in case of any error, send message data packet with error
                // if this fails, do not try again
                self.pipe
                    .send_data::<COM_BUF_SIZE>(DataT::Message(heapless::String::from(
                        alloc::format!("E s {:#?}", err).as_str(),
                    )));
            }

            false
        } else {
            true
        }
    }

    /// Guarantees the return of some data, even in case of error. If error
    /// returns `DataT::Empty`.
    pub fn read_data(&mut self) -> DataT {
        match self.pipe.read_data::<COM_BUF_SIZE>() {
            Ok(data) => data,
            // TODO: do more error handling
            Err(err) => {
                match err {
                    ComErr::Empty => {}

                    _ => {
                        #[allow(unused_must_use)]
                        {
                            // send message packet with error if error not Empty
                            // if this fails, do not try again
                            self.pipe.send_data::<COM_BUF_SIZE>(DataT::Message(
                                heapless::String::from(alloc::format!("E r {:#?}", err).as_str()),
                            ));
                        }
                    }
                };

                DataT::Empty
            }
        }
    }

    #[inline]
    /// This function MUST be used to set the motor command and NOT the
    /// function `tudelft_quadrupel::motor::set_motors(motor_command)`!
    pub fn set_motors(&mut self, motor_command: [u16; 4]) {
        self.motor_command = motor_command;
        tudelft_quadrupel::motor::set_motors(motor_command);
    }

    #[inline]
    /// Seturns the internally stored motor command, NOT the value from
    /// `tudelft_quadrupel::motors::get_motors()`!
    pub fn get_motors(&self) -> [u16; 4] {
        self.motor_command
    }

    /// Increments the counter since last keep alive message that was received.
    fn tick_time_since_keep_alive(&mut self) {
        self.ticks_since_last_ka += 1;
    }

    /// Resets the counter since last keep alive message.
    pub fn got_keep_alive(&mut self) {
        self.ticks_since_last_ka = 0;
    }

    /// Checks if the connection is still considdered good and alive. If `true`
    /// everything should be all right, if `false` we can consider
    /// disconnected.
    pub fn check_alive(&self) -> bool {
        if self.ticks_since_last_ka > self.config.max_ticks_no_ka {
            false
        } else {
            true
        }
    }

    /// Send keep alive packet to inform the runner the drone is still
    /// connected.
    pub fn send_alive(&mut self) {
        self.send_data(DataT::KeepAlive);
    }

    /// Enable the logging
    pub fn start_logging(&mut self) {
        if self.log_on == true || self.log_report_on == true {
            return;
        }

        self.log_on = true;
        self.flash_iterator = ADDRESS_OF_LOG_REPORT_EOF + 0x04;

        self.send_data(DataT::Message(heapless::String::from(
            alloc::format!("log start").as_str(),
        )));
    }

    /// Stop the logging and store the end of the logged data in the first 4 bytes
    pub fn stop_logging(&mut self) {
        if self.log_on == false {
            return;
        }
        self.send_data(DataT::Message(heapless::String::from(
            alloc::format!("log stop").as_str(),
        )));

        self.log_on = false;
        let bytes: [u8; 4] = [
            self.flash_iterator.to_be_bytes()[0],
            self.flash_iterator.to_be_bytes()[1],
            self.flash_iterator.to_be_bytes()[2],
            self.flash_iterator.to_be_bytes()[3],
        ];

        self.flash_iterator = ADDRESS_OF_LOG_REPORT_EOF + 0x04;

        let result: Result<(), FlashError> =
            tudelft_quadrupel::flash::flash_write_bytes(ADDRESS_OF_LOG_REPORT_EOF, &bytes);

        match result {
            Ok(_) => {}
            Err(_) => {
                self.send_data(DataT::Message(heapless::String::from(
                    alloc::format!("e: log stop").as_str(),
                )));
            }
        };
    }

    /// The part that performs the actual logging
    fn log_if_enabled(&mut self) {
        if self.log_on == false {
            return;
        }

        // TODO: get data from the raw source that Panos is writing
        // prepare data
        let sensor_data_raw = self.sensors_raw.read(&self.calibrated_data);
        let data: [i16; LOG_DATA_FIELD_NO] = [
            sensor_data_raw.0.x,
            sensor_data_raw.0.y,
            sensor_data_raw.0.z,
            sensor_data_raw.1.x,
            sensor_data_raw.1.y,
            sensor_data_raw.1.z,
            (FP::from_num(10000) * self.sensors_dmp.get_dmp_pitch_value(self)).to_num::<i16>(),
            (FP::from_num(10000) * self.sensors_dmp.get_dmp_roll_value(self)).to_num::<i16>(),
            (FP::from_num(10000) * self.sensors_dmp.get_dmp_yaw_value(self)).to_num::<i16>(),
        ];

        let mut bytes: [u8; LOG_DATA_FIELD_NO * 2] = [0u8; LOG_DATA_FIELD_NO * 2];
        for i in 0..data.len() {
            bytes[i * 2] = data[i].to_be_bytes()[0];
            bytes[i * 2 + 1] = data[i].to_be_bytes()[1];
        }

        let result: Result<(), FlashError> =
            tudelft_quadrupel::flash::flash_write_bytes(self.flash_iterator, &bytes);

        match result {
            Ok(_) => self.flash_iterator += (LOG_DATA_FIELD_NO as u32) * 2,
            Err(e) => {
                match e {
                    FlashError::SpiError(_) => {}
                    FlashError::OutOfSpace => {
                        self.send_data(DataT::Message(heapless::String::from(
                            alloc::format!("e: log").as_str(),
                        )));
                        self.stop_logging();
                    }
                };
            }
        };
    }

    /// Start log report to the PC
    pub fn log_report_start(&mut self) {
        if self.log_on == true || self.log_report_on == true {
            return;
        }

        // get the end address of the log report
        let mut recv_buf: [u8; 4] = [0; 4];

        let result: Result<(), FlashError> =
            tudelft_quadrupel::flash::flash_read_bytes(ADDRESS_OF_LOG_REPORT_EOF, &mut recv_buf);
        match result {
            Ok(_) => {
                self.log_report_on = true;
                self.log_report_eof = u32::from_be_bytes(recv_buf);
                self.flash_iterator = ADDRESS_OF_LOG_REPORT_EOF + 0x04;
            }
            Err(_) => {
                // well, unlucky
                self.send_data(DataT::Message(heapless::String::from(
                    alloc::format!("e: log report start").as_str(),
                )));
            }
        };

        self.send_data(DataT::Message(heapless::String::from(
            alloc::format!("log report start").as_str(),
        )));
    }

    /// Stop the sending of the log report and announced the PC it did this
    /// through a DataT::StopLogReporting message
    pub fn log_report_stop(&mut self) {
        if self.log_report_on == false {
            return;
        }

        self.send_data(DataT::StopLogReporting);

        self.log_report_on = false;
        self.flash_iterator = ADDRESS_OF_LOG_REPORT_EOF + 0x04;

        self.send_data(DataT::Message(heapless::String::from(
            alloc::format!("log report stop").as_str(),
        )));
    }

    /// Send a data entry from the log
    fn log_report_if_enabled(&mut self, iter_count: u32) {
        if self.log_report_on == false {
            return;
        }

        if self.flash_iterator >= self.log_report_eof {
            self.log_report_stop();
            return;
        }

        if iter_count % self.config.log_report_send_period != 0 {
            return;
        }

        let mut recv_buf: [u8; LOG_DATA_FIELD_NO * 2] = [0; LOG_DATA_FIELD_NO * 2];

        let result: Result<(), FlashError> =
            tudelft_quadrupel::flash::flash_read_bytes(self.flash_iterator, &mut recv_buf);
        self.flash_iterator += (LOG_DATA_FIELD_NO * 2) as u32;

        match result {
            Ok(_) => {
                let log_report_data: DataT = DataT::SensorLog(SensorLogDT {
                    gyro_x: i16::from_be_bytes([recv_buf[0], recv_buf[1]]),
                    gyro_y: i16::from_be_bytes([recv_buf[2], recv_buf[3]]),
                    gyro_z: i16::from_be_bytes([recv_buf[4], recv_buf[5]]),

                    accel_x: i16::from_be_bytes([recv_buf[6], recv_buf[7]]),
                    accel_y: i16::from_be_bytes([recv_buf[8], recv_buf[9]]),
                    accel_z: i16::from_be_bytes([recv_buf[10], recv_buf[11]]),

                    pitch: FP::from_num(i16::from_be_bytes([recv_buf[12], recv_buf[13]]))
                        / FP::from_num(10000),
                    roll: FP::from_num(i16::from_be_bytes([recv_buf[14], recv_buf[15]]))
                        / FP::from_num(10000),
                    yaw: FP::from_num(i16::from_be_bytes([recv_buf[16], recv_buf[17]]))
                        / FP::from_num(10000),
                });

                //     self.sensors_raw.get_pitch_value().to_num::<i16>(),
                // self.sensors_raw.get_roll_value().to_num::<i16>(),
                // self.sensors_raw.get_yaw_value_der().to_num::<i16>(),
                // self.sensors_raw.get_pitch_der().to_num::<i16>(),
                // self.sensors_raw.get_roll_der().to_num::<i16>(),
                // self.sensors_dmp.get_dmp_pitch_value(self).to_num::<i16>(),
                // self.sensors_dmp.get_dmp_roll_value(self).to_num::<i16>(),
                // self.sensors_dmp.get_dmp_yaw_value(self).to_num::<i16>(),

                self.send_data(log_report_data);
            }
            Err(_) => {
                self.send_data(DataT::Message(heapless::String::from(
                    alloc::format!("e: log report").as_str(),
                )));
                self.log_report_stop();
            }
        };
    }
}
