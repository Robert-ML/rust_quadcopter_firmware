use chrono::{DateTime, Local};
use log::{LevelFilter, Log, Metadata, Record};
use std::fs::{File, OpenOptions};
use std::io::prelude::*;
use std::sync::Mutex;

struct EslLogger;

static V: Mutex<i32> = Mutex::new(0); // Needed to create critical section for the logger. Do not remove

impl Log for EslLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= LevelFilter::Debug
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let timestamp: DateTime<Local> = Local::now();
            let message = format!("{}", record.args());

            let file: Mutex<File> = Mutex::new(
                OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open("ESL.log")
                    .unwrap(),
            );

            let _guard = V.lock().unwrap();
            writeln!(
                &mut file.try_lock().unwrap(),
                "{} - {} - {}",
                timestamp.format("%d-%m-%Y %H:%M:%S:%3f"),
                record.level(),
                message
            )
            .unwrap();
        }
    }

    fn flush(&self) {}
}

pub fn set_logger() {
    log::set_logger(&EslLogger)
        .map(|()| log::set_max_level(LevelFilter::Debug))
        .unwrap();
}
