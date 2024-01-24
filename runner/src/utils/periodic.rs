use std::option::Option;
use std::time::Duration;

pub struct Timer {
    period: Duration,
    timer: Duration,

    func: Option<fn() -> ()>,
}

impl Timer {
    #[allow(dead_code)]
    pub fn new(period: Duration, f: Option<fn() -> ()>) -> Self {
        Self {
            period: period,
            timer: Duration::new(0, 0),
            func: f,
        }
    }

    /// pass check reset do
    /// passes the time, checks if the timer expiered, if yes resets it and call function
    #[allow(dead_code)]
    pub fn pcrd(&mut self, dt: Duration) -> bool {
        if self.pcr(dt) {
            if let Some(func) = self.func {
                func();
            }
            true
        } else {
            false
        }
    }

    /// pass check reset
    /// passes the time, checks if the timer expiered, if yes resets it
    #[allow(dead_code)]
    pub fn pcr(&mut self, dt: Duration) -> bool {
        self.timer += dt;

        self.cr()
    }

    /// pass check
    /// passes the time, checks if the timer expiered
    #[allow(dead_code)]
    pub fn pc(&mut self, dt: Duration) -> bool {
        self.timer += dt;

        self.c()
    }

    /// check reset
    /// checks if the timer expiered, if yes resets it
    #[allow(dead_code)]
    pub fn cr(&mut self) -> bool {
        if self.timer > self.period {
            self.timer = Duration::new(0, 0);
            true
        } else {
            false
        }
    }

    /// check
    /// checks if the timer expiered
    #[allow(dead_code)]
    pub fn c(&self) -> bool {
        if self.timer > self.period {
            true
        } else {
            false
        }
    }
}
