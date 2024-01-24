/// Configuration structure with default values for some variables. It can be
/// used in the future to dnamically change parameters on the drone like PID
/// values, telemetry periods etc.
pub struct DroneConfig {
    pub dead_margin: u16,
    pub panic_motor_reduction: u16,

    // once how many ticks does the drone send aa keep alive
    pub ka_tick_period: u32,
    // after how many ticks considers the drone the serial dropped
    pub max_ticks_no_ka: u32,

    pub battery_printing_time: u32,
    pub check_battery: bool, // TO enable and disable battery checks

    pub log_report_send_period: u32,

    // debug print periods
    pub debug_info_period: u32,
    pub debug_motor_command_period: u32,
}

impl DroneConfig {
    pub fn default() -> Self {
        Self {
            dead_margin: 50,
            panic_motor_reduction: 2,

            ka_tick_period: 40,
            max_ticks_no_ka: 120,

            battery_printing_time: 100,
            check_battery: true,

            log_report_send_period: 2,

            // debug print periods
            debug_info_period: 50,
            debug_motor_command_period: 20,
        }
    }
}
