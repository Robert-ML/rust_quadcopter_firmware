use eframe::egui;
use std::sync::{Arc, Mutex};

#[derive(Clone)]
pub struct GuiParams {
    pub(crate) status: Arc<Mutex<u8>>,
    pub(crate) drone_mode: Arc<Mutex<String>>,
    pub(crate) last_keyboard_key_pressed: Arc<Mutex<String>>,
    pub(crate) joystick_roll_input: Arc<Mutex<i32>>,
    pub(crate) joystick_pitch_input: Arc<Mutex<i32>>,
    pub(crate) joystick_yaw_input: Arc<Mutex<i32>>,
    pub(crate) joystick_throttle_input: Arc<Mutex<i32>>,
    pub(crate) motor_1_value: Arc<Mutex<u16>>,
    pub(crate) motor_2_value: Arc<Mutex<u16>>,
    pub(crate) motor_3_value: Arc<Mutex<u16>>,
    pub(crate) motor_4_value: Arc<Mutex<u16>>,
    pub(crate) yaw_p: Arc<Mutex<i32>>,
    pub(crate) rp_p1: Arc<Mutex<i32>>,
    pub(crate) rp_p2: Arc<Mutex<i32>>,
    pub(crate) last_message_received: Arc<Mutex<String>>,
    pub(crate) debug_prints_from_drone: Arc<Mutex<String>>,
    pub(crate) is_battery_weak: Arc<Mutex<bool>>,
    pub(crate) battery_health: Arc<Mutex<u16>>,
}

pub fn gui_terminal_init(drone_status: GuiParams) -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions {
        initial_window_size: Some(egui::vec2(1200.0, 500.0)),
        ..Default::default()
    };

    eframe::run_native(
        "GCS",
        options,
        // Box::new(|_cc| Box::new(MyApp::default())),
        Box::new(|_cc| {
            Box::new(GuiParams {
                status: drone_status.status,
                drone_mode: drone_status.drone_mode,
                last_keyboard_key_pressed: drone_status.last_keyboard_key_pressed,
                joystick_roll_input: drone_status.joystick_roll_input,
                joystick_pitch_input: drone_status.joystick_pitch_input,
                joystick_yaw_input: drone_status.joystick_yaw_input,
                joystick_throttle_input: drone_status.joystick_throttle_input,
                motor_1_value: drone_status.motor_1_value,
                motor_2_value: drone_status.motor_2_value,
                motor_3_value: drone_status.motor_3_value,
                motor_4_value: drone_status.motor_4_value,
                last_message_received: drone_status.last_message_received,
                debug_prints_from_drone: drone_status.debug_prints_from_drone,
                is_battery_weak: drone_status.is_battery_weak,
                yaw_p: drone_status.yaw_p,
                rp_p1: drone_status.rp_p1,
                rp_p2: drone_status.rp_p2,
                battery_health: drone_status.battery_health,
            })
        }),
    )
}

impl eframe::App for GuiParams {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("Left").show(ctx, |ui| {
            egui::widgets::global_dark_light_mode_buttons(ui);

            ui.heading("PC ACCESSORIES INPUT");
            ui.heading(format!(
                "Last key pressed: {:?}",
                self.last_keyboard_key_pressed.lock().unwrap()
            ));
            ui.heading(format!(
                "Joystick Roll: {:?}",
                self.joystick_roll_input.lock().unwrap()
            ));
            ui.heading(format!(
                "Joystick Pitch: {:?}",
                self.joystick_pitch_input.lock().unwrap()
            ));
            ui.heading(format!(
                "Joystick Yaw: {:?}",
                self.joystick_yaw_input.lock().unwrap()
            ));

            ui.heading(format!(
                "Joystick THrottle: {:?}",
                self.joystick_throttle_input.lock().unwrap()
            ));
        });
        egui::SidePanel::right("right").show(ctx, |ui| {
            egui::widgets::global_dark_light_mode_buttons(ui);
            ui.heading("STATUS");

            ui.heading(format!(
                "Motor_1_Value: {:?}",
                self.motor_1_value.lock().unwrap()
            ));
            ui.heading(format!(
                "Motor_2_Value: {:?}",
                self.motor_2_value.lock().unwrap()
            ));
            ui.heading(format!(
                "Motor_3_Value: {:?}",
                self.motor_3_value.lock().unwrap()
            ));
            ui.heading(format!(
                "Motor_4_Value: {:?}",
                self.motor_4_value.lock().unwrap()
            ));
        });
        // egui::TopBottomPanel::Top("Left").show(ctx, |ui| {
        //     egui::widgets::global_dark_light_mode_buttons(ui);
        //
        // });
        egui::CentralPanel::default().show(ctx, |ui| {
            egui::widgets::global_dark_light_mode_buttons(ui);
            ui.heading("Drone STUFF");

            ui.heading(format!("Drone Mode: {:?}", self.drone_mode.lock().unwrap()));
            ui.heading(format!(
                "Battery weak: {:?}",
                self.is_battery_weak.lock().unwrap()
            ));

            ui.heading(format!(
                "Message Received From Drone: {:?}",
                self.last_message_received.lock().unwrap()
            ));

            ui.heading(format!(
                "Debug prints values: {:?}",
                self.debug_prints_from_drone.lock().unwrap()
            ));

            ui.heading(format!("Yaw P: {:?}", self.yaw_p.lock().unwrap()));
            ui.heading(format!("FC P1: {:?}", self.rp_p1.lock().unwrap()));
            ui.heading(format!("FC P2: {:?}", self.rp_p2.lock().unwrap()));
            ui.heading(format!(
                "Battery Value: {:?}",
                self.battery_health.lock().unwrap()
            ));

            ui.ctx().request_repaint();
            if ui.button("Exit").clicked() {
                std::process::exit(0);
            }
        });

        egui::TopBottomPanel::bottom("top").show(ctx, |ui| {
            egui::widgets::global_dark_light_mode_buttons(ui);
            ui.heading("Keyboard Bindings");

            ui.heading(format!("PANIC MODE : 1  ESC , SPACE BAR"));
            ui.heading(format!("SAFE MODE : 0                            ||        MANUAL MODE : 2            ||    CALIBRATE MODE: 3"));
            ui.heading(format!("YAW CONTROL : 4                     ||        FULL CONTROL : 5            ||     RAW MODE : 6"));
            ui.heading(format!("THROTTLE TRIM : A/Z               ||        YAW TRIM: Q/W"));
            ui.heading(format!("PITCH TRIM : Arrow U/D          ||        ROLL TRIM: Arrow L/R"));
            ui.heading(format!("Reset trim values: R                  ||  Reset GUI debug: F"));
            ui.heading(format!("P trim: U,J                                     ||        P1 trim: I,K                               || P2 trim: O,L"));
            ui.heading(format!("Start Logging: C                                     ||        Stop Logging: V"));
            ui.heading(format!("Start Log Reporting: B                                     ||        Stop Log Reporting: N"));
            ui.ctx().request_repaint();
            if ui.button("Exit").clicked() {
                std::process::exit(0);
            }
        });
    }
}
//
// fn main() {
//     let mut state = State {
//         status: Arc::new(Mutex::new(5)),
//     };
//
//     let mut x = state.clone();
//
//
//
//
//     thread::spawn(
//         {
//             let state_clone = state.clone();
//             move || {
//                 let mut i = 10;
//                 loop {
//                     thread::sleep(time::Duration::from_millis(3000));
//                     *state_clone.status.lock().unwrap() += i ;
//                 }
//             }
//         });
//     gui_terminal_init(state);
//
//
//
//
//
//
// }
