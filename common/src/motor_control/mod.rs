use core::cmp::{max, min};

use fixed_sqrt::FixedSqrt;
type FP = fixed::types::I26F6;

// constants
const MAX_THRUST: i32 = 600;
const MAX_MOTOR_COMMAND: u16 = 800;
const MOTOR_STALL: u16 = 180;

// command constants
const MAX_INPUT_COMMAND: u16 = 2047;
const MIN_THRUST_COMMAND: u16 = 10;

// throttle command rate
const THROTTLE_A1: i32 = 12;
const THROTTLE_A0: i32 = (MOTOR_STALL as i32) - 3 * THROTTLE_A1; // ~sqrt(MIN_THRUST_COMMAND) * THROTTLE_A1

// maximum motor modifiers the commands can inflict on the base throttle
const MIN_ROLL_MODIF: i32 = -200;
const MAX_ROLL_MODIF: i32 = -MIN_ROLL_MODIF;

const MIN_PITCH_MODIF: i32 = MIN_ROLL_MODIF;
const MAX_PITCH_MODIF: i32 = MAX_ROLL_MODIF;

const MIN_YAW_MODIF: i32 = -300;
const MAX_YAW_MODIF: i32 = -MIN_YAW_MODIF;

// MACROS
macro_rules! get_untill_stall {
    ($throttle: expr) => {
        $throttle - (MOTOR_STALL as i32)
    };
}

#[derive(Debug, PartialEq)]
pub enum MappingError {
    InputOutOfBounds,
}

/// Input is in range [0, 2048]
/// Output has to be in range [0, MAX_THRUST]
pub fn motor_mapping(input: [u16; 4]) -> Result<[u16; 4], MappingError> {
    for i in input {
        if i > MAX_INPUT_COMMAND {
            return Err(MappingError::InputOutOfBounds);
        }
    }

    // do not spin the mottors if throttle command is small
    if input[0] <= MIN_THRUST_COMMAND {
        return Ok([0, 0, 0, 0]);
    }

    let throttle: i32 = min(
        (FixedSqrt::sqrt(FP::from_num(input[0])) * FP::from_num(THROTTLE_A1)
            + FP::from_num(THROTTLE_A0))
        .to_num::<i32>(),
        MAX_THRUST,
    );
    let until_stall: i32 = get_untill_stall!(throttle);

    let roll: i32 = (input[1] as i32) - 1024;
    let pitch: i32 = (input[2] as i32) - 1024;
    let yaw: i32 = (input[3] as i32) - 1024;

    let roll_modif: i32 = get_roll_modifier(roll, until_stall);
    let pitch_modif: i32 = get_pitch_modifier(pitch, until_stall);
    let yaw_modif: i32 = get_yaw_modifier(yaw, until_stall);

    // #[cfg(test)]
    // print!(
    //     "{}, {}, {}, {}\n",
    //     throttle, roll_modif, pitch_modif, yaw_modif
    // );

    let ae1: i32 = throttle + pitch_modif + yaw_modif;
    let ae2: i32 = throttle - roll_modif - yaw_modif;
    let ae3: i32 = throttle - pitch_modif + yaw_modif;
    let ae4: i32 = throttle + roll_modif - yaw_modif;

    // Convert the values back to u16
    let mapping: [i32; 4] = [ae1, ae2, ae3, ae4];
    let mut converted: [u16; 4] = [0; 4];

    // First make sure all the values are positive
    for i in 0..mapping.len() {
        // Bound the value by 0
        let mut bounded: u16 = max(mapping[i], 0) as u16;

        // Bound the value by MAX_THRUST
        //bounded = min(MAX_THRUST, bounded);
        bounded = min(MAX_MOTOR_COMMAND, bounded);

        converted[i] = bounded;
    }

    // #[cfg(test)]
    // println!("{:?}", converted);

    return Ok(converted);
}

/// check if the motors will stall with maximum modifiers
fn check_stall(until_stall: i32) -> bool {
    if until_stall < MAX_PITCH_MODIF + MAX_YAW_MODIF
        || until_stall < MAX_ROLL_MODIF + MAX_YAW_MODIF
        || until_stall < MAX_PITCH_MODIF + MAX_YAW_MODIF
        || until_stall < MAX_ROLL_MODIF + MAX_YAW_MODIF
    {
        return true;
    }

    return false;
}

fn get_roll_modifier(roll: i32, until_stall: i32) -> i32 {
    const OTHER_COMMAND_RANGE: i32 = MAX_YAW_MODIF - MIN_YAW_MODIF;

    get_modifier::<-1024, 1023, MIN_ROLL_MODIF, MAX_ROLL_MODIF, OTHER_COMMAND_RANGE>(
        roll,
        until_stall,
    )
}

fn get_pitch_modifier(pitch: i32, until_stall: i32) -> i32 {
    const OTHER_COMMAND_RANGE: i32 = MAX_YAW_MODIF - MIN_YAW_MODIF;

    get_modifier::<-1024, 1023, MIN_PITCH_MODIF, MAX_PITCH_MODIF, OTHER_COMMAND_RANGE>(
        pitch,
        until_stall,
    )
}

fn get_yaw_modifier(yaw: i32, until_stall: i32) -> i32 {
    const A: i32 = MAX_ROLL_MODIF - MIN_ROLL_MODIF;
    const B: i32 = MAX_PITCH_MODIF - MIN_PITCH_MODIF;

    const OTHER_COMMAND_RANGE: i32 = [A, B][(A < B) as usize];

    get_modifier::<-1024, 1023, MIN_YAW_MODIF, MAX_YAW_MODIF, OTHER_COMMAND_RANGE>(yaw, until_stall)
}

fn get_modifier<
    const FROM_L: i32,
    const FROM_H: i32,
    const TO_L: i32,
    const TO_H: i32,
    const OTHER_COMMAND_RANGE: i32,
>(
    command: i32,
    until_stall: i32,
) -> i32 {
    if until_stall < 0 {
        return 0;
    }

    let mut com_modif: i32 = map_range(
        (FP::from_num(FROM_L), FP::from_num(FROM_H)),
        (FP::from_num(TO_L), FP::from_num(TO_H)),
        FP::from_num(command),
    )
    .to_num::<i32>();

    if com_modif == 0 {
        return 0;
    }

    if check_stall(until_stall) == true {
        // caps the current command proportionally with another command
        let tcr: FP = FP::from_num(TO_H - TO_L); // this command's range
        let ocr: FP = FP::from_num(OTHER_COMMAND_RANGE);

        let capped_com_modif: i32 =
            ((tcr * FP::from_num(until_stall)) / (tcr + ocr)).to_num::<i32>();

        if com_modif > 0 {
            com_modif = min(com_modif, capped_com_modif);
        } else if com_modif < 0 {
            com_modif = max(com_modif, -capped_com_modif);
        }
    }

    return com_modif;
}

fn map_range(from_range: (FP, FP), to_range: (FP, FP), s: FP) -> FP {
    to_range.0 + (s - from_range.0) * (to_range.1 - to_range.0) / (from_range.1 - from_range.0)
}

#[cfg(test)]
mod test {
    use crate::motor_control::*;

    #[test]
    fn test_zero_state() {
        let zero_state: [u16; 4] = [0, 1024, 1024, 1024];
        let expected: [u16; 4] = [0, 0, 0, 0];
        assert_eq!(motor_mapping(zero_state).unwrap(), expected);
    }

    #[test]
    fn test_hover() {
        let zero_state: [u16; 4] = [2047, 1024, 1024, 1024];
        let result = motor_mapping(zero_state).unwrap();
        assert_eq!(result[0], result[1]);
        assert_eq!(result[1], result[2]);
        assert_eq!(result[2], result[3]);
    }

    #[test]
    fn test_roll_left() {
        let state: [u16; 4] = [1024, 0, 1024, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[1] > mapping[3]);
        assert_eq!(mapping[0], mapping[2]);
    }

    #[test]
    fn test_roll_right() {
        let state: [u16; 4] = [1024, 2047, 1024, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[1] < mapping[3]);
        assert_eq!(mapping[0], mapping[2]);
    }

    #[test]
    fn test_pitch_forward() {
        let state: [u16; 4] = [1024, 1024, 0, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[0] < mapping[2]);
        assert_eq!(mapping[1], mapping[3]);
    }

    #[test]
    fn test_pitch_backward() {
        let state: [u16; 4] = [1024, 1024, 2047, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[0] > mapping[2]);
        assert_eq!(mapping[1], mapping[3]);
    }

    #[test]
    fn test_turn_diag_fl() {
        let state: [u16; 4] = [1024, 0, 0, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[0] < mapping[2]);
        assert!(mapping[1] > mapping[3]);
    }

    #[test]
    fn test_turn_diag_fr() {
        let state: [u16; 4] = [1024, 2047, 0, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[0] < mapping[2]);
        assert!(mapping[1] < mapping[3]);
    }

    #[test]
    fn test_turn_diag_br() {
        let state: [u16; 4] = [1024, 2047, 2047, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[0] > mapping[2]);
        assert!(mapping[1] < mapping[3]);
    }

    #[test]
    fn test_turn_diag_bl() {
        let state: [u16; 4] = [1024, 0, 2047, 1024];
        let mapping: [u16; 4] = motor_mapping(state).unwrap();
        assert!(mapping[0] > mapping[2]);
        assert!(mapping[1] > mapping[3]);
    }

    #[test]
    fn test_zero_thrust() {
        let expected: [u16; 4] = [0, 0, 0, 0];

        let mut random_state: [u16; 4] = [0, 0, 0, 0];
        assert_eq!(motor_mapping(random_state).unwrap(), expected);

        random_state = [0, 2047, 0, 0];
        assert_eq!(motor_mapping(random_state).unwrap(), expected);

        random_state = [0, 2047, 2047, 0];
        assert_eq!(motor_mapping(random_state).unwrap(), expected);

        random_state = [0, 0, 2047, 0];
        assert_eq!(motor_mapping(random_state).unwrap(), expected);

        random_state = [0, 2047, 1000, 1000];
        assert_eq!(motor_mapping(random_state).unwrap(), expected);
    }

    #[test]
    fn test_input_out_of_bounds() {
        let expected = MappingError::InputOutOfBounds;
        let random_state: [u16; 4] = [2048, 0, 0, 0];
        assert_eq!(motor_mapping(random_state).err().unwrap(), expected);
    }
}
