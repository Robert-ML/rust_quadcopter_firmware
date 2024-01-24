use crate::{yaw_pitch_roll::YawPitchRoll};

use common::{protocol::{DataT, SensorDT, HealthDT, ModeDT}, io::{ComT, ComErr}, state_machine};
use tudelft_quadrupel::{motor::get_motors, block, mpu::{read_raw, read_dmp_bytes}, battery::read_battery, barometer::read_pressure};

const BUF_CAP: usize = 32;

pub fn read_payload(pipe:&mut ComT<64>) -> Option<DataT> {
    if let Ok(payload) = pipe.read_data::<BUF_CAP>() {
        return Some(payload)
    }
    None
}

pub fn inform_mode_change(pipe:&mut ComT<64>, mode: state_machine::Mode) -> Result<(), ComErr> {
    let packet = DataT::Mode(mode);
    return pipe.send_data::<BUF_CAP>(packet);
}

fn read_sensors() -> DataT{
    let motors = get_motors();
    let quaternion = block!(read_dmp_bytes()).unwrap();
    let ypr = YawPitchRoll::from(quaternion);
    let (accel, _) = read_raw().unwrap();

    DataT::SensorData(SensorDT{sp: ypr.yaw as u8, sq: ypr.pitch as u8, sr: ypr.roll as u8, sax: accel.x as u8, say: accel.y as u8, saz: accel.z as u8})   // Check this

}

fn read_health_info() -> DataT{
    let bat = read_battery();
    let pres = read_pressure();
    DataT::HealthData(HealthDT{bat: bat as u8, cpu: 0, mode: pres as u8})
}


pub fn send_periodic_data(pipe: &mut ComT<64>) {
    pipe.send_data::<BUF_CAP>(read_sensors()).unwrap();
    pipe.send_data::<BUF_CAP>(read_health_info()).unwrap();
}
