use cordic::atan2;
use fixed_sqrt::FixedSqrt;
use tudelft_quadrupel::fixed::traits::FromFixed;
use tudelft_quadrupel::mpu::structs::Quaternion;
type FP = fixed::types::I16F16;
/// This struct holds the yaw, pitch, and roll that the drone things it is in.
/// The struct is currently implemented using `f32`, you may want to change this to use fixed point arithmetic.
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub yaw: FP,
    pub pitch: FP,
    pub roll: FP,
}

impl From<Quaternion> for YawPitchRoll {
    /// Creates a YawPitchRoll from a Quaternion
    fn from(q: Quaternion) -> Self {
        let Quaternion { w, x, y, z } = q;
        let w = FP::from_fixed(w);
        let x = FP::from_fixed(x);
        let y = FP::from_fixed(y);
        let z = FP::from_fixed(z);

        let gx = FP::from_num(2) * (x * z - w * y);
        let gy = FP::from_num(2) * (w * x + y * z);
        let gz = w * w - x * x - y * y + z * z;

        // yaw: (about Z axis)
        let yaw = atan2(
            FP::from_num(2) * x * y - FP::from_num(2) * w * z,
            FP::from_num(2) * w * w + FP::from_num(2) * x * x - FP::from_num(1),
        );
        //micromath::I16F16::atan2(I16F16::from_num(2) * x * y - I16F16::from_num(2) * w * z, I16F16::from_num(2) * w * w + I16F16::from_num(2) * x * x - I16F16::from_num(1));

        // pitch: (nose up/down, about Y axis)
        let pitch = atan2(gx, FixedSqrt::sqrt(gy * gy + gz * gz));
        // micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        let roll = atan2(gy, gz);
        //micromath::F32Ext::atan2(gy, gz);

        Self { yaw, pitch, roll }
    }
}
