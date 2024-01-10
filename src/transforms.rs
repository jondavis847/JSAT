use nalgebra::{Matrix3, Matrix6, UnitQuaternion, Vector3, Vector6};
use std::ops::*;

pub struct RotationMatrix {
    value: Matrix3<f64>
}
pub struct Cartesian {
    rotation:RotationMatrix,
    translation: Vector3<f64>,
}

impl Mul<Cartesian> for Cartesian {
    type Output = Cartesian;

    fn mul(self, other: Cartesian) -> Cartesian {
        Cartesian {
            rotation: RotationMatrix{value: self.rotation.value * other.rotation.value},
            translation: other.translation + other.rotation.value.transpose() * self.translation,
        }
    }
}
impl Mul<Vector3<f64>> for Cartesian {
    type Output = Vector3<f64>;

    fn mul(self, other: Vector3<f64>) -> Vector3<f64> {
        let v = self.rotation * (other - self.translation);
        Vector3::new(v[0], v[1], v[2])
    }
}
