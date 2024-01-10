use nalgebra::{Matrix3, Matrix6, UnitQuaternion, Vector3, Vector6};
use std::ops::*;
use std::fmt;

struct SpatialValue {
    rotation: Vector3<f64>,
    translation: Vector3<f64>,
}

struct CoordinateValue {
    rotation: UnitQuaternion<f64>,
    translation: Vector3<f64>,
}

pub struct Position(CoordinateValue);
pub struct Velocity(SpatialValue);
pub struct Acceleration(SpatialValue);
pub struct Momentum(SpatialValue);
pub struct Force(SpatialValue);

#[inline]
pub fn velocity(w: Vector3<f64>, v: Vector3<f64>) -> Velocity {
    Velocity(SpatialValue {
        rotation: w,
        translation: v,
    })
}

impl fmt::Display for SpatialValue {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "rotation: {} translation: {}",
            self.rotation, self.translation
        )
    }
}

impl fmt::Display for Velocity {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Velocity\n{}",
            self.0
        )
    }
}
impl fmt::Display for Momentum {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Momentum\n{}",
            self.0
        )
    }
}

pub struct Inertia {
    mass: f64,
    center_of_mass: Vector3<f64>,
    inertia: Matrix3<f64>,
    value: Matrix6<f64>,
}

fn skew(x: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, x[2], -x[1], -x[2], 0.0, x[0], x[1], -x[0], 0.0)
}

fn concat_block_2x2(
    b1: Matrix3<f64>,
    b2: Matrix3<f64>,
    b3: Matrix3<f64>,
    b4: Matrix3<f64>,
) -> Matrix6<f64> {
    Matrix6::new(
        b1[(0, 0)],
        b1[(0, 1)],
        b1[(0, 2)],
        b3[(0, 0)],
        b3[(0, 1)],
        b3[(0, 2)],
        b1[(1, 0)],
        b1[(1, 1)],
        b1[(1, 2)],
        b3[(1, 0)],
        b3[(1, 1)],
        b3[(1, 2)],
        b1[(2, 0)],
        b1[(2, 1)],
        b1[(2, 2)],
        b3[(2, 0)],
        b3[(2, 1)],
        b3[(2, 2)],
        b2[(0, 0)],
        b2[(0, 1)],
        b2[(0, 2)],
        b4[(0, 0)],
        b4[(0, 1)],
        b4[(0, 2)],
        b2[(1, 0)],
        b2[(1, 1)],
        b2[(1, 2)],
        b4[(1, 0)],
        b4[(1, 1)],
        b4[(1, 2)],
        b2[(2, 0)],
        b2[(2, 1)],
        b2[(2, 2)],
        b4[(2, 0)],
        b4[(2, 1)],
        b4[(2, 2)],
    )
}

pub fn inertia(
    mass: f64,
    center_of_mass: Vector3<f64>,
    inertia: Matrix3<f64>,
) -> Inertia {
    let c = skew(center_of_mass);
    let ct = c.transpose();
    let b1 = inertia + mass * c * ct;
    let b2 = mass * ct;
    let b3 = mass * c;
    let b4 = Matrix3::from_diagonal_element(mass);
    let value = concat_block_2x2(b1, b2, b3, b4);
    Inertia { mass, center_of_mass, inertia, value: value }
}

impl Mul<Velocity> for Inertia {
    type Output = Momentum;

    fn mul(self, velocity: Velocity) -> Momentum {
        let inertia = self.value.fixed_view::<3, 3>(0, 0);
        let mass = self.value[self.value.len()-1];
        let angular_momentum = inertia * velocity.0.rotation;
        let linear_momentum = mass * velocity.0.translation;
        Momentum(SpatialValue {
            rotation: angular_momentum,
            translation: linear_momentum,
        })
    }
}
