use nalgebra::{Matrix3, Matrix6, UnitQuaternion, Vector3};
use num::Float;
use std::fmt;
use std::ops::*;

pub struct SpatialValue<T> {
    rotation: Vector3<T>,
    translation: Vector3<T>,
}

impl<T:Float> SpatialValue<T> {
    fn new(rotation: Vector3<T>, translation: Vector3<T>) -> Self {
        Self {
            rotation,
            translation,
        }
    }
}

pub struct Velocity<T> {
    value: SpatialValue<T>,
}

impl<T: Float> Velocity<T> {
    fn new(rotation: Vector3<T>, translation: Vector3<T>) -> Self {
        Velocity {
            value: SpatialValue {
                rotation,
                translation,
            },
        }
    }
}

pub struct Acceleration<T> {
    value: SpatialValue<T>,
}

impl<T: Float> Acceleration<T> {
    fn new(rotation: Vector3<T>, translation: Vector3<T>) -> Self {
        Acceleration {
            value: SpatialValue {
                rotation,
                translation,
            },
        }
    }
}

impl<T: Float> fmt::Display for SpatialValue<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "rotation: {} translation: {}",
            self.rotation, self.translation
        )
    }
}

pub struct Inertia<T> {
    mass: T,
    center_of_mass: Vector3<T>,
    inertia: Matrix3<T>,
    value: Matrix6<T>,
}

fn skew<T>(x: Vector3<T>) -> Matrix3<T> {
    Matrix3::new(0.0, x[2], -x[1], -x[2], 0.0, x[0], x[1], -x[0], 0.0)
}

fn concat_block_2x2<T>(
    b1: Matrix3<T>,
    b2: Matrix3<T>,
    b3: Matrix3<T>,
    b4: Matrix3<T>,
) -> Matrix6<T> {
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

impl<T: Float> Inertia<T> {
    fn new(mass: T, center_of_mass: Vector3<T>, inertia: Matrix3<T>) -> Inertia<T> {
        let c = skew(center_of_mass);
        let ct = c.transpose();
        let b1 = inertia + mass * c * ct;
        let b2 = mass * ct;
        let b3 = mass * c;
        let b4 = Matrix3::from_diagonal_element(mass);
        let value = concat_block_2x2(b1, b2, b3, b4);
        Inertia {
            mass,
            center_of_mass,
            inertia,
            value: value,
        }
    }
}

impl<T: Float> Mul<Velocity<T>> for Inertia<T> {
    type Output = Momentum<T>;

    fn mul(self, velocity: Velocity<T>) -> Momentum<T> {
        let inertia = self.value.fixed_view::<3, 3>(0, 0);
        let mass = self.value[self.value.len() - 1];
        let angular_momentum = inertia * velocity.0.rotation;
        let linear_momentum = mass * velocity.0.translation;
        Momentum::new(angular_momentum, linear_momentum)
    }
}
