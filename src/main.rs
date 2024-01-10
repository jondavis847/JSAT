use nalgebra::{Matrix3, Matrix6, UnitQuaternion, Vector3, Vector6};

mod transforms;
pub mod spatial;

fn main() {
    
    let v: Vector3<f64> = Vector3::new_random();
    let w: Vector3<f64> = Vector3::new_random();
    let mv = spatial::velocity(w, v);

    print!("{}", mv);

    let m: f64 = 1.0;
    let cm: Vector3<f64> = Vector3::new_random();
    let i: Matrix3<f64> = Matrix3::new_random();
    let si = spatial::inertia(m, cm, i);

    let sm = si * mv;
    print!("{}",sm)
}
