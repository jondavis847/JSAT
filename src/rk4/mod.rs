use num::Float;
mod explicit;
pub use explicit::ExplicitRK;
mod dopri;
pub use dopri::dopri;

struct AdaptiveConfig {
    dt_max:f64,
    abs_tol:f64,
    rel_tol:f64,
    safety_factor:f64
}

fn calculate_time_step(dt:f64,error:f64,config:AdaptiveConfig) {    
    let new_dt = config.safety_factor * dt * f64::powf(config.tolerance/error, 0.2); // dt*(tol/err)^1/5
    if error > config.abs_tol {
        new_dt = ;
    }
}

pub fn calculate_step<T: Float>(f:&dyn Fn(f64,T) ->T, t:f64, dt:f64, y:T,tab:ExplicitRK, config:AdaptiveConfig) -> T 
where
T:Float
{
    let k1 = dt * f(t,y);
}