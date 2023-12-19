mutable struct SimpleAttitudeSensor4 <: AbstractSensor
    name::Symbol
    attitude::SVector{4,Float64}    
    body::Body
    SimpleAttitudeSensor4(name) = new(name)
end

function get_callback(S::SimpleAttitudeSensor4,i)
    affect! = (u,t,integrator) -> integrator.p.sys.sensors[i].attitude = integrator.p.sys.sensors[i].body.state.q_base
    return FunctionCallingCallback(affect!)    
end
