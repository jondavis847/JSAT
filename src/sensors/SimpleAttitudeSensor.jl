#= need to figure out if we want this on a joint
mutable struct SimpleAttitudeSensor <: AbstractSensor
    name::Symbol
    attitude::Float64
    body::Body
    SimpleAttitudeSensor(name) = new(name)
end
=#

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

function get_savedict(S::SimpleAttitudeSensor4,i)
    save_config = Dict[]
    save_dict!(
        save_config,
        "$(S.name)_quaternion",
        typeof(S.attitude),
        integrator -> integrator.p.sys.sensors[i].attitude
    )
    return save_config
end
