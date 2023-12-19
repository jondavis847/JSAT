mutable struct SimpleRateSensor <: AbstractSensor
    name::Symbol
    index::Int16 # 1 2 or 3 for x y or z
    rate::Float64
    body::Body
    SimpleRateSensor(name,index) = new(name,index)
end

function get_callback(S::SimpleRateSensor,i)
    affect! = (u,t,integrator) -> integrator.p.sys.sensors[i].rate = body.state.v_body[S.index]
    return FunctionCallingCallback(affect!)    
end

mutable struct SimpleRateSensor3 <: AbstractSensor
    name::Symbol
    rate::SVector{3,Float64}    
    body::Body
    SimpleRateSensor3(name) = new(name)
end

function get_callback(S::SimpleRateSensor3,i)
    affect! = (u,t,integrator) -> integrator.p.sys.sensors[i].rate = integrator.p.sys.sensors[i].body.state.v_body[SVector{3,Int16}(1,2,3)]
    return FunctionCallingCallback(affect!)    
end

function get_savedict(S::SimpleRateSensor3,i)
    save_config = Dict[]
    save_dict!(
        save_config,
        "$(S.name)_rate",
        typeof(S.rate),
        integrator -> integrator.p.sys.sensors[i].rate
    )
    return save_config
end