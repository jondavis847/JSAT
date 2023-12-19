mutable struct SimpleAttitudeSensor
    name::Symbol
    attitude::SVector{4,Float64}    
    body::Body
    SimpleAttitudeSensor(name) = new(name)
end

function get_callback(S::SimpleAttitudeSensor,i)
    affect! = (integrator) -> integrator.p.sys.sensors[i].attitude = body.state.q_base
    return FunctionCallingCallback(affect!)    
end