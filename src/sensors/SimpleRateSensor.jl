mutable struct SimpleRateSensor
    name::Symbol
    index::Int16 # 1 2 or 3 for x y or z
    rate::Float64
    body::Body
    SimpleRateSensor(name,index) = new(name,index)
end

function get_callback(S::SimpleRateSensor,i)
    affect! = (integrator) -> integrator.p.sys.sensors[i].rate = body.state.ω_body[S.index]
    return FunctionCallingCallback(affect!)    
end

mutable struct SimpleRateSensor3
    name::Symbol
    rate::SVector{3,Float64}    
    body::Body
    SimpleRateSensor3(name) = new(name)
end

function get_callback(S::SimpleRateSensor3,i)
    affect! = (integrator) -> integrator.p.sys.sensors[i].rate = body.state.ω_body
    return FunctionCallingCallback(affect!)    
end