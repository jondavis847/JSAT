includet("SimpleAttitudeSensor.jl")
includet("SimpleRateSensor.jl")


function connect!(S::AbstractSensor,B::Body) 
    S.body = B 
    push!(B.models.sensors, S)   
    return nothing
end

function connect!(SW::AbstractSoftware,S::AbstractSensor) 
    push!(SW.sensors,S)
    return nothing 
end

get_callback(S::AbstractSensor) = S.callback