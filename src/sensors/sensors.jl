includet("SimpleAttitudeSensor.jl")
includet("SimpleRateSensor.jl")


function connect!(B::Body,S::AbstractSensor) 
    S.body = B 
    push!(B.models.sensors, S)   
    return nothing
end

get_callback(S::AbstractSensor) = S.callback