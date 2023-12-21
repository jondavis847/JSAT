mutable struct SensorConnection 
    sensed_source::Body
    transform_source::SMatrix{6,6,Float64,36}
    reference::Body
    SensorConnection() = new()
end

function connect!(B::Body,S::AbstractSensor,F::Cartesian = Cartesian(I(3),[0,0,0]))
    S.connection.sensed_source = B
    S.connection.transform_source = ℳ(F) #spatial motion transform from body frame to sensor frame    
    push!(B.models.sensors,S)
    return nothing
end

function connect!(S::AbstractSensor,B::Body)
    S.connection.reference = B    
    return nothing
end

get_callback(S::AbstractSensor) = S.callback

includet("SimpleAttitudeSensor.jl")
includet("SimpleRateSensor.jl")
