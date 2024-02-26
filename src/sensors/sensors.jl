mutable struct SensorConnection 
    body::Body # body sensor is attached to
    reference::Body # reference body for sensor to compare to body to reference (usually base but maybe a different body)
    body_to_sensor::SMatrix{6,6,Float64,36}    
    SensorConnection() = new()
end

# attach a sensor to a body with some frame
function connect!(S::AbstractSensor,B::Body,F::Cartesian = Cartesian(I(3),[0,0,0]))
    S.connection.body = B
    S.connection.body_to_sensor = F #spatial motion transform from body frame to sensor frame    
    push!(B.models.sensors,S)
    return nothing
end

# set the reference frame for a sensor to some other body
function connect!(B::Body,S::AbstractSensor,F::Cartesian = Cartesian(I(3),[0,0,0]))
    S.connection.body = B
    S.connection.body_to_sensor = F #spatial motion transform from body frame to sensor frame    
    push!(B.models.sensors,S)
    return nothing
end

get_callback(S::AbstractSensor) = S.callback

includet("SimpleAttitudeSensor.jl")
includet("SimpleRateSensor.jl")
