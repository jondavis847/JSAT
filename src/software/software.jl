mutable struct SoftwareConnection
    sensors::Vector{AbstractSensor}
    software::Vector{AbstractSoftware}
    actuators::Vector{AbstractActuator}
    SoftwareConnection() = new(AbstractSensor[],AbstractSoftware[],AbstractActuator[])
end

function connect!(S::AbstractSensor,SW::AbstractSoftware) 
    push!(SW.connections.sensors,S)
    return nothing 
end

includet("TimedCommand.jl")
includet("CustomSoftware.jl")

