mutable struct SimpleAttitudeSensor <: AbstractSensor
    name::Symbol
    connection::SensorConnection
    attitude::Float64        
    SimpleAttitudeSensor(name) = new(Symbol(name))
end
mutable struct SimpleAttitudeSensor4 <: AbstractSensor
    name::Symbol    
    connection::SensorConnection    
    attitude::SVector{4,Float64}    
    SimpleAttitudeSensor4(name) = new(Symbol(name),SensorConnection())
end

function get_callback(S::SimpleAttitudeSensor4, i)
    i3 = SVector{3,Int16}(1, 2, 3)
    affect! = function (u,t,integrator)
        sensor = integrator.p.sys.sensors[i]
        body_to_sensor_transform = sensor.connection.transform_source[i3,i3]
        q_body_to_sensor = atoq(body_to_sensor_transform)
        
        q_base_to_body = sensor.connection.sensed_source.state.q_base
        if isdefined(sensor.connection, :reference)
            q_reference_to_body = qmult(q_base_to_body,qinv(sensor.connection.reference.state.q_base)) #TODO: Check this
        else
            q_reference_to_body = q_base_to_body #TODO: Check this
        end
        q_reference_to_sensor = qmult(q_body_to_sensor,q_reference_to_body)

        integrator.p.sys.sensors[i].attitude = q_reference_to_sensor
        return nothing
    end
    return FunctionCallingCallback(affect!)
end

function get_savedict(S::SimpleAttitudeSensor4,i)
    save_config = Dict[]
    save_dict!(
        save_config,
        "$(S.name)_attitude",
        typeof(S.attitude),
        integrator -> integrator.p.sys.sensors[i].attitude
    )
    return save_config
end
