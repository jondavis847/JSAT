mutable struct SimpleRateSensor <: AbstractSensor
    name::Symbol
    index::Int16 # 1 2 or 3 for x y or z
    connection::SensorConnection
    rate::Float64
    SimpleRateSensor(name, index) = new(Symbol(name), index, SensorConnection())
end

function get_callback(S::SimpleRateSensor, i)
    affect! = function (u,t,integrator)
        body_to_sensor_transform = integrator.p.sys.sensors[i].body_to_sensor
        body_rate = integrator.p.sys.sensors[i].body.state.v_body
        sensed_rate = (body_to_sensor_transform*body_rate)[S.index]
        integrator.p.sys.sensors[i].rate = sensed_rate
        return nothing
    end
    return FunctionCallingCallback(affect!)
end

mutable struct SimpleRateSensor3 <: AbstractSensor
    name::Symbol    
    connection::SensorConnection
    rate::SVector{3,Float64}
    SimpleRateSensor3(name) = new(Symbol(name),SensorConnection())
end

function get_callback(S::SimpleRateSensor3, i)
    affect! = function (u,t,integrator)
        body_to_sensor_transform = integrator.p.sys.sensors[i].connection.body_to_sensor
        body_rate = integrator.p.sys.sensors[i].connection.sensed_source.state.v_body
        sensed_rate = (body_to_sensor_transform*body_rate)[SVector{3,Int16}(1, 2, 3)]
        integrator.p.sys.sensors[i].rate = sensed_rate
        return nothing
    end
    return FunctionCallingCallback(affect!)
end

function get_savedict(S::SimpleRateSensor3, i)
    save_config = Dict[]
    save_dict!(
        save_config,
        "$(S.name)_rate",
        typeof(S.rate),
        integrator -> integrator.p.sys.sensors[i].rate
    )
    return save_config
end