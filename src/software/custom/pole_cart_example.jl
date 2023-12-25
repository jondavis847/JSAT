
function create_pole_cart_software()

    function pole_cart_software(software)
        attitude_sensor = software.connections.sensors[getfield.(software.connections.sensors, :name).==:attitude][1]
        rate_sensor = software.connections.sensors[getfield.(software.connections.sensors, :name).==:rate][1]

        left_thruster = software.connections.actuators[getfield.(software.connections.actuators, :name).==:left_thruster][1]
        right_thruster = software.connections.actuators[getfield.(software.connections.actuators, :name).==:right_thruster][1]

        attitude_reference = @SVector zeros(3)
        rate_reference = @SVector zeros(3)

        software.variables.attitude_error = (qtoe(attitude_sensor.attitude)-attitude_reference)[3]
        software.variables.rate_error = (rate_sensor.rate-rate_reference)[3]

        software.variables.accel_cmd = -(
            software.parameters.kp * software.variables.attitude_error +
            software.parameters.kd * software.variables.rate_error +
            software.parameters.ki * software.variables.integral_error)

        software.variables.accel_cmd > 1 ? left_thruster.command = true : left_thruster.command = false
        software.variables.accel_cmd < -1 ? right_thruster.command = true : right_thruster.command = false

        return nothing
    end

    fsw_parameters = (kp=-10, kd=0, ki=0)
    fsw_variables = ComponentArray(attitude_error=0.0, rate_error=0.0, integral_error=0.0, accel_cmd=0.0)
    required_sensors = [:attitude,:rate]
    required_actuators = [:left_thruster,:right_thruster]
    fsw = CustomSoftware(:fsw, pole_cart_software, 0.1, fsw_parameters, fsw_variables, required_sensors, required_actuators)

    return fsw
end