includet("..//src//jsat.jl")

function pole_cart_test()
    N = BaseFrame(:N,-9.8)
    J1 = Prismatic(:J1)
    B1 = Body(:B1,10,zeros(3),10*I(3))
    connect!(J1,N,B1)

    J2 = Revolute(:J2,5*pi/180,0)
    B2 = Body(:B2,10,zeros(3),diagm([50,5,50]))
    connect!(J2,B1,B2,Cartesian(I(3),[0,-0.5,0]),Cartesian(I(3),[0,0.5,0]))

    A = SimpleAttitudeSensor4(:A)
    R = SimpleRateSensor3(:R)

    connect!(B2,A)
    connect!(B2,R)

    TL = SimpleThruster(:TL,100)
    TR = SimpleThruster(:TR,100)

    connect!(B1,TL,Cartesian([0 0 -1 ; 0 1 0 ; 1 0 0],zeros(3)))
    connect!(B1,TR,Cartesian([0 0 1 ; 0 1 0 ; -1 0 0],zeros(3)))

    fsw_parameters = (kp = 1, kd = 0, ki = 0)
    fsw_variables = ComponentArray(attitude_error = 0.0, rate_error = 0.0, integral_error = 0.0, accel_cmd = 0.0)
    SW = CustomSoftware(:fsw,pole_cart_software,0.1,fsw_parameters,fsw_variables)

    connect!(A,SW)
    connect!(R,SW)
    connect!(SW,TL)
    connect!(SW,TR)

    sys = MultibodySystem(:sys, N, [B1,B2], [J1,J2], sensors=[A,R], actuators=[TL,TR], software=SW)
    return sys
end

function  pole_cart_software(software)
    attitude_sensor = software.connections.sensors[getfield.(software.connections.sensors,:name) .== :A][1]
    rate_sensor = software.connections.sensors[getfield.(software.connections.sensors,:name) .== :R][1]

    left_thruster = software.connections.actuators[getfield.(software.connections.actuators,:name) .== :TL][1]
    right_thruster = software.connections.actuators[getfield.(software.connections.actuators,:name) .== :TR][1]

    attitude_reference = @SVector zeros(3)
    rate_reference = @SVector zeros(3)

    software.variables.attitude_error = (qtoe(attitude_sensor.attitude) - attitude_reference)[3]
    software.variables.rate_error = (rate_sensor.rate - rate_reference)[3]

    software.variables.accel_cmd = - (
        software.parameters.kp*software.variables.attitude_error + 
        software.parameters.kd*software.variables.rate_error + 
        software.parameters.ki* software.variables.integral_error)

    software.variables.accel_cmd > 1 ? left_thruster.command = true : left_thruster.command = false
    software.variables.accel_cmd < -1 ? right_thruster.command = true : right_thruster.command = false

    return nothing
end
