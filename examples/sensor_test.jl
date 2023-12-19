includet("..//src//jsat.jl")

function sensor_test()
    N = BaseFrame(:N,0)
    J1 = FloatingJoint(:J1,[0,0,0,1],rand(3),zeros(3),zeros(3))
    B1 = Body(:B1,1,zeros(3),I(3))
    connect!(J1,N,B1)

    A = SimpleAttitudeSensor4(:A)
    R = SimpleRateSensor3(:R)

    connect!(A,B1)
    connect!(R,B1)

    sys = MultibodySystem(:sys,N,B1,J1,sensors = [A,R])
    sol = simulate(sys,(0,10),output_type = NamedTuple)
    return (sys,sol)
end