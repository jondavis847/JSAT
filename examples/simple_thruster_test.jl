includet("..//src/jsat.jl")

function simple_thruster_test()
    N = BaseFrame(:N)
    #J = FloatingJoint(:J)
    J = Revolute(:J)
    B = Body(:B,10,zeros(3),10*I(3))
    connect!(J,N,B,eye(Cartesian),Cartesian(I(3),[0,1,0]))

    T = SimpleThruster(:T, 25)
    connect!(B,T, Cartesian(I(3),[0,-1,0]))

    C = TimedCommand(:C,false,[1,5],[1.5,5.5])
    
    connect!(T,C)

    sys = MultibodySystem(:sys,N,B,J,T,C)
    sol = simulate(sys,(0,10); output_type = DataFrame)

    return sys,sol
end