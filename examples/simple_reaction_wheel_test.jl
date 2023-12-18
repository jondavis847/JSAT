includet("..//src/jsat.jl")

function simple_reaction_wheel_test()
    N = BaseFrame(:N)
    J = FloatingJoint(:J)    
    B = Body(:B,10,zeros(3),10*I(3))
    connect!(J,N,B,eye(Cartesian),Cartesian(I(3),[0,0,0]))

    RW = SimpleReactionWheel(:RW, 1, 1)
    connect!(B,RW, Cartesian(I(3),[0,0,1]))

    C = TimedCommand(:C,false,[1],[6])
    
    connect!(RW,C)

    sys = MultibodySystem(:sys,N,B,J,RW,C)
    sol = simulate(sys,(0,10); output_type = DataFrame)

    return sys,sol
end