includet("..//src/blue42.jl")

function simple_thruster_test()
    N = WorldFrame()
    J = FloatingJoint(:J)
    b = Body(:B,1000,zeros(3),1000*I(3))
    connect!(J,N,b)

    T = SimpleThruster(:T, 25)
    connect!(b,T, Cartesian(I(3),[0,1,0]))

    C = TimedCommand(:C,false,[1,5],[1.5,5.5])
    
    connect!(T,C)

    B = OffsetArray([N,b], 0:1)

    sys = MultibodySystem(:sys,B,[J],[T],[C])
    sol = simulate(sys,(0,10); output_type = DataFrame)

    return sys,sol
end