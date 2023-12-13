includet("..//src//blue42.jl")

function gravity_test()    
    N = WorldFrame(:N,-9.8)
    B = Body(:B,1,zeros(3),I(3))
    Fs = Cartesian(I(3),[0,1,0])
    J = Revolute(:J,pi/4,0)
    connect!(J,N,B,eye(Cartesian),Fs)

    sys = MultibodySystem(:sys,N,B,J)
    sol = simulate(sys,(0,10),output_type = DataFrame)
    return sys,sol
end
    