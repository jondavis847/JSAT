includet("..\\src\\blue42.jl")

function body_wheel()

    N = WorldFrame()
    
    
    g1 = Revolute(:g1,0.,0.)        
    fp1 = eye(Cartesian)
    fs1 = eye(Cartesian)
    b = Body(:b,1000,1500*I(3),zeros(3))
    connect!(g1,N,b,fp1,fs1)

    g2 = Revolute(:g2,0.,pi/4)
    fp2 = Cartesian(I(3),[0,0,1])
    fs2 = eye(Cartesian)
    w = Body(:w,1,I(3),zeros(3))
    connect!(g2,b,w,fp2,fs2)

    B = OffsetArray([N,b,w], 0:2)
    
    sys = MultibodySystem(:body,B,[g1,g2])

    sol = simulate(sys,(0,10); output_type = DataFrame)
    return sol
end