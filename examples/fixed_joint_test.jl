includet("..\\src\\jsat.jl")
function fixed_joint_test()
    N = BaseFrame(:N,-9.8)
    j1 = Revolute(:j1,pi/4,0.)
    b1 = Body(:p1,1,zeros(3),I(3))
    fp1 = eye(Cartesian)
    fs1 = Cartesian(I(3),[0,1,0])

    connect!(j1,N,b1,fp1,fs1)

    j2 = FixedJoint(:j2,[0,0,0,1],[1,0,0])
    b2 = Body(:p2,1,zeros(3),I(3))
    fp2 = Cartesian(I(3),[0,-1,0])
    fs2 = Cartesian(I(3),zeros(3))        
    connect!(j2,b1,b2,fp2,fs2)

    sys = MultibodySystem(:fjt,N,[b1,b2],[j1,j2])

    sol = simulate(sys,(0,10); output_type = DataFrame)
    return sol
end