includet("..\\src\\blue42.jl")
function fixed_joint_test()
    N = BaseFrame()
    j1 = Revolute(:j1,pi/4,0.)
    b1 = Body(:p1,1,I(3),zeros(3))
    fp1 = eye(Cartesian)
    fs1 = Cartesian(I(3),[0,1,0])

    connect!(j1,N,b1,fp1,fs1)

    j2 = FixedJoint(:j2,[0,0,0,1],[1,0,0])
    b2 = Body(:p2,1,I(3),zeros(3))
    fp2 = Cartesian(I(3),[0,-1,0])
    fs2 = Cartesian(I(3),zeros(3))        
    connect!(j2,b1,b2,fp2,fs2)

    B = OffsetArray([N,b1,b2], 0:2) #offset array lets us do 0 based indexing which matches featherstone notation
    G = [j1,j2]

    sys = MultibodySystem(:fjt,B,G)

    #sol = simulate(sys,(0,10); output_type = DataFrame)
    #return sol
end