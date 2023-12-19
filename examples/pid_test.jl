includet("..//src//jsat.jl")

function pid_test()
    N = BaseFrame(:N,-9.8)
    J1 = Prismatic(:J1,0,0)
    B1 = Body(:B1,100,zeros(3),100*I(3))
    connect!(J1,N,B1)

    J2 = Revolute(:J2,80*pi/180,0)
    B2 = Body(:B2,10,zeros(3),[50,5,50])
    connect!(J2,B1,B2,eye(Cartesian),Cartesian(I(3),[0,-2,0]))

    A = SimpleAttitudeSensor(:A)
    R = SimpleRateSensor(:R,3)

    connect!(A,B2)
    connect!(R,B2)

    sys = MultibodySystem
end