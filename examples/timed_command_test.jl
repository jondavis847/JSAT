includet("..\\src\\blue42.jl")

function timed_command_test()

    N = WorldFrame()
    
    
    g1 = Revolute(:g1,0.,0.)        
    fp1 = eye(Cartesian)
    fs1 = eye(Cartesian)
    b = Body(:b,1000,zeros(3),1500*I(3))
    connect!(g1,N,b,fp1,fs1)
    
    B = OffsetArray([N,b], 0:1)
    
    tc1 = TimedCommand(:tc1,false,[1,3,5],[2,4,6])
    tc2 = TimedCommand(:tc2,true,[3,5,7],[2,4,6])

    sys = MultibodySystem(:body,B,[g1],[],[tc1,tc2])

    sol = simulate(sys,(0,10); output_type = DataFrame)
    return sol
end