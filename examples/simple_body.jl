includet("..\\src\\blue42.jl")

N = WorldFrame()
g = DOF6(:g,[0.,0.,0.,1.],[1.,0.,0.],[0.,0.,0.],[0.,-1.,0.])
b = Body(:b,1,I(3),[0.,0.,0.])
Fp = eye(Cartesian)
Fs = eye(Cartesian)
connect!(g,N,b,Fp,Fs)

B = OffsetArray([N,b], 0:1)

sys = MultibodySystem(:body,B,[g])
sol = simulate(sys,(0,10));