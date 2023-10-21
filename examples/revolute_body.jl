include("..\\core.jl")

N = WorldFrame()
g = Revolute(:g,0.,0.)
b = Body(:b,1,I(3),[0.,0.,0.])
Fp = eye(Cartesian)
Fs = eye(Cartesian)
connect!(g,N,b,Fp,Fs)

B = OffsetArray([N,b], 0:1)

sys = MultibodySystem(:body,B,[g])