includet("..\\core.jl")

N = WorldFrame()
hinge = Revolute(:hinge,0.,0.)
pendulum = Body(:pend,1,I(3),zeros(3))

Fp = Cartesian(I(3),[0,1,0])
Fs = Cartesian(I(3),[1,0,0])

connect!(hinge,N,pendulum,Fp,Fs)

B = OffsetArray([N,pendulum], 0:1) #offset array lets us do 0 based indexing which matches featherstone notation

sys = MultibodySystem(:pend,B,[hinge])

sol = simulate(sys,(0,10))