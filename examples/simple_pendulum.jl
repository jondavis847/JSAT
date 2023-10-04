include("..\\core.jl")

N = WorldFrame()
hinge = Revolute(:hinge)
pendulum = Body(:pend,1,I(3),zeros(3))
Fi = Cartesian(I(3),[0,1,0])
Fo = Cartesian(I(3),[1,0,0])
U = Connection(N,Fi,pendulum,Fo,hinge)

B = OffsetArray([N,pendulum], 0:1)


sys = System(:pend,N,B,[hinge],[U])