include("..\\core.jl")

N = WorldFrame()
g = DOF6(:g,[0.,0.,0.,1.],[0.,0.,0.],zeros(3),[0.,0.,0.])
b = Body(:b,1,I(3),zeros(3))
u = Connection(N,Cartesian(I(3),zeros(3)),b,Cartesian(I(3),zeros(3)),g)

B = OffsetArray([N,b], 0:1)

sys = System(:body,N,B,[g],[u])