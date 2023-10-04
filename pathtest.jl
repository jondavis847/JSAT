includet("core.jl")

B = OffsetArray(Vector{AbstractBody}(undef,11),0:10)
G = Vector{Joint}(undef,10)
U = Vector{Connection}(undef,10)

B[0] = N = WorldFrame()

G[1] = DOF6(:G1)
B[1] = Body(:B1,10,10*I(3),zeros(3))
U[1] = Connection(N,Cartesian(I(3),zeros(3)),B[1],Cartesian(I(3),zeros(3)),G[1])

B[2] = Body(:B2,10,10*I(3),zeros(3))
G[2] = Revolute(:G2)
U[2] = Connection(B[1],Cartesian(I(3),zeros(3)),B[2],Cartesian(I(3),zeros(3)),G[2])

B[3] = Body(:B3,10,10*I(3),zeros(3))
G[3] = Revolute(:G3)
U[3] = Connection(B[1],Cartesian(I(3),zeros(3)),B[3],Cartesian(I(3),zeros(3)),G[3])

B[4] = Body(:B4,10,10*I(3),zeros(3))
G[4] = Revolute(:G4)
U[4] = Connection(B[2],Cartesian(I(3),zeros(3)),B[4],Cartesian(I(3),zeros(3)),G[4])

B[5] = Body(:B5,10,10*I(3),zeros(3))
G[5] = Revolute(:G5)
U[5] = Connection(B[2],Cartesian(I(3),zeros(3)),B[5],Cartesian(I(3),zeros(3)),G[5])

B[6] = Body(:B6,10,10*I(3),zeros(3))
G[6] = Revolute(:G6)
U[6] = Connection(B[3],Cartesian(I(3),zeros(3)),B[6],Cartesian(I(3),zeros(3)),G[6])

G[7] = DOF6(:G7)
B[7] = Body(:B7,10,10*I(3),zeros(3))
U[7] = Connection(N,Cartesian(I(3),zeros(3)),B[7],Cartesian(I(3),zeros(3)),G[7])

B[8] = Body(:B8,10,10*I(3),zeros(3))
G[8] = Revolute(:G8)
U[8] = Connection(B[7],Cartesian(I(3),zeros(3)),B[8],Cartesian(I(3),zeros(3)),G[8])

B[9] = Body(:B9,10,10*I(3),zeros(3))
G[9] = Revolute(:G9)
U[9] = Connection(B[7],Cartesian(I(3),zeros(3)),B[9],Cartesian(I(3),zeros(3)),G[9])

G[10] = DOF6(:G10)
B[10] = Body(:B10,10,10*I(3),zeros(3))
U[10] = Connection(N,Cartesian(I(3),zeros(3)),B[10],Cartesian(I(3),zeros(3)),G[10])

sys = System(:pathtest,N,B,G,U)

nothing