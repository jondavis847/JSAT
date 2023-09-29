include("core.jl")

# Frame → validation
# Transform a frame from Identity to a prescribed frame, multiply an axis vector by to confirm the expected result 

F1 = Cartesian(I(3),zeros(3)) # identity frame
F2 = Cartesian([0 -1 0; 0 0 1; -1 0 0], [1,1,0]) # another frame where x2 = -z1, y2 = -x1, z2 = y1, origin at [1,1,0] of parent frame

E = F1→F2

p = [2,-1,1] #random point for testing

@assert E*p == [-1,-1,-2] "test failed"


