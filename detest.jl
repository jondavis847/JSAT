using DifferentialEquations, StaticArrays

Q = @SMatrix [
    q[4] -q[3] q[2]
    q[3] q[4] -q[1]
    -q[2] q[1] q[4]
    -q[1] -q[2] -q[3]
]
dx.body.q = 0.5 * Q * x.body.ω