using LinearAlgebra,RigidBodyDynamics,StaticArrays

# create the world frame
N = RigidBody{Float64}("N")

bodywheel = Mechanism(N; gravity = [0,0,0])

# create the N to B joint
G1 = Joint("G1",  QuaternionFloating{Float64}())

# create the body
j_body = SpatialInertia(
    frame_after(G1),
    com = zeros(3),
    moment_about_com = 1.0 * I(3),
    mass = 1.
)
B = RigidBody(j_body)
#=
# create the B to W joint
G2 = Joint("G2",  Revolute{Float64}([0,0,1]))

# create the wheel
j_wheel = SpatialInertia(
    frame_after(G2),
    com = [0.,-1.,0.], #zeros(3),
    moment_about_com = 1.0 * I(3),
    mass = 1.
)
W = RigidBody(j_wheel)

# create joint connection in body frame
#F1i = Transform3D(frame_before(G1), default_frame(N), I(3),zeros(3)) # one means its identical
#F1o = Transform3D(frame_after(G1), default_frame(B), zeros(3))

=#
# attach all  the components
attach!(bodywheel,N,B,G1)
#attach!(bodywheel,N,W,G2)

#sim
state = MechanismState(bodywheel)
result = DynamicsResult(bodywheel)

set_configuration!(state, G1, [0,0,0,1,0,0,0]) # [q,r]
set_velocity!(state, G1, [1,0,0,0,-1,0]) # [ω,v]

#set_configuration!(state, G2, pi/4) # θ
#set_velocity!(state, G2, 0) #ω


function simple_control!(torques::AbstractVector, t, state::MechanismState)
    torques[velocity_range(state, G1)] .= -1000 * velocity(state,G1)    
end


#ts, qs, vs = simulate(state, 10., simple_control!, Δt = .1);
ts, qs, vs = simulate(state, 10., Δt = .1);