"""
Revolute Joint

    1DOF rotation in the x-y plane about z

States:
    θ - rotation angle
    ω - angular rate
Joint frame:
    - right hand rule
    - x to the right, y up, z out of the page
    - θ referenced from +x        
"""
mutable struct RevoluteState <: AbstractJointState
    θ::Float64
    ω::Float64    
    τ::SVector{1,Float64}
    q̈::SVector{1,Float64}
end
mutable struct Revolute <: AbstractJoint
    meta::JointMeta
    state::RevoluteState
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,1,Float64}
end

function Revolute(name, θ = 0., ω = 0.)
    jm = JointMeta(name, 1, 1)
    js = RevoluteState(θ, ω, SVector{1,Float64}(0),SVector{1,Float64}(0))
    S = SMatrix{6,1,Float64}(0,0,1,0,0,0)
    joint = Revolute(jm, js, JointConnection(),eye(Cartesian),S)
    update_joint_frame!(joint,θ)
    return joint
end


get_q(G::Revolute) = SVector{1,Float64}(G.state.θ)
get_q̇(G::Revolute) = SVector{1,Float64}(G.state.ω)
get_dq(G::Revolute) = SVector{1,Float64}(G.state.ω)

function set_state!(G::Revolute, x)
    q = @view x[G.meta.xindex]
    q̇ = @view x[G.meta.ẋindex]

    G.state.θ = q[1]
    G.state.ω = q̇[1]    
    update_joint_frame!(G::Revolute,G.state.θ)
    nothing
end

function update_joint_frame!(G::Revolute, θ = G.state.θ)
    R = SA[
        cos(θ) sin(θ) 0 
        -sin(θ) cos(θ) 0 
        0 0 1
    ]
    G.frame = Cartesian(R,@SVector zeros(3))
    nothing
end

#need to make this spring/dampener/loss forces at some point
calculate_τ!(G::Revolute) = nothing
