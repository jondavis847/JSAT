"""
Spherical Joint

    3DOF rotation

States:
    q - quaternion
    ω - angular rate
Joint frame:
    - right hand rule
    - x to the right, y up, z out of the page    
"""
mutable struct SphericalState <: AbstractJointState
    q::SVector{4,Float64}
    ω::SVector{3,Float64}
    τ::SVector{3,Float64}
    q̈::SVector{3,Float64}
end
mutable struct Spherical <: AbstractJoint
    meta::JointMeta
    state::SphericalState
    connection::JointConnection
    frame::Cartesian
    S::SMatrix{6,3,Float64}
end

function Spherical(name, q = [0.,0.,0.,1.], ω = zeros(3))
    jm = JointMeta(name, 4, 3)
    js = SphericalState(q, ω, SVector{3,Float64}(zeros(3)),SVector{3,Float64}(zeros(3)))
    S = SMatrix{6,3,Float64}([
        1 0 0
        0 1 0
        0 0 1
        0 0 0
        0 0 0
        0 0 0
    ])
    joint = Spherical(jm, js, JointConnection(),eye(Cartesian),S)
    update_joint_frame!(joint,q)
    return joint
end

get_q(G::Spherical) = SVector{4,Float64}(G.state.q)
get_q̇(G::Spherical) = SVector{3,Float64}(G.state.ω)
get_dq(G::Spherical) = quaternion_derivative(G.state.q, G.state.ω)

#need to make this spring/dampener/loss forces at some point
calculate_τ!(G::Spherical,pA) = nothing

function set_state!(G::Spherical, x)
    q = @view x[G.meta.xindex]
    q̇ = @view x[G.meta.ẋindex]

    G.state.q = q
    G.state.ω = q̇
    update_joint_frame!(G::Spherical)
    nothing
end

function update_joint_frame!(G::Spherical,q = G.state.q,r = @SVector zeros(3))    
    R = qtoa(q)
    G.frame = Cartesian(R,r)
    nothing
end
