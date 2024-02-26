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

mutable struct RevoluteLocks
    lower::Bool
    upper::Bool
    RevoluteLocks() = new(false,false)
end

struct RevoluteParameters
    f::Float64 # constant force
    κ::Float64 # spring constant
    ζ::Float64 # dampening parameter
    pos_upper_limit::Float64
    pos_lower_limit::Float64
end

mutable struct Revolute <: AbstractJoint
    meta::JointMeta
    state::RevoluteState
    parameters::RevoluteParameters
    connection::JointConnection
    frame::Cartesian
    locks::RevoluteLocks
    locked::Bool
    S::SMatrix{6,1,Float64}
end

function Revolute(name, θ=0.0, ω=0.0; f=0.0, κ=0.0, ζ=0.0, pos_upper_limit=Inf, pos_lower_limit=-Inf, locked = false)
    jm = JointMeta(name, 1, 1)
    js = RevoluteState(θ, ω, SVector{1,Float64}(0), SVector{1,Float64}(0))
    jp = RevoluteParameters(f, κ, ζ, pos_upper_limit, pos_lower_limit)
    S = SMatrix{6,1,Float64}(0, 0, 1, 0, 0, 0)
    joint = Revolute(jm, js, jp, JointConnection(), eye(Cartesian), RevoluteLocks(), locked, S)
    update_joint_frame!(joint, θ)
    return joint
end

function calculate_τ!(G::Revolute)
    if G.locked
        G.state.τ = SVector{1,Float64}(0.0)
    else
        G.state.τ = SVector{1,Float64}(G.parameters.f - G.parameters.κ * G.state.θ - G.parameters.ζ * G.state.ω)
    end
    return nothing
end

get_S(G::Revolute) = !G.locked * G.S

get_q(G::Revolute) = SVector{1,Float64}(G.state.θ)
get_q̇(G::Revolute) = SVector{1,Float64}(G.state.ω)
get_dq(G::Revolute) = SVector{1,Float64}(G.state.ω)

function set_state!(G::Revolute, x)
    q = @view x[G.meta.xindex]
    q̇ = @view x[G.meta.ẋindex]

    G.state.θ = q[1]
    G.state.ω = q̇[1]
    update_joint_frame!(G::Revolute, G.state.θ)
    nothing
end

function update_joint_frame!(G::Revolute, θ=G.state.θ)
    R = SA[
        cos(θ) sin(θ) 0
        -sin(θ) cos(θ) 0
        0 0 1
    ]
    G.frame = Cartesian(R, @SVector zeros(3))
    nothing
end

function get_callback(J::Revolute,i)
    callbacks = []
    function pos_lower_lock_condition(u,t,integrator)
        joint = integrator.p.sys.joints[i]
        θ = u[joint.meta.xindex][1]
        !joint.locked && (θ < joint.parameters.pos_lower_limit)        
    end
    function pos_lower_lock_affect!(integrator)           
        xindex = integrator.p.sys.joints[i].meta.xindex
        ẋindex = integrator.p.sys.joints[i].meta.ẋindex
        integrator.u[xindex] .= integrator.p.sys.joints[i].parameters.pos_lower_limit
        integrator.u[ẋindex] .= 0.0
        integrator.p.sys.joints[i].locks.lower = true
        integrator.p.sys.joints[i].locked = true
        return nothing
    end
    push!(callbacks, DiscreteCallback(pos_lower_lock_condition,pos_lower_lock_affect!))
    
    function pos_lower_unlock_condition(u,t,integrator)
        joint = integrator.p.sys.joints[i]
        ω = u[joint.meta.ẋindex][1]
        joint.locks.lower && (ω > 0.0)
    end
    function pos_lower_unlock_affect!(integrator)                 
        integrator.p.sys.joints[i].locks.lower = false
        integrator.p.sys.joints[i].locked = false
        return nothing
    end
    push!(callbacks, DiscreteCallback(pos_lower_unlock_condition,pos_lower_unlock_affect!))

    function pos_upper_lock_condition(u,t,integrator)
        joint = integrator.p.sys.joints[i]
        θ = u[joint.meta.xindex][1]
        !joint.locked && (θ > joint.parameters.pos_upper_limit)        
    end
    function pos_upper_lock_affect!(integrator)              
        xindex = integrator.p.sys.joints[i].meta.xindex
        ẋindex = integrator.p.sys.joints[i].meta.ẋindex
        integrator.u[xindex] .= integrator.p.sys.joints[i].parameters.pos_upper_limit
        integrator.u[ẋindex] .= 0.0
        integrator.p.sys.joints[i].locks.upper = true
        integrator.p.sys.joints[i].locked = true        
        return nothing
    end
    push!(callbacks, DiscreteCallback(pos_upper_lock_condition,pos_upper_lock_affect!))

    function pos_upper_unlock_condition(u,t,integrator)
        joint = integrator.p.sys.joints[i]
        ω = u[joint.meta.ẋindex][1]
        joint.locks.upper && (ω < 0.0)
    end
    function pos_upper_unlock_affect!(integrator)                 
        integrator.p.sys.joints[i].locks.upper = false
        integrator.p.sys.joints[i].locked = false
        return nothing
    end
    push!(callbacks, DiscreteCallback(pos_upper_unlock_condition,pos_upper_unlock_affect!))

    return callbacks
end