using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays, DataFrames, OffsetArrays
import Base: show

includet("utils//quaternion.jl")
includet("spatial.jl")

abstract type AbstractBody end

Base.@kwdef struct WorldFrame <: AbstractBody
    name::Symbol = :N
    id::Int64 = 0
end

Base.@kwdef mutable struct Body <: AbstractBody
    name::Union{String,Symbol}
    #parameters
    m::Float64 # mass
    I::SMatrix{3,3,Float64} # inertia tensor
    cm::SVector{3,Float64} # center of mass    
    id::Int64 = 0 # body number for table designation (applied automatically by sys)
    Body(name, m, I, cm) = new(name, m, SMatrix{3,3,Float64}(I), SVector{3,Float64}(cm))
end

includet("joints.jl")
includet("utils//pathutils.jl")

struct MultibodySystem
    name::Union{String,Symbol}      
    bodies::OffsetVector{AbstractBody, Vector{AbstractBody}}
    joints::Vector{AbstractJoint}    
    p::Vector{Int16} # joint predecessor body array
    s::Vector{Int16} # joint successor body array
    λ::Vector{Int16} # body parent array
    κ::Vector{Vector{Int16}} # all joints between body i and base
    μ::OffsetVector{Vector{Int16}} # body children array
    γ::OffsetVector{Vector{Int16}} # all bodies from body i to tip (this is nu in Featherstone but nu just looks like v in Julia, so \gamma it is)
    ᵖXᵢᵐ::Vector{SpatialTransform} #spatial motion transform from body i to predecessor of body i
    ᵖXᵢᶠ::Vector{SpatialTransform} #spatial force transform from body i to predecessor of body i
    ⁱXₚᵐ::Vector{SpatialTransform} #spatial motion transform from predecessor of body i to body i
    ⁱXₚᶠ::Vector{SpatialTransform} #spatial force transform from predecessor of body i to body i       
    ᵒXᵢᵐ::OffsetVector{SpatialTransform} #spatial motion transform from body i to worldframe
    ᵒXᵢᶠ::OffsetVector{SpatialTransform} #spatial force transform from body i to worldframe
    ⁱXₒᵐ::OffsetVector{SpatialTransform} #spatial motion transform from worldframe to body i
    ⁱXₒᶠ::OffsetVector{SpatialTransform} #spatial force transform from worldframe to body i         
    q::MVector #generalized coordinates
    q̇::MVector #generalized speeds
    q̈::MVector #generalized accel
    x::MVector #ode state vector = [q;q̇]
    H::MMatrix #generalized mass matrix
    C::MVector #generalized bias force
    τ::MVector #generalized applied force (control force only for my application?)
    fˣ::MVector #external forces applied in the inertial frame (need a separate force for body force? not in featherstones equations but makes sense)
    Iᵇ::Vector{SpatialInertia} #body inertia
    Iᶜ::Vector{SpatialInertia} #composite inertia (inertia of body and all subtree bodies combined)
    r::MVector #body frame spatial position
    v::OffsetVector #body frame spatial velocity
    a::OffsetVector #body frame spatial acceleration
    f::MVector #total generalized force?
end

#MultibodySystem constructor
function MultibodySystem(name, bodies, joints)
    # preallocate vectors/matrices at MultibodySystem definition, mutate in place during sim
    # map the bodies and joints 
    p, s, λ, κ, μ, γ = map_tree!(bodies,joints)  
    
    # sort arrays so we can index directly by ID    
    permute!(bodies,sortperm(map(x->x.id,bodies)))
    permute!(joints,sortperm(map(x->x.meta.id,joints)))

    Iᵇ,Iᶜ = initialize_inertias(bodies) # Iᵇ for now to not conflict with LinearAlgebra: I
    
    #grab q,q̇ initial conditions
    q,q̇,q̈,x =  initialize_state_vectors(joints)    

    nb = length(bodies)    

    # body frame spatial vectors 
    r = MVector{nb-1,SVector{7,Float64}}(fill(SVector{7,Float64}(zeros(7)),nb-1))
    v = OffsetVector(fill(SVector{6,Float64}(zeros(6)),nb),0:nb-1)
    a = OffsetVector(fill(SVector{6,Float64}(zeros(6)),nb),0:nb-1)
    f = MVector{nb-1,SVector{6,Float64}}(fill(SVector{6,Float64}(zeros(6)),nb-1))
    fˣ = MVector{nb-1,SVector{6,Float64}}(fill(SVector{6,Float64}(zeros(6)),nb-1))

    # generalized vectors
    nq̇ = length(q̇)
    τ = MVector{nq̇}(zeros(nq̇))
    C = MVector{nq̇,Float64}(zeros(nq̇))
    H = MMatrix{nq̇,nq̇,Float64}(zeros(nq̇,nq̇))
    
    # spatial transformations from body i to predecessor body
    identity_X = SpatialTransform(SMatrix{6,6,Float64}(I(6)))
    ᵖXᵢᵐ = fill(identity_X,nb-1)
    ᵖXᵢᶠ = fill(identity_X,nb-1)
    ⁱXₚᵐ = fill(identity_X,nb-1)
    ⁱXₚᶠ = fill(identity_X,nb-1)

    # spatial transformations from body i to worldframe
    # superscript little o here since can't start with 0 to represent 0th body   
    # could use n but it looks wierd since superscript is capital and subscript is lower case    
    ᵒXᵢᵐ = OffsetVector(fill(identity_X,nb),0:nb-1)
    ᵒXᵢᶠ = OffsetVector(fill(identity_X,nb),0:nb-1)
    ⁱXₒᵐ = OffsetVector(fill(identity_X,nb),0:nb-1)
    ⁱXₒᶠ = OffsetVector(fill(identity_X,nb),0:nb-1)
    
    MultibodySystem(name,bodies,joints,p,s,λ,κ,μ,γ,ᵖXᵢᵐ,ᵖXᵢᶠ,ⁱXₚᵐ,ⁱXₚᶠ,ᵒXᵢᵐ,ᵒXᵢᶠ,ⁱXₒᵐ,ⁱXₒᶠ,q,q̇,q̈,x,H,C,τ,fˣ,Iᵇ,Iᶜ,r,v,a,f)
end

function model!(sys)    
    #sensors!(sys)
    #software!(sys)
    #actuators!(sys)
    environments!(sys)
    dynamics!(sys)
    nothing
end

# uses Featherstone notation
function dynamics!(sys)  
    calculate_τ!(sys)  # generalized actuator forces
    calculate_X!(sys)  # rotations
    calculate_C!(sys)  # bias forces (gravity, environments, disturbances, gyroscopic, coriolis, etc.)
    calculate_H!(sys)  # mass matrix
    calculate_q̈!(sys)  # generalized acceleration
    nothing
end

calculate_X!(sys::MultibodySystem) = calculate_X!(sys.ᵖXᵢᵐ,sys.ᵖXᵢᶠ,sys.ⁱXₚᵐ,sys.ⁱXₚᶠ,sys.ᵒXᵢᵐ,sys.ᵒXᵢᶠ,sys.ⁱXₒᵐ,sys.ⁱXₒᶠ,sys.λ,sys.joints)
calculate_C!(sys::MultibodySystem) = inverse_dynamics!(sys.C,0*sys.q̈,sys.q̇,sys.fˣ,sys.Iᵇ,sys.ⁱXₚᵐ,sys.ⁱXₒᶠ,sys.ᵖXᵢᶠ,sys.v,sys.a,sys.f,sys.λ,sys.joints)
calculate_H!(sys::MultibodySystem) = forward_dynamics!(sys.H,sys.Iᶜ,sys.Iᵇ,sys.ᵖXᵢᶠ,sys.ⁱXₚᵐ,sys.λ,sys.joints)    

function calculate_τ!(τ) 
    #until we figure out how to do software
    for i in eachindex(τ)
        τ[i] = 0
    end
    nothing
end
calculate_τ!(sys::MultibodySystem) = calculate_τ!(sys.τ)

function calculate_X!(ᵖXᵢᵐ,ᵖXᵢᶠ,ⁱXₚᵐ,ⁱXₚᶠ,ᵒXᵢᵐ,ᵒXᵢᶠ,ⁱXₒᵐ,ⁱXₒᶠ,λ,joints)
    for i in eachindex(joints)                
        joint = joints[i]
        
        p_to_s = joint.connection.Fp → Cartesian(joint) → inv(joint.connection.Fs)
        s_to_p = joint.connection.Fs → inv(Cartesian(joint)) → inv(joint.connection.Fp)

        ᵖXᵢᵐ[i] = ℳ(p_to_s) 
        ᵖXᵢᶠ[i] = ℱ(p_to_s)
        ⁱXₚᵐ[i] = ℳ(s_to_p)
        ⁱXₚᶠ[i] = ℱ(s_to_p)
        
        ᵒXᵢᵐ[i] = ᵒXᵢᵐ[λ[i]] * ᵖXᵢᵐ[i]
        ᵒXᵢᶠ[i] = ᵒXᵢᶠ[λ[i]] * ᵖXᵢᶠ[i]
        ⁱXₒᵐ[i] = ᵖXᵢᵐ[i] * ⁱXₒᵐ[λ[i]] 
        ⁱXₒᶠ[i] = ᵖXᵢᶠ[i] * ⁱXₒᶠ[λ[i]] 
    end
    nothing
end

function forward_dynamics!(H,Iᶜ,Iᵇ,ᵖXᵢᶠ,ⁱXₚᵐ,λ,joints)    
    #Featherstone 6.2
    for i in eachindex(Iᶜ)
        Iᶜ[i] = Iᵇ[i]
    end
    for i in reverse(eachindex(joints))        
        if λ[i] != 0
            Iᶜ[λ[i]] += ᵖXᵢᶠ * Iᶜ * ⁱXₚᵐ
        end
        F = Iᶜ[i] * 𝒮(joints[i])
        H[joints[i].meta.q̇index,joints[i].meta.q̇index] = 𝒮(joints[i])'*F
        j = i
        while λ[j] != 0
            F = ᵖXᵢᶠ[j] * F
            j = λ[j]            
            H[joints[i].meta.q̇index,joints[j].meta.q̇index] = F' * 𝒮(joints[j])
            H[joints[j].meta.q̇index,joints[i].meta.q̇index] = H[joints[i].meta.q̇index,joints[j].meta.q̇index]'
        end
    end
    nothing
end

# Featherstone chapter 5
function inverse_dynamics!(τ,q̈,q̇,fˣ,I,ⁱXₚᵐ,ⁱXₒᶠ,ᵖXᵢᶠ,v,a,f,λ,joints)
    # note that τ will actually be C with q̈ = 0 for the C bias force calculation
    
    # we may need to add the S∘ term if translation looks really off in base 6dof joint!
    # see Featherstone example 4.5 for more details

    # v[0] already set to 0 as default, never touch
    # a[0] = already set to 0 as default, never touch 
    for i in eachindex(joints)        
        joint = joints[i]
        λi = λ[i]         
        S = 𝒮(joint)

        v[i] = ⁱXₚᵐ[i] * v[λi] + S * q̇[joint.meta.q̇index]
        a[i] = ⁱXₚᵐ[i] * a[λi] + S * q̈[joint.meta.q̇index] + (v[i]) ×ᵐ (S * q̇[joint.meta.q̇index]) # + S∘(joint)*q̇[i]
        f[i] = I[i] * a[i] + v[i] ×ᶠ (I[i] * v[i]) - ⁱXₒᶠ[i] * fˣ[i]

    end
    for i in reverse(eachindex(joints))                
        joint = joints[i]
        
        τ[joint.meta.q̇index] = 𝒮(joint)' * f[i]
        if λ[i] != 0
            f[λ[i]] += ᵖXᵢᶠ * f[i]
        end
    end
   nothing
end

function calculate_q̈!(q̈,H,τ,C) #LinearSolve might be better!?
    q̈ .= H\(τ-C)    
    nothing
end
calculate_q̈!(sys::MultibodySystem) = calculate_q̈!(sys.q̈,sys.H,sys.τ,sys.C)

function initialize_state_vectors(joints)
    q = []
    q̇ = []
    x = []
    for joint in joints
        this_q = get_q(joint)
        this_q̇ = get_q̇(joint)
        joint.meta.qindex = SVector{length(this_q),Int16}((length(q)+1):(length(q)+length(this_q)))
        joint.meta.q̇index = SVector{length(this_q̇),Int16}((length(q̇)+1):(length(q̇ )+length(this_q̇)))
        joint.meta.xindex = SVector{length(this_q),Int16}((length(x)+1):(length(x)+length(this_q)))        
        append!(x,this_q)
        joint.meta.ẋindex = SVector{length(this_q̇),Int16}((length(x)+1):(length(x)+length(this_q̇)))        
        append!(x,this_q̇)
        append!(q,this_q)
        append!(q̇,this_q̇) 
    end
    q = MVector{length(q),Float64}(q)
    q̇ = MVector{length(q̇),Float64}(q̇)
    q̈ = MVector{length(q̇),Float64}(zeros(length(q̇)))
    x = MVector{length(x),Float64}(x)
    return (q,q̇,q̈,x)
end

function initialize_inertias(bodies)
    Iᵇ =  Vector{SpatialInertia}(undef,length(bodies)-1)       
    F1 = Cartesian(I(3),zeros(3)) #identity frame for origin at cm
    for i in 1:length(bodies)-1
        body = bodies[i]
        F2 = Cartesian(I(3),-body.cm) #frame translated from body frame to body cm
        #convert cm inertia to body frame inertia
        Iᵇ[i] = (SpatialInertia(body.I,body.m) ∈ F1 → F2)
    end
    Iᶜ = copy(Iᵇ)
    return Iᵇ,Iᶜ
end

function environments!(sys)
    gravity!(sys)
end


function gravity!(sys::MultibodySystem)
    for i in eachindex(sys.fˣ)        
        sys.fˣ[i] += sys.ᵒXᵢᶠ[i] * sys.Iᵇ[i] *  sys.ⁱXₒᵐ[i] * SVector{6,Float64}(0,0,0,0,-9.81,0)
    end
    nothing
end

function reset_f!(sys)
    for i in eachindex(sys.fˣ)
        sys.fˣ[i] *= 0
    end
    nothing
end

function update_model!(sys,x)
    reset_f!(sys)
    sys.x .= x #we probably dont need to do this other than FYI
    for joint in sys.joints
        set_state!(joint,x)
        sys.q[joint.meta.qindex] = x[joint.meta.xindex]
        sys.q̇[joint.meta.q̇index] = x[joint.meta.ẋindex]
    end
    nothing
end

function ode_func!(dx,x,p,t)
    update_model!(p.sys,x)
    model!(p.sys)
    pack_dq_in_dx!(dx,p.sys)    
    nothing
end
#=
function get_saved_values(sys::MultibodySystem)
    save_types = []
    for G in sys.G

end

saved_values = SavedValues
=#
function simulate(sys::MultibodySystem,tspan,dt=nothing)
    p = (sys = sys,)
    prob = ODEProblem(ode_func!,sys.x,tspan,p)
    sol = solve(prob)
end

#needed each timestep to fill dx with ode state derivatives [dq,q̈] (dq not q̇ since q̇ can = ω and dq is quaternion deriv)
function pack_dq_in_dx!(dx,sys)    
    for joint in sys.joints
        dx[joint.meta.xindex] = get_dq(joint)
        dx[joint.meta.ẋindex] = sys.q̈[joint.meta.q̇index]
    end    
    nothing
end