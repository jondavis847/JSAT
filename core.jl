using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays, DataFrames, OffsetArrays
import Base: show

includet("utils//quaternion.jl")
includet("spatial.jl")
includet("joints.jl")
includet("utils//pathutils.jl")

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

Base.@kwdef struct Connection
    Bᵢ::AbstractBody
    Fᵢ::Cartesian #Body to Joint spatial transform
    Bₒ::Body
    Fₒ::Cartesian #Joint to Body spatial transform
    G::Joint    
end
struct System
    name::Union{String,Symbol}  
    N::WorldFrame      
    B::OffsetVector{AbstractBody, Vector{AbstractBody}}
    G::Vector{Joint}
    U::Vector{Connection}        
    p::Vector{Int64} # joint predecessor body array
    s::Vector{Int64} # joint successor body array
    λ::Vector{Int64} # body parent array
    κ::Vector{Vector{Int64}} # joints between body i and base
    μ::OffsetVector{Vector{Int64}} # body children array
    γ::OffsetVector{Vector{Int64}} # all bodies from body i to tip (this is nu in Featherstone but nu just looks like v in Julia, so \gamma now)
    ᵖXᵢᵐ::Vector{SMatrix{6,6,Float64,36}}
    ᵖXᵢᶠ::Vector{SMatrix{6,6,Float64,36}}
    ⁱXₚᵐ::Vector{SMatrix{6,6,Float64,36}}
    ⁱXₚᶠ::Vector{SMatrix{6,6,Float64,36}}        
    ᵒXᵢᵐ::OffsetVector{SMatrix{6, 6, Float64, 36}, Vector{SMatrix{6, 6, Float64, 36}}}
    ᵒXᵢᶠ::OffsetVector{SMatrix{6, 6, Float64, 36}, Vector{SMatrix{6, 6, Float64, 36}}}
    ⁱXₒᵐ::OffsetVector{SMatrix{6, 6, Float64, 36}, Vector{SMatrix{6, 6, Float64, 36}}}
    ⁱXₒᶠ::OffsetVector{SMatrix{6, 6, Float64, 36}, Vector{SMatrix{6, 6, Float64, 36}}}        
    q::MVector
    q̇::MVector
    q̈::MVector
    x::MVector    
    H::MMatrix
    C::MVector  
    τ::MVector 
    fˣ::MVector
    Ī::Vector{SMatrix{6,6,Float64}}
    Īᶜ::Vector{SMatrix{6,6,Float64}}
    r::MVector
    v::OffsetVector
    a::OffsetVector
    f::MVector
end

#system constructor
function System(name, N, B̄, Ḡ, Ū)
    # preallocate vectors/matrices at system definition, mutate in place during sim

    # map the bodies and joints 
    p, s, λ, κ, μ, γ = map_path!(B̄,Ū)  
    
    # sort arrays so we can index directly by ID    
    permute!(B̄,sortperm(map(x->x.id,B̄)))
    permute!(Ḡ,sortperm(map(x->x.meta.id,Ḡ)))
    permute!(Ū,sortperm(map(x->x.G.meta.id,Ū)))


    Ī,Īᶜ = initialize_inertias(B̄)            
    
    #grab q,q̇ initial conditions
    q,q̇,q̈,x =  initialize_state_vectors(Ū)    

    nB = length(B̄)    
    # body frame spatial vectors 
    r = MVector{nB-1,SVector{7,Float64}}(fill(SVector{7,Float64}(zeros(7)),nB-1))
    v = OffsetVector(fill(SVector{6,Float64}(zeros(6)),nB),0:nB-1)
    a = OffsetVector(fill(SVector{6,Float64}(zeros(6)),nB),0:nB-1)
    f = MVector{nB-1,SVector{6,Float64}}(fill(SVector{6,Float64}(zeros(6)),nB-1))
    fˣ = MVector{nB-1,SVector{6,Float64}}(fill(SVector{6,Float64}(zeros(6)),nB-1))

    # generalized vectors
    nq̇ = length(q̇)
    τ = MVector{nq̇}(zeros(nq̇))
    C = MVector{nq̇,Float64}(zeros(nq̇))
    H = MMatrix{nq̇,nq̇,Float64}(zeros(nq̇,nq̇))
    
    # spatial transformations from body i to predecessor/parent body
    identity_X = SMatrix{6,6,Float64}(I(6))
    ᵖXᵢᵐ = fill(identity_X,nB-1)
    ᵖXᵢᶠ = fill(identity_X,nB-1)
    ⁱXₚᵐ = fill(identity_X,nB-1)
    ⁱXₚᶠ = fill(identity_X,nB-1)

    # spatial transformations from body i to worldframe
    # superscript little o here since can't start with 0 to represent 0th body   
    # could use n but it looks wierd since superscript is capital and subscript is lower case    
    ᵒXᵢᵐ = OffsetVector(fill(identity_X,nB),0:nB-1)
    ᵒXᵢᶠ = OffsetVector(fill(identity_X,nB),0:nB-1)
    ⁱXₒᵐ = OffsetVector(fill(identity_X,nB),0:nB-1)
    ⁱXₒᶠ = OffsetVector(fill(identity_X,nB),0:nB-1)
    
    System(name,N,B̄,Ḡ,Ū,p,s,λ,κ,μ,γ,ᵖXᵢᵐ,ᵖXᵢᶠ,ⁱXₚᵐ,ⁱXₚᶠ,ᵒXᵢᵐ,ᵒXᵢᶠ,ⁱXₒᵐ,ⁱXₒᶠ,q,q̇,q̈,x,H,C,τ,fˣ,Ī,Īᶜ,r,v,a,f)
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

calculate_X!(sys::System) = calculate_X!(sys.ᵖXᵢᵐ,sys.ᵖXᵢᶠ,sys.ⁱXₚᵐ,sys.ⁱXₚᶠ,sys.ᵒXᵢᵐ,sys.ᵒXᵢᶠ,sys.ⁱXₒᵐ,sys.ⁱXₒᶠ,sys.λ,sys.U)
calculate_C!(sys::System) = inverse_dynamics!(sys.C,0*sys.q̈,sys.q̇,sys.fˣ,sys.Ī,sys.ⁱXₚᵐ,sys.ⁱXₒᶠ,sys.ᵖXᵢᶠ,sys.v,sys.a,sys.f,sys.λ,sys.U)
calculate_H!(sys::System) = forward_dynamics!(sys.H,sys.Īᶜ,sys.Ī,sys.ᵖXᵢᶠ,sys.ⁱXₚᵐ,sys.λ,sys.U,)    

function calculate_τ!(τ) 
    #until we figure out how to do software
    for i in eachindex(τ)
        τ[i] = 0
    end
    nothing
end
calculate_τ!(sys::System) = calculate_τ!(sys.τ)

function calculate_X!(ᵖXᵢᵐ,ᵖXᵢᶠ,ⁱXₚᵐ,ⁱXₚᶠ,ᵒXᵢᵐ,ᵒXᵢᶠ,ⁱXₒᵐ,ⁱXₒᶠ,λ,Ū)
    for i in eachindex(Ū)        
        # U is id'd by its G, and G is id'd by its outer body or successor (s)
        U = Ū[i] #maybe try Ref?
        
        Bi_to_Bo = U.Fᵢ→Cartesian(U.G)→inv(U.Fₒ)
        Bo_to_Bi = U.Fₒ→inv(Cartesian(U.G))→inv(U.Fᵢ)

        ᵖXᵢᵐ[i] = ℳ(Bi_to_Bo).value 
        ᵖXᵢᶠ[i] = ℱ(Bi_to_Bo).value
        ⁱXₚᵐ[i] = ℳ(Bo_to_Bi).value
        ⁱXₚᶠ[i] = ℱ(Bo_to_Bi).value
        
        ᵒXᵢᵐ[i] = ᵒXᵢᵐ[λ[i]] * ᵖXᵢᵐ[i]
        ᵒXᵢᶠ[i] = ᵒXᵢᶠ[λ[i]] * ᵖXᵢᶠ[i]
        ⁱXₒᵐ[i] = ᵖXᵢᵐ[i] * ⁱXₒᵐ[λ[i]] 
        ⁱXₒᶠ[i] = ᵖXᵢᶠ[i] * ⁱXₒᶠ[λ[i]] 
    end
    nothing
end

function forward_dynamics!(H,Īᶜ,Ī,ᵖXᵢᶠ,ⁱXₚᵐ,λ,Ū)    
    #Featherstone 6.2
    for i in eachindex(Īᶜ)
        Īᶜ[i] = Ī[i]
    end
    for i in reverse(eachindex(Ū))
        Gi = Ū[i].G
        if λ[i] != 0
            Īᶜ[λ[i]] += ᵖXᵢᶠ * Īᶜ *  ⁱXₚᵐ
        end
        F = Īᶜ[i] * 𝒮(Gi)
        H[Gi.meta.q̇index,Gi.meta.q̇index] = 𝒮(Gi)'*F
        j = i
        while λ[j] != 0
            F = ᵖXᵢᶠ[j] * F
            j = λ[j]
            Gj = Ū[j].G
            H[Gi.meta.q̇index,Gj.meta.q̇index] = F' * 𝒮(Gj)
            H[Gj.meta.q̇index,Gi.meta.q̇index] = H[Gi.meta.q̇index,Gj.meta.q̇index]'
        end
    end
    nothing
end

# Featherstone chapter 5
function inverse_dynamics!(τ,q̈,q̇,fˣ,I,ⁱXₚᵐ,ⁱXₒᶠ,ᵖXᵢᶠ,v,a,f,λ,Ū)
    # note that τ will actually be C with q̈ = 0 for the C bias force calculation
    
    # we may need to add the S∘ term if translation looks really off in base 6dof joint!
    # see Featherstone example 4.5 for more details

    # v[0] already set to 0 as default, never touch
    # a[0] = already set to 0 as default, never touch 
    for i in eachindex(Ū)        
        λi = λ[i] 
        G = Ū[i].G        
        S = 𝒮(G)

        v[i] = ⁱXₚᵐ[i] * v[λi] + S * q̇[G.meta.q̇index]
        a[i] = ⁱXₚᵐ[i] * a[λi] + S * q̈[G.meta.q̇index] + (v[i]) ×ᵐ (S * q̇[G.meta.q̇index]) # + S∘(U.G)*q̇[i]
        f[i] = I[i] * a[i] + v[i] ×ᶠ (I[i] * v[i]) - ⁱXₒᶠ[i] * fˣ[i]

    end
    for i in reverse(eachindex(Ū))                
        G = Ū[i].G        
        τ[G.meta.q̇index] = 𝒮(G)' * f[i]
        if λ[i] != 0
            f[λ[i]] += ˢXᵢᶠ * f[i]
        end
    end
   nothing
end

function calculate_q̈!(q̈,H,τ,C) #LinearSolve might be better!?
    q̈ .= H\(τ-C)    
    nothing
end
calculate_q̈!(sys::System) = calculate_q̈!(sys.q̈,sys.H,sys.τ,sys.C)

function initialize_state_vectors(Ū)    
    q = []
    q̇ = []
    x = []
    for U in Ū        
        this_q = get_q(U.G)
        this_q̇ = get_q̇(U.G)
        U.G.meta.qindex = SVector{length(this_q),Int16}((length(q)+1):(length(q)+length(this_q)))
        U.G.meta.q̇index = SVector{length(this_q̇),Int16}((length(q̇)+1):(length(q̇ )+length(this_q̇)))
        U.G.meta.xindex = SVector{length(this_q),Int16}((length(x)+1):(length(x)+length(this_q)))        
        append!(x,this_q)
        U.G.meta.ẋindex = SVector{length(this_q̇),Int16}((length(x)+1):(length(x)+length(this_q̇)))        
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

function initialize_inertias(B̄)
    Ī =  Vector{SMatrix{6,6,Float64}}(undef,length(B̄)-1)       
    F1 = Cartesian(I(3),zeros(3)) #identity frame for origin at cm
    for i in 1:length(B̄)-1
        F2 = Cartesian(I(3),-B̄[i].cm) #frame translated from body frame to body cm
        #convert cm inertia to body frame inertia
        Ī[i] = (Inertia(B̄[i].I,B̄[i].m) ∈ F1 → F2).value
    end
    Īᶜ = copy(Ī)
    return Ī,Īᶜ
end

function environments!(sys)
    gravity!(sys)
end


function gravity!(sys::System)
    for i in eachindex(sys.fˣ)        
        sys.fˣ[i] += sys.Ī[i]*SVector{6,Float64}(1,0,0,0,0,0)
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
    for G in sys.G        
        set_state!(G,x)
        sys.q[G.meta.qindex] = x[G.meta.xindex]
        sys.q̇[G.meta.q̇index] = x[G.meta.ẋindex]
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
function get_saved_values(sys::System)
    save_types = []
    for G in sys.G

end

saved_values = SavedValues
=#
function simulate(sys::System,tspan,dt=nothing)
    p = (sys = sys,)
    prob = ODEProblem(ode_func!,sys.x,tspan,p)
    sol = solve(prob)
end
#= delete if okay, we do this a different way now
#just needed to initialize sys.x, the ode state variable [q;q̇]
function pack_q_in_x!(x,q,q̇,Ū)
    nq = length(q)
    for U in Ū
        x[U.G.meta.qindex] = get_q(U.G)
        x[U.G.meta.q̇index.+nq] = get_q̇(U.G)
    end
    nothing
end
=#
#needed each timestep to fill dx with ode state derivatives [dq,q̈] (dq not q̇ since q̇ can = ω and dq is quaternion deriv)
function pack_dq_in_dx!(dx,sys)    
    for G in sys.G
        dx[G.meta.xindex] = get_dq(G)
        dx[G.meta.ẋindex] = sys.q̈[G.meta.q̇index]
    end    
    nothing
end