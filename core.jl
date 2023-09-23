using LinearAlgebra, DifferentialEquations, StaticArrays, Plots, UnPack, ComponentArrays, DataFrames
import Base: show

include("utils//quaternion.jl")
include("joints.jl")

Base.@kwdef struct FrameRef
    r::SVector{3,Float64} # ᵇrʲ - location of joint origin in body frame 
    Φ::SMatrix{3,3,Float64,9} # transforms body frame vectors to joint frame, Φ' transforms joint frame vector to body frame
    FrameRef(r, Φ) = new(SVector{3,Float64}(r), SMatrix{3,3,Float64}(Φ))
end

abstract type AbstractBody end

Base.@kwdef mutable struct Body <: AbstractBody
    name::Union{String,Symbol}
    #parameters
    m::Float64 # mass
    I::SMatrix{3,3,Float64} # inertia tensor
    cm::SVector{3,Float64} # center of mass    
    id::Int64 = 0 # body number for table designation (applied automatically by sys)
    Body(name, m, I, cm) = new(name, m, SMatrix{3,3,Float64}(I), SVector{3,Float64}(cm))
end

Base.@kwdef struct WorldFrame <: AbstractBody
    name::Symbol = :N
    id::Int64 = 0
end

Base.@kwdef struct Connection
    Bᵢ::Union{WorldFrame,Body}
    Fᵢ::FrameRef = FrameRef(r=zeros(3), Φ=I(3))
    Bₒ::Body
    Fₒ::FrameRef = FrameRef(r=zeros(3), Φ=I(3))
    G::Joint
end

struct PathTable
    string_table::Matrix{String}
    bool_table::Matrix{Bool}
end

struct System
    name::Union{String,Symbol}  
    N::WorldFrame      
    B::Vector{AbstractBody}
    G::Vector{Joint}
    U::Vector{Connection}
    roots::Vector{Connection}
    bodypath::PathTable
    jointpath::PathTable
    C::Matrix{SMatrix{3,3,Float64,9}}
    Γ::Vector #TODO: figure out how to do SVectors?
    Ω::Matrix{Float64} #TODO: figure out how to do SVectors?
    ρ::Matrix{SVector{3,Float64}}
    β::Vector{SVector{3,Float64}}
    #V::Matrix{Float64}
end

includet("utils//pathutils.jl")

#system constructor
function System(; name, N, B, G, U)
    #preallocate vectors/matrices at system definition, mutate in place during sim

    # map the bodies and joints 
    map_path!([N], U, 0, 0)
    roots = U[isa.(getfield.(U,:Bᵢ),WorldFrame)] 

    # get the body path
    B̄ = body_path(B, U)
    Ḡ = joint_path(B,U)

    # 2) find all the rotations of the system    
    Cs = fill(SMatrix{3,3,Float64}(zeros(3, 3)), (length(B), length(B)))
    calculate_Cs!(Cs, B, U, B̄)

    # 3) calculate joint partials Vector    
    DOFs = map(x->x.G.DOF, U) #get dofs for each joint in the systems
    Γs = map(x-> Matrix{Float64}(undef,3,x), DOFs) # each Γ is 3 x nDofs in the joint    
    calculate_Γs!(Γs,U)

    # 4) calculate Ω
    m = 3 * length(Γs) #guaranteed to be 3 elements per Γ
    sc = size.(Γs, 2)
    n = sum(sc) #num cols = sum of DOF of each joint
    Ω = zeros(m, n) #TODO: make MMatrix?    
    
    calculate_Ω!(Ω, Cs, Γs, B̄)

    # 5) calculate ρ (ρᵢⱼ = position vector from body i to joint j)
    ρs = fill(SVector{3,Float64}(zeros(3)), (length(B), length(B))) #TODO: make MMatrix?
    calculate_ρs!(ρs,B,G,U,Ḡ)    

    # 6) calculate β (βᵢ = position vector from body i to body 1)
   βs = fill(SVector{3,Float64}(zeros(3)), length(B)) #TODO: make MMatrix?
   calculate_βs!(βs, B, U)

    System(name, N, B, G, U, roots, B̄, Ḡ, Cs, Γs, Ω, ρs, βs)
end

function calculate_sys!(sys)
    #TODO: make these functions singular rather than plural as a matter of preference
    calculate_Cs!(sys)
    calculate_Γs!(sys)
    calculate_Ω!(sys)
    calculate_ρs!(sys)
    #calculate_βs!(sys)
    #calculate_V!(sys)
end

function calculate_Cs!(Cs, B, U, bodypath=body_path(B, U))
    for i in eachindex(B)
        for j in eachindex(B)
            if bodypath.bool_table[i, j]
                if i == j
                    Cs[i, j] = SMatrix{3,3,Float64}(1, 0, 0, 0, 1, 0, 0, 0, 1)
                else
                    Bₒ = B[map(x -> x.id == i, B)][1]
                    Bᵢ = B[map(x -> x.id == j, B)][1]
                    Cs[i, j] = calculate_C(Bₒ, Bᵢ, U)
                end
            end
        end
    end
    return nothing
end
calculate_Cs!(sys::System) = calculate_Cs!(sys.C,sys.B,sys.U,sys.bodypath)

#find the C matrix from one body to another in its path
function calculate_C(Bₒ, Bᵢ, Us)
    #find the connection where input arg Bₒ is the ... Bₒ
    U = find_U_by_Bₒ(Bₒ, Us)
    # get C for this layer
    C = U.Fᵢ.Φ * U.Fₒ.Φ * Φ(U.G)
    # dig deeper if needed
    if U.Bᵢ != Bᵢ
        C *= calculate_C(U.Bᵢ, Bᵢ, Us)
    end
    return C
end


function calculate_Γs!(Γs,Us)
    # I believe map_path is set up to handle the ordering here, may need to test to ensure
    for i in eachindex(Us) # length(Us) == length(Γs)
        Γs[i] = Γ(Us[i].G)
    end
    nothing
end
calculate_Γs!(sys::System) = calculate_Γs!(sys.Γs,sys.Us)

function calculate_Ω!(Ω, Cs, Γs, bodypath=trues(size(Cs))) #default to calc all  
    #TODO: can Ω be a matrix of matrices/vectors rather than a big matrix?  
    sc = size.(Γs, 2)
    for i in eachindex(Γs) # C is guaranteed to be length(B),length(B) and Γ is guaranteed to be length(B)        
        rows = (3*i:3*i+2) .- 2
        for j in eachindex(Γs)
            #only do these calcs if theres a connect between bodies, otherwise leave as zeros
            #TODO: if we make joints reconfigurable midsim, do we need to ensure this element is zeros?
            if bodypath.bool_table[i, j]
                ncols = sc[j]
                icol = sum(sc[1:j-1]) + 1
                cols = icol:icol+ncols-1

                Ω[rows, cols] = Cs[i, j] * Γs[j] #TODO should this be @views?
            end
        end
    end
    nothing
end
calculate_Ω!(sys::System) = calculate_Ω!(sys.Ω, sys.C, sys.Γ, sys.bodypath)

#=
function calculate_ρs!(ρs,roots,Us,Ḡ)    
    for U in roots
        ρ = -U.G.

    end

    for i in eachindex(Bs) #number of rows of ρs is num bodies
        for j in eachindex(Gs) #number of cols of ρs is num joints
            # if theres a body-joint connection in the jointpath, calc rho for this body-joint
            if Ḡ.bool_table[i]
                ρs[i,j] = calculate_ρ(i,j,Bs,Gs,Us)
            end
        end
    end
    nothing
end
calculate_ρs!(sys::System) = calculate_ρs!(sys.ρ,sys.B,sys.G,sys.U,sys.jointpath)
=#

function calculate_ρ(id_B, id_G, Bs, Gs, Us)
    B = find_B_by_id(id_B, Bs)
    G = find_G_by_id(id_G, Gs)
    U = find_U_by_Bₒ(B, Us)

    ρ = U.Fₒ.r - B.cm
    if U.G == G
        return ρ
    else
        return calculate_ρ(U.Bᵢ.id, id_G, Bs, Gs, Us) + ρ
    end
end

#=
function calculate_βs!(βs, Bs, Us)
    for B in Bs        
        if B.id != 1 # keep zeros for B₁ to B₁
            U = find_U_by_Bₒ(B,Us)                              
            # get vector from cm to joint in outer body frame
            # rotate from outer body frame to joint frame
            # rotates from joint frame to inner body frame
            #add vector from inner body to joint in inner body frame
            βs[B.id] = U.Fᵢ.Φ' * U.Fₒ.Φ * (U.Fₒ.r - B.cm) - U.Fᵢ.r 
            # iterate on connections until we hit root body
            U = find_U_by_Bₒ(U.Bᵢ,Us)            
            while !isa(U.Bᵢ,WorldFrame)                                
                # dont account for cm until the last step, just go from joint to body frame center                
                βs[B.id] = U.Fᵢ.Φ' * U.Fₒ.Φ * (U.Fₒ.r-βs[B.id]) - U.Fᵢ.r
                U = find_U_by_Bₒ(U.Bᵢ,Us)
            end
            # reached worldframe, just add cm
            βs[B.id] += U.Bₒ.cm
        end
    end
    nothing
end
calculate_βs!(sys::System) = calculate_βs!(sys.β, sys.B, sys.U)
=#
