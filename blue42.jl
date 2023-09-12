
    include("utils//quaternion.jl")
    using StaticArrays
    I3 = SMatrix{3,3,Float64}(1,0,0,0,1,0,0,0,1)

    abstract type Joint end

    Base.@kwdef struct WorldFrame
        name::Symbol = :N
    end

    Base.@kwdef struct FrameRef
        r::SVector{3,Float64} # ᵇrʲ - location of joint origin in body frame 
        Φ::SMatrix{3,3,Float64,9} # ᵇΦʲ - rotation of joint frame in body frame  
        FrameRef(r::SVector{3,Float64},Φ::SMatrix{3,3,Float64}) = new()
        FrameRef(r::Vector{Float64},Φ::Matrix{Float64}) = new(SVector{3,Float64}(r),SMatrix{3,3,Float64,9}(Φ))
    end
  

    Base.@kwdef struct Body                
        name::Union{String,Symbol}
        #parameters
        m::Float64 # mass
        I::SMatrix{3,3,Float64} # inertia tensor
        cm::SVector{3,Float64} # center of mass    
        Body(name,m,I,cm) = new(name,m,SMatrix{3,3,Float64}(I),SVector{3,Float64}(cm))
    end

    Base.@kwdef struct Connection
        Bᵢ::Union{WorldFrame,Body}
        Fᵢ::FrameRef = FrameRef(r=SVector{3}(zeros(3)),Φ=I3)
        Bₒ::Body
        Fₒ::FrameRef = FrameRef(r=SVector{3}(zeros(3)),Φ=I3)
        G::Joint
    end
   
    Base.@kwdef struct System
        name::Union{String,Symbol}
        world::WorldFrame
        bodies::Union{Nothing,Body,Vector{Body}}
        joints::Union{Nothing,Joint,Vector{Joint}}
        connections::Union{Nothing,Connection,Vector{Connection}}
    end

    """
    DOF6 Joint
    
        6DOF rotation and translation

    States:
        q - quaternion rotation of Bₒ in Bᵢ
        ω - angular rate
    Joint frame:
        x right, y up, z out the page
        identity quaternion means body x,y,z aligns with joint x,y,z        
        
    """
    Base.@kwdef mutable struct DOF6 <: Joint        
        name::Union{String,Symbol}
        Γ::Function =  q->qtoa(q)# joint partial function
        DOF::Int64 = 6 # number of joint speeds
        q::SVector{4,Float64} = SVector{4,Float64}(0,0,0,1)
        ω::SVector{3,Float64} = SVector{3,Float64}(0,0,0)
        r::SVector{3,Float64} = SVector{3,Float64}(0,0,0)
        v::SVector{3,Float64} = SVector{3,Float64}(0,0,0)
        DOF6(name,Γ,DOF,q,ω,r,v) = new(name,Γ,DOF,SVector{4,Float64}(q),SVector{3,Float64}(ω),SVector{3,Float64}(r),SVector{3,Float64}(v))
    end    


    """
    Revolute Joint
    
        1DOF rotation in the x-z plane about y (to be consistent with Three.js geometry)

    States:
        θ - rotation angle
        ω - angular rate
    Joint frame:
        - right hand rule
        - x to the right, y up, z out of the page
        - θ referenced from +x        

    """
    Base.@kwdef mutable struct Revolute <: Joint        
        const name::Union{String,Symbol}
        const Γ::Function = θ -> SA[sin(θ), 0, cos(θ)] # joint partial function
        const DOF::Int64 = 1 # number of joint speeds
        θ::Float64 = 0
        ω::Float64 = 0
        Revolute(name,Γ,DOF,q,ω,r,v) = new(name,Γ,DOF,SVector{3,Float64}(θ),SVector{3,Float64}(ω))
    end    