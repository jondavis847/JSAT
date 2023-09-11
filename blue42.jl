
    using StaticArrays

    Base.@kwdef struct WorldFrame
        name::Symbol = :N
    end

    Base.@kwdef struct FrameRef
        r::SVector{3,Float64} # ᵇrʲ - location of joint origin in body frame 
        Φ::SMatrix{3,3,Float64} # ᵇΦʲ - rotation of joint frame in body frame  
        FrameRef(r::Vector{Float64},Φ::Matrix{Float64}) = new(SVector{3}(r),SMatrix{3,3}(Φ))
    end

    Base.@kwdef struct Joint        
        name::Union{String,Symbol}
        Γ::Function # joint partial function
        DOF::Int64 # number of joint speeds
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
        Bᵢ::Body
        Fᵢ::FrameRef = FrameRef(zeros(3),I(3))
        Bₒ::Body
        Fₒ::FrameRef = FrameRef(zeros(3),I(3))
        G::Joint
    end
   
    Base.@kwdef struct System
        name::Union{String,Symbol}
        bodies::Union{Nothing,Body,Vector{Body}}
        joints::Union{Nothing,Joint,Vector{Joint}}
        connections::Union{Nothing,Connection,Vector{Connection}}
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
    function Revolute(name)
        Joint(
            name =name,                        
            Γ = θ -> SA[sin(θ), 0, cos(θ)],
            DOF = 1            
        )
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
    function DOF6(name)
        Joint(
            name =name,                        
            Γ = q -> qtoa(q),
            DOF = 6            
        )
    end



