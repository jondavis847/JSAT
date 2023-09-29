##NOTE we may need to add S∘ for joint space derivatives if translations looks really bad for the base 6dof joint!!!!


abstract type Joint end

"""
    DOF6(;name,[q,ω,r,v])

6DOF joint for rotation and translation

States:
- `q::SVector{4,Float64}` quaternion rotation of Bₒ in Bᵢ
- `ω::SVector{3,Float64}`  angular rate

Joint frame:
- x right, y up, z out the page
- identity quaternion means body x,y,z aligns with joint x,y,z        
    
"""

mutable struct DOF6 <: Joint
    name::Union{String,Symbol}
    DOF::Int64# number of joint speeds
    id::Int64# joint number for table designation (applied automatically by sys)    #TODO: put id on connection?
    q::SVector{4,Float64}
    ω::SVector{3,Float64}
    r::SVector{3,Float64}
    v::SVector{3,Float64}    
end

#constructor
function DOF6(name,
    q = SVector{4,Float64}(0, 0, 0, 1),
    ω = SVector{3,Float64}(0, 0, 0),
    r = SVector{3,Float64}(0, 0, 0),
    v = SVector{3,Float64}(0, 0, 0),
    )
    DOF6(name,6,0,q,ω,r,v)
end

#Γ(G::DOF6) = qtoa(G.q)
Φ(G::DOF6) = qtoa(G.q)
ρ(G::DOF6) = G.r
𝒮(G::DOF6) = SMatrix{6,6,Float64}(I(6))
𝒯(G::DOF6) = SMatrix{6,6,Float64}(I(6))

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

mutable struct Revolute <: Joint
    name::Union{String,Symbol}
    DOF::Int64# number of joint speeds
    id::Int64# joint number for table designation (applied automatically by sys)    
    θ::Float64
    ω::Float64    
end
function Revolute(name,θ=0.0,ω=0.0)
    Revolute(name,1,0,θ,ω)
end

#Γ(G::Revolute) = SMatrix{3,1,Float64}(cos(G.θ), sin(G.θ), 0.0)
Φ(G::Revolute) = SA[cos(G.θ) -sin(G.θ) 0.0; sin(G.θ) cos(G.θ) 0.0; 0.0 0.0 1.0]
ρ(G::Revolute) = SVector{3,Float64}(0,0,0)
𝒮(G::Revolute) = SMatrix{6,1,Float64}(0,0,1,0,0,0)
𝒯(G::Revolute) = SMatrix{6,5,Float64}([
    1 0 0 0 0
    0 1 0 0 0
    0 0 0 0 0
    0 0 1 0 0
    0 0 0 1 0
    0 0 0 0 1
])
