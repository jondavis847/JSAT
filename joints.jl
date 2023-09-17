include("utils//quaternion.jl")
using StaticArrays

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
    id::Int64# joint number for table designation (applied automatically by sys)
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

Γ(G::DOF6) = qtoa(G.q)
Φ(G::DOF6) = qtoa(G.q)

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

Γ(G::Revolute) = SA[cos(G.θ), sin(G.θ), 0.0]
Φ(G::Revolute) = SA[cos(G.θ) -sin(G.θ) 0.0; sin(G.θ) cos(G.θ) 0.0; 0.0 0.0 1.0]
