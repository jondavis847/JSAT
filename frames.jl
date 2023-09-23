include("rotations.jl")

using StaticArrays
import Base: rand,show,*

abstract type Frame end
# always referenced to N frame
struct Cartesian <: Frame
    Φ::RotationMatrix # coordinate basis dyad in N frame 
    ρ::SVector{3,Float64}   # coordinate origin in N frame
end

# ensure always StaticArray
Cartesian(Φ::RotationMatrix,r) = Cartesian(Φ,SVector{3,Float64}(r))
Cartesian(Φ::AbstractMatrix,r) = Cartesian(RotationMatrix(Φ),SVector{3,Float64}(r))

rand(::Type{Cartesian}) = Cartesian(rand(RotationMatrix), SVector{3,Float64}(20*rand(3).-1)) #randomly draw up to 10 m for testing purposes

struct FrameTransform <: Frame#not sure I need to call this a thing but here we are
    Φ::Rotation
    ρ::SVector{3,Float64}
end

ℛ(C::T) where T <: Frame = C.Φ.value
𝒯(C::T) where T <: Frame= C.ρ

→(A::Frame,B::Frame) = FrameTransform(RotationMatrix(ℛ(B)'*ℛ(A)), ℛ(B)*(𝒯(A) - 𝒯(B)))

*(F::FrameTransform,v::Vector) = ℛ(F)*v - 𝒯(F)
