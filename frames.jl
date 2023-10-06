include("rotations.jl")

using StaticArrays
import Base: rand,show,*

abstract type Frame end

struct Cartesian <: Frame
    Φ::RotationMatrix
    r::SVector{3,Float64}   
    Cartesian(Φ::RotationMatrix,r::AbstractVector) = new(Φ,SVector{3,Float64}(r))        
    Cartesian(Φ::AbstractMatrix,r::AbstractVector) = new(RotationMatrix(Φ),SVector{3,Float64}(r))        
end

rand(::Type{Cartesian}) = Cartesian(rand(RotationMatrix), 20*(rand(3).-1)) #randomly draw up to 10 m for testing purposes

ℛ(C::Cartesian) = C.Φ.value
𝒯(C::Cartesian) = C.r

# calculates the transform that takes quantities expressed in frame A to frame B
function →(A::Cartesian,B::Cartesian)
    E = ℛ(B)'*ℛ(A)
    Cartesian(RotationMatrix(E),E*(𝒯(B) - 𝒯(A)))
end

#*(F::FrameTransform,v::Vector) = ℛ(F)*v - 𝒯(F)

inv(F::Cartesian) = Cartesian(inv(F.Φ.value), -F.r)
