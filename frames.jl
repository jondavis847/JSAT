include("rotations.jl")

using StaticArrays
import Base: rand,show,*

abstract type Frame{T} end

struct Cartesian{T<:AbstractFloat} <: Frame{T}
    Φ::RotationMatrix{T} 
    r::SVector{3,T}   
    Cartesian(Φ::AbstractMatrix,r::AbstractVector) = new{Float64}(RotationMatrix(Float64.(Φ)),SVector{3,Float64}(r))
    Cartesian(Φ::AbstractMatrix{T},r::AbstractVector{T}) where {T<:AbstractFloat} = new{T}(RotationMatrix(Φ),SVector{3,T}(r))
    Cartesian(Φ::RotationMatrix{T},r::AbstractVector{T}) where {T<:AbstractFloat}= new{T}(Φ,SVector{3,T}(r))    
end

rand(::Type{Cartesian}) = Cartesian(rand(RotationMatrix), 20*(rand(3).-1)) #randomly draw up to 10 m for testing purposes

ℛ(C::Cartesian) = C.Φ.value
𝒯(C::Cartesian) = C.r

# calculates the transform that takes quantities expressed in frame A to frame B
function →(A::Cartesian{T},B::Cartesian{T}) where T <: AbstractFloat    
    E = ℛ(B)'*ℛ(A)
    Cartesian(RotationMatrix(E),E*(𝒯(B) - 𝒯(A)))
end

#*(F::FrameTransform,v::Vector) = ℛ(F)*v - 𝒯(F)

inv(F::Cartesian) = Cartesian(inv(F.Φ.value), -F.r)
