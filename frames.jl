include("rotations.jl")

using StaticArrays, LinearAlgebra
import Base: rand,show,*,∈

abstract type AbstractFrame{T<:AbstractFloat} end
abstract type AbstractFrameValue{T<:AbstractFloat} end

struct Cartesian{T <: AbstractFloat} <: AbstractFrame{T}
    Φ::RotationMatrix{T}
    r::SVector{3,T}   
    Cartesian(Φ::RotationMatrix{T},r::AbstractVector{T}) where {T<: AbstractFloat} = new{T}(Φ,SVector{3,T}(r))        
    Cartesian(Φ::AbstractMatrix{T},r::AbstractVector{T}) where {T<: AbstractFloat} = new{T}(RotationMatrix(Φ),SVector{3,T}(r))        
    Cartesian(Φ::AbstractMatrix,r::AbstractVector) = new{Float64}(RotationMatrix(Φ),SVector{3,Float64}(r))        
end
struct FrameVector{S,T<:AbstractFloat,TF<:AbstractFrame{T}} <: AbstractFrameValue{T}
    value::SVector{S,T}
    F::TF    
end
struct FrameMatrix{S1,S2,T<:AbstractFloat,TF<:AbstractFrame{T}} <: AbstractFrameValue{T}
    value::SMatrix{S1,S2,T}
    F::TF    
end

∈(val::SVector{S,T}, F::TF) where {S,T<: AbstractFloat,TF<:AbstractFrame{T}} = FrameVector{S,T,TF}(val,F)

eye(::Type{Cartesian}) = Cartesian(I(3),zeros(3))
rand(::Type{Cartesian}) = Cartesian(rand(RotationMatrix), 20*(rand(3).-1)) #randomly draw up to 10 m for testing purposes

ℛ(C::Cartesian) = C.Φ.value
𝒯(C::Cartesian) = C.r

# calculates the transform that takes quantities expressed in frame A to frame B



*(A::Cartesian,B::Cartesian) = Cartesian(RotationMatrix(ℛ(A)*ℛ(B)), 𝒯(B) + ℛ(B)'*𝒯(A))

*(A::Cartesian,v::AbstractVector) = ℛ(A)*(v-𝒯(A))

→(v::AbstractVector,F::AbstractFrame) = F.Φ * (v - F.r)
→(v::T,F::AbstractFrame) where {T<:AbstractFrameValue} = T(v.value → (v.F → F), F)
#→(v::FrameValue,F::AbstractFrame) = FrameValue(v.value → (v.F → F), F)


inv(F::Cartesian) = Cartesian(inv(F.Φ.value),F.Φ.value*(-F.r))
