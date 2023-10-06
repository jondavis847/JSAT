includet("frames.jl")

using StaticArrays, LinearAlgebra
import Base: +, -, *, ∈
import LinearAlgebra: ×

const i1to3 = SA[1,2,3]
const i4to6 = SA[4,5,6]

abstract type SpatialValue  end
abstract type SpatialVector <: SpatialValue end

struct FrameValue
    value::SpatialValue
    F::Cartesian
end

struct MotionVector <: SpatialVector
    value::SVector{6,Float64}    
    MotionVector(val::AbstractVector) = new(SVector{6,Float64}(val))    
end

struct ForceVector <: SpatialVector
    value::SVector{6,Float64}    
    ForceVector(val::AbstractVector) = new(SVector{6,Float64}(val))    
end

struct SpatialTransform
    value::SMatrix{6,6,Float64,36}
    SpatialTransform(val::AbstractMatrix) = new(SMatrix{6,6,Float64,36}(val))    
end

struct SpatialInertia <: SpatialValue
    value::SMatrix{6,6,Float64,36}            
    SpatialInertia(val::AbstractMatrix) = new(SMatrix{6,6,Float64,36}(val))
    SpatialInertia(inertia::AbstractMatrix, mass::Real) = new(SMatrix{6,6,Float64,36}([inertia zeros(3,3) ; zeros(3,3) mass*I(3)]))
end

∈(val::SpatialValue, F::Frame) = FrameValue(val,F)

# functions to grab either the rotation or translation part of a SpatialVector
# ℛ = \scrR + tab, 𝒯 = \scrT + tab
ℛ(v::SpatialVector) = v.value[i1to3]
𝒯(v::SpatialVector) = v.value[i4to6]

# calculates the spatial equivalents of frame transformations
function ℱ(E::SMatrix{3,3,T}, r::SVector{3,T}) where T<: AbstractFloat
    out = MMatrix{6,6,T}(undef)    
    #Featherstone 2.25, Jain 1.33
    out[i1to3,i1to3] = E    
    out[i1to3,i4to6] =  -E * ×(r)
    out[i4to6,i1to3] = @SMatrix zeros(T,3,3)    
    out[i4to6,i4to6] = E
    SpatialTransform(SMatrix{6,6,T}(out))
end
ℱ(F::Cartesian) = ℱ(ℛ(F),𝒯(F))

function ℳ(E::SMatrix{3,3,T}, r::SVector{3,T}) where T<: AbstractFloat
    out = MMatrix{6,6,T}(undef)    
    #Featherstone 2.24, Jain 1.30
    out[i1to3,i1to3] = E    
    out[i1to3,i4to6] = @SMatrix zeros(3,3)    
    out[i4to6,i1to3] =  -E * ×(r)
    out[i4to6,i4to6] = E
    SpatialTransform(SMatrix{6,6,T}(out))
end
ℳ(F::Cartesian) = ℳ(ℛ(F),𝒯(F))

function ℳ⁻¹(E::SMatrix{3,3,T}, r::SVector{3,T}) where T<: AbstractFloat
    out = MMatrix{6,6,T}(undef)    
    #Featherstone 2.26, Jain 1.32
    out[i1to3,i1to3] = E'
    out[i1to3,i4to6] = @SMatrix zeros(3,3)
    out[i4to6,i1to3] = ×(r) * E'    
    out[i4to6,i4to6] = E'
    SpatialTransform(SMatrix{6,6,T}(out))
end
ℳ⁻¹(F::Cartesian) = ℳ⁻¹(ℛ(F),𝒯(F))

function ℱ⁻¹(E::SMatrix{3,3,T}, r::SVector{3,T}) where T<:AbstractFloat
    out = MMatrix{6,6,T}(undef)    
    #Featherstone 2.26, Jain 1.32
    out[i1to3,i1to3] = E'
    out[i1to3,i4to6] = ×(r)*E' #may need to go to inplace multiplication here        
    out[i4to6,i1to3] = @SMatrix zeros(3,3)
    out[i4to6,i4to6] = E'
    SpatialTransform(SMatrix{6,6,T}(out))
end
ℱ⁻¹(F::Cartesian) = ℱ⁻¹(ℛ(F),𝒯(F))


*(F::Cartesian,v::MotionVector) = MotionVector(ℳ(F).value * v.value)
*(F::Cartesian,v::ForceVector)  = ForceVector(ℱ(F).value * v.value)
*(F::SpatialTransform,v::T) where T<:SpatialValue = T(F.value * v.value)
*(A::SpatialTransform,B::SpatialTransform) = SpatialTransform(A.value * B.value)
*(v::SpatialInertia,F::SpatialTransform) = SpatialInertia(F.value * v.value)

+(a::T,b::T) where {T<:SpatialVector} = T(a.value + b.value)
-(a::T,b::T) where {T<:SpatialVector} = T(a.value - b.value)

×(v::SVector{3,T}) where T <: AbstractFloat = SMatrix{3,3,T}(0,v[3],-v[2],-v[3],0,v[1],v[2],-v[1],0) #Featherstone 2.23, Jain 1.9
×(v::Vector{T}) where T <: AbstractFloat = ×(SVector{3,T}(v))

function ×(v::MotionVector) 
    out=MMatrix{6,6,T}(undef)
    #Featherstone 2.31, Jain 1.23
    out[i1to3,i1to3] = ×(ℛ(v))
    out[i1to3,i4to6] = @SMatrix zeros(3,3)
    out[i4to6,i1to3] = ×(𝒯(v))
    out[i4to6,i4to6] = ×(ℛ(v))
    SMatrix{6,6,T}(out)    
end
function ×(v::ForceVector) 
    out=MMatrix{6,6,T}(undef)
    #Featherstone 2.32
    out[i1to3,i1to3] = ×(ℛ(v))
    out[i1to3,i4to6] = ×(𝒯(v))
    out[i4to6,i1to3] = @SMatrix zeros(3,3)
    out[i4to6,i4to6] = ×(ℛ(v))
    SMatrix{6,6,T}(out)    
end
function ×(a::ForceVector,b::ForceVector) 
    out = MVector{6,T}(undef)
    #Featherstone 2.34
    out[i1to3] = ℛ(a)×ℛ(b) .+ 𝒯(a)×𝒯(b)
    out[i4to6] = ℛ(a)×𝒯(b)
    SVector{6,T}(out)
end

function ×ᶠ(a::StaticVector{6,T},b::StaticVector{6,T}) where T <: AbstractFloat   
    out = MVector{6,T}(undef)
    #Featherstone 2.34
    out[i1to3] = a[i1to3]×b[i1to3] .+ a[i4to6]×b[i4to6] 
    out[i4to6] = a[i1to3]×b[i4to6] 
    SVector{6,T}(out)
end



function ×(a::MotionVector,b::MotionVector)
    out = MVector{6,T}(undef)
    #Featherstone 2.33
    out[i1to3] = ℛ(a)×ℛ(b)
    out[i4to6] = ℛ(a)×𝒯(b) .+ 𝒯(a)×ℛ(b)
    SVector{6,T}(out)
end

function ×ᵐ(a::StaticVector{6,T},b::StaticVector{6,T}) where T <: AbstractFloat   
    out = MVector{6,T}(undef)
    #Featherstone 2.33
    out[i1to3] = a[i1to3]×b[i1to3]
    out[i4to6] = a[i1to3]×b[i4to6] .+ a[i4to6]×b[i1to3]
    SVector{6,T}(out)
end


ℛ(v::SpatialInertia) = v.value[i1to3,i1to3]
𝒯(v::SpatialInertia) = v.value[i4to6,i4to6]

#current allocates, maybe need to make a method for each ∈ FrameValue
function →(FV::FrameValue,C::Cartesian)
    X = FV.F → C
    FV.value → X
end

→(v::MotionVector,F::Cartesian) = ℳ(F) * v
→(v::ForceVector,F::Cartesian) = ℱ(F) * v
→(J::SpatialInertia,F::Cartesian) = ℱ(F) * J * ℳ⁻¹(F) #Featherstone 2.66

#= this isnt right, use featherstone 2.74, not 2.73
function inv(J::Inertia)    
    [inv(J.value);;    zeros(3,3) ; zeros(3,3);; I(3)./J.value[end]]
end
=#

*(J::SpatialInertia,v::MotionVector) = ForceVector(J.value * v.value)

*(J::SpatialInertia, v::SVector{6, Float64}) = J.value * v
*(X::SpatialTransform, v::SVector{6, Float64}) = X.value * v
*(J::SpatialInertia, v::SMatrix{6, 1, Float64, 6}) = J.value * v