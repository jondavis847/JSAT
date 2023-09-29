include("frames.jl")

using StaticArrays, LinearAlgebra, LabelledArrays
import Base: +, -, *
import LinearAlgebra: ×

abstract type SpatialVector end
abstract type MotionVector <: SpatialVector end
abstract type ForceVector <: SpatialVector end

const i1to3 = SA[1,2,3]
const i4to6 = SA[4,5,6]
struct Position <: MotionVector
    value::SVector{6,Float64}
    F::Frame
end
struct Velocity <: MotionVector
    value::SVector{6,Float64}
    F::Frame
end
struct Acceleration <: MotionVector
    value::SVector{6,Float64}
    F::Frame
end
struct Energy <: ForceVector
    value::SVector{6,Float64}
    F::Frame
end
struct Momentum <: ForceVector
    value::SVector{6,Float64}
    F::Frame
end
struct Force <: ForceVector
    value::SVector{6,Float64}
    F::Frame
end

struct SpatialTransform
    value::SMatrix{6,6,Float64}
end

function →(A::SpatialTransform,B::SpatialTransform)
    SMatrix{6,6,Float64}
end

# functions to grab either the rotation or translation part of a SpatialVector
# ℛ = \scrR + tab, 𝒯 = \scrT + tab
ℛ(v::SpatialVector) = getindex(v.value,SVector{3}(1,2,3))
𝒯(v::SpatialVector) = getindex(v.value,SVector{3}(4,5,6))

# calculates the spatial equivalents of frame transformations

function ℱ(E::SMatrix{3,3,Float64}, r::SVector{3,Float64})
    out = MMatrix{6,6,Float64}(undef)    
    #Featherstone 2.25, Jain 1.33
    out[i1to3,i1to3] = E    
    mul!(out[i1to3,i4to6],-E,×(r))
    out[i1to3,i4to6] = @SMatrix zeros(3,3)    
    out[i4to6,i4to6] = E
    SMatrix{6,6,Float64(out)}
end
ℱ(F::Frame) = ℱ(ℛ(F),𝒯(F))

function ℳ(E::SMatrix{3,3,Float64}, r::SVector{3,Float64})
    out = MMatrix{6,6,Float64}(undef)    
    #Featherstone 2.24, Jain 1.30
    out[i1to3,i1to3] = E    
    out[i1to3,i4to6] = @SMatrix zeros(3,3)    
    mul!(out[i4to6,i1to3], -E, ×(r))    
    out[i4to6,i4to6] = E
    SMatrix{6,6,Float64}(out)
end

ℳ(F::Frame) = ℳ(ℛ(F),𝒯(F))

function ℳ⁻¹(E::SMatrix{3,3,Float64}, r::SVector{3,Float64})
    out = MMatrix{6,6,Float64}(undef)    
    #Featherstone 2.26, Jain 1.32
    out[i1to3,i1to3] = E'
    out[i1to3,i4to6] = @SMatrix zeros(3,3)
    mul!(out[i4to6,i1to3], ×(r),E')    
    out[i4to6,i4to6] = E'
    SMatrix{6,6,Float64}(out)
end

ℳ⁻¹(F::Frame) = ℳ⁻¹(ℛ(F),𝒯(F))

function ℱ⁻¹(E::SMatrix{3,3,Float64}, r::SVector{3,Float64})
    out = MMatrix{6,6,Float64}(undef)    
    #Featherstone 2.26, Jain 1.32
    out[i1to3,i1to3] = E'
    mul!(out[i1to3,i4to6], ×(r),E')        
    out[i4to6,i1to3] = @SMatrix zeros(3,3)
    out[i4to6,i4to6] = E'
    SMatrix{6,6,Float64}(out)
end

ℱ⁻¹(F::Frame) = ℱ⁻¹(ℛ(F),𝒯(F))

*(F::FrameTransform,v::T) where T <: MotionVector = ℳ(F) * v
*(F::FrameTransform,v::T) where T <: ForceVector = ℱ(F) * v
*(F::SpatialTransform,v::T) where T <: SpatialVector = F.value * v.value

→(v::T,F::Frame) where T <: SpatialVector = (v.F === F) ? v : T( (v.F→F)*v, F)

function +(a::T,b::T) where T <: SpatialVector 
    if a.C !== b.C
        throw("Frames across + did not match. Convert both SpatialVectors to the frame you want in the output first by doing v→F (\to+tab)")
    end
    T(a.V + b.V, a.C)
end

function -(a::T,b::T) where T <: SpatialVector 
    if a.C !== b.C
        throw("Frames across + did not match. Convert both SpatialVectors to the frame you want in the output first by doing v→F (\to+tab)")
    end
    T(a.V - b.V, a.C)
end

norm(v::SpatialVector) = SVector{2}(norm(ℛ(v)), norm(𝒯(v)))
    
×(v::SVector{3,Float64}) = SMatrix{3,3,Float64}(0,v[3],-v[2],-v[3],0,v[1],v[2],-v[1],0) #Featherstone 2.23, Jain 1.9
×(v::Vector{Float64}) = ×(SVector{3,Float64}(v))

function ×(v::MotionVector) 
    #preallocate
    out=MMatrix{6,6,Float64}(undef)
    #Featherstone 2.31, Jain 1.23
    out[i1to3,i1to3] = ×(ℛ(v))
    out[i1to3,i4to6] = @SMatrix zeros(3,3)
    out[i4to6,i1to3] = ×(𝒯(v))
    out[i4to6,i4to6] = ×(ℛ(v))

    SMatrix{6,6,Float64}(out)    
end
function ×(v::ForceVector) 
    out=MMatrix{6,6,Float64}(undef)
    #Featherstone 2.32
    out[i1to3,i1to3] = ×(ℛ(v))
    out[i1to3,i4to6] = ×(𝒯(v))
    out[i4to6,i1to3] = @SMatrix zeros(3,3)
    out[i4to6,i4to6] = ×(ℛ(v))

    SMatrix{6,6,Float64}(out)    
end
function ×(a::MotionVector,b::MotionVector)    
    out = MVector{6,Float64}(undef)
    #Featherstone 2.33
    out[i1to3] = ℛ(a)×ℛ(b)
    out[i4to6] = ℛ(a)×𝒯(b) .+ 𝒯(a)×ℛ(b)
    SVector{6,Float64}(out)
end
function ×(a::ForceVector,b::ForceVector)    
    out = MVector{6,Float64}(undef)
    #Featherstone 2.34
    out[i1to3] = ℛ(a)×ℛ(b) .+ 𝒯(a)×𝒯(b)
    out[i4to6] = ℛ(a)×𝒯(b)
    SVector{6,Float64}(out)
end

struct Inertia
    value::SMatrix{6,6,Float64}    
    F::Frame
end
Inertia(J::Union{Matrix,Diagonal}, m::Number, F::Frame) = Inertia([[J;;zeros(3,3)];[zeros(3,3);;m*I(3)]], F)

ℛ(v::Inertia) = v.value[i1to3,i1to3]
𝒯(v::Inertia) = v.value[i4to6,i4to6]

function →(J::Inertia,F::Frame)
    out = MMatrix{6,6,Float64}(undef)
    E = J.F→F
    cx = ×(-𝒯(E)) #opposite since FrameTransform gives vector from old point to new point, and c in Featherstone is new point to old point
    m = J.value[end] #just need scalar m for this
    #Featherstone 2.63 ( - instead of ' since its the same result for c)
    out[i1to3,i1to3] = ℛ(J) .+  m .* cx * cx'
    out[i1to3,i4to6] = (m .*cx) 
    out[i4to6,i1to3] = (m .*cx)' 
    out[i4to6,i4to6] = (m.*I(3))
    Inertia(SMatrix{6,6,Float64}(out),F)
end

#= this isnt right, use featherstone 2.74, not 2.73
function inv(J::Inertia)    
    [inv(J.value);;    zeros(3,3) ; zeros(3,3);; I(3)./J.value[end]]
end
=#

ℳ(G::Joint) = ℳ(Φ(G),ρ(G))
ℱ(G::Joint) = ℱ(Φ(G),ρ(G))

