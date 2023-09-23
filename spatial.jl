include("frames.jl")

using StaticArrays, LinearAlgebra, LabelledArrays
import Base: +, -, *
import LinearAlgebra: ×

abstract type SpatialVector end
abstract type MotionVector <: SpatialVector end
abstract type ForceVector <: SpatialVector end

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

# functions to grab either the rotation or translation part of a SpatialVector
# ℛ = \scrR + tab, 𝒯 = \scrT + tab
ℛ(v::SpatialVector) = getindex(v.value,SVector{3}(1,2,3))
𝒯(v::SpatialVector) = getindex(v.value,SVector{3}(4,5,6))

*(F::FrameTransform,v::T) where T <: MotionVector = [ℛ(F)*ℛ(v) ; - ×(𝒯(F))*𝒯(v) + ℛ(F)*𝒯(v)] #Featherstone 2.24
*(F::FrameTransform,v::T) where T <: ForceVector = [ℛ(F)*ℛ(v) - ×(𝒯(F))*𝒯(v) ;  ℛ(F)*𝒯(v)] #Featherstone 2.25

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
    
×(v::SVector{3,Float64}) = SMatrix{3,3,Float64}(0,v[3],-v[2],-v[3],0,v[1],v[2],-v[1],0) #Featherstone 2.23
×(v::Vector{Float64}) = ×(SVector{3,Float64}(v))

×(v::MotionVector) = [(×(ℛ(v))) (SMatrix{3,3}(zeros(3,3))); (×(𝒯(v))) (×(ℛ(v)))] #Featherstone 2.31
×(v::ForceVector) = [(×(ℛ(v))) (×(𝒯(v))) ; (SMatrix{3,3}(zeros(3,3))) (×(ℛ(v)))] #Featherstone 2.32
×(a::MotionVector,b::MotionVector) = [ℛ(a)×ℛ(b); ℛ(a)×𝒯(b) + 𝒯(a)×ℛ(b)] #Featherstone 2.33
×(a::ForceVector,b::ForceVector) = [ℛ(a)×ℛ(b); ℛ(a)×𝒯(b) + 𝒯(a)×ℛ(b)] #Featherstone 2.34

struct Inertia
    value::SMatrix{6,6,Float64}    
    F::Frame
end
ℛ(v::Inertia) = v.value[SVector{3}(1,2,3),SVector{3}(1,2,3)]
𝒯(v::Inertia) = v.value[SVector{3}(4,5,6),SVector{3}(4,5,6)]

Inertia(J::Union{Matrix,Diagonal}, m::Number, F::Frame) = Inertia(
    [[J;;zeros(3,3)];[zeros(3,3);;m*I(3)]], F)

function →(J::Inertia,F::Frame)
    E = J.F→F
    cx = ×(-𝒯(E)) #opposite since FrameTransform gives vector from old point to new point, and c in Featherstone is new point to old point
    m = J.value[end] #just need scalar m for this
    #Featherstone 2.63
    [(ℛ(J) .+  m .* cx * cx')   (m .*cx) 
     (m .*cx')       (m.*I(3))]
end

function inv(J::Inertia)    
    [inv(J.value)    zeros(3,3); zeros(3,3) I(3)./J.value(end)]
end

