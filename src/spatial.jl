includet("frames.jl")

using StaticArrays, LinearAlgebra
import Base: +, -, *, ∈
import LinearAlgebra: ×

const i3 = SA[1, 2, 3]
const i6 = SA[4, 5, 6]

abstract type SpatialValue{T} end
abstract type SpatialVector{T} <: SpatialValue{T} end

struct MotionVector{T<:AbstractFloat} <: SpatialVector{T}
    value::SVector{6,T}
end

struct ForceVector{T<:AbstractFloat} <: SpatialVector{T}
    value::SVector{6,T}
end


struct SpatialInertia{T<:AbstractFloat} <: SpatialValue{T}
    value::SMatrix{6,6,T,36}
end

function SpatialInertia(inertia::AbstractMatrix, mass::Real)
    SpatialInertia{Float64}(SMatrix{6,6,Float64,36}([inertia zeros(3, 3); zeros(3, 3) mass*I(3)]))
end

# functions to grab either the rotation or translation part of a SpatialVector
# ℛ = \scrR + tab, 𝒯 = \scrT + tab
ℛ(v::SpatialVector) = v.value[i3]
𝒯(v::SpatialVector) = v.value[i6]


struct SpatialTransform{T<:AbstractFloat}
    value::SMatrix{6,6,T,36}
end

# calculates the spatial equivalents of frame transformations
function ℱ(E::SMatrix{3,3,T}, r::SVector{3,T}) where {T<:AbstractFloat}
    out = MMatrix{6,6,T}(undef)
    #Featherstone 2.25, Jain 1.33
    out[i3, i3] = E
    out[i3, i6] = -E * ×(r)
    out[i6, i3] = @SMatrix zeros(T, 3, 3)
    out[i6, i6] = E
    #SpatialTransform(SMatrix{6,6,T}(out))
    SMatrix{6,6,T}(out)
end
ℱ(F::Cartesian) = ℱ(ℛ(F), 𝒯(F))

function ℳ(E::SMatrix{3,3,T}, r::SVector{3,T}) where {T<:AbstractFloat}
    out = MMatrix{6,6,T}(undef)
    #Featherstone 2.24, Jain 1.30
    out[i3, i3] = E
    out[i3, i6] = @SMatrix zeros(3, 3)
    out[i6, i3] = -E * ×(r)
    out[i6, i6] = E
    #SpatialTransform(SMatrix{6,6,T}(out))
    SMatrix{6,6,T}(out)
end
ℳ(F::Cartesian) = ℳ(ℛ(F), 𝒯(F))

function ℳ⁻¹(E::SMatrix{3,3,T}, r::SVector{3,T}) where {T<:AbstractFloat}
    out = MMatrix{6,6,T}(undef)
    #Featherstone 2.26, Jain 1.32
    out[i3, i3] = E'
    out[i3, i6] = @SMatrix zeros(3, 3)
    out[i6, i3] = ×(r) * E'
    out[i6, i6] = E'
    #SpatialTransform(SMatrix{6,6,T}(out))
    SMatrix{6,6,T}(out)
end
ℳ⁻¹(F::Cartesian) = ℳ⁻¹(ℛ(F), 𝒯(F))

function ℱ⁻¹(E::SMatrix{3,3,T}, r::SVector{3,T}) where {T<:AbstractFloat}
    out = MMatrix{6,6,T}(undef)
    #Featherstone 2.26, Jain 1.32
    out[i3, i3] = E'
    out[i3, i6] = ×(r) * E' #may need to go to inplace multiplication here        
    out[i6, i3] = @SMatrix zeros(3, 3)
    out[i6, i6] = E'
    #SpatialTransform(SMatrix{6,6,T}(out))
    SMatrix{6,6,T}(out)
end
ℱ⁻¹(F::Cartesian) = ℱ⁻¹(ℛ(F), 𝒯(F))


*(F::SpatialTransform, v::T) where {T<:SpatialValue} = T(F.value * v.value)
*(A::SpatialTransform, B::SpatialTransform) = SpatialTransform(A.value * B.value)

+(a::T, b::T) where {T<:SpatialVector} = T(a.value + b.value)
-(a::T, b::T) where {T<:SpatialVector} = T(a.value - b.value)

×(v::SVector{3,T}) where {T<:AbstractFloat} = SMatrix{3,3,T}(0, v[3], -v[2], -v[3], 0, v[1], v[2], -v[1], 0) #Featherstone 2.23, Jain 1.9
×(v::Vector{T}) where {T<:AbstractFloat} = ×(SVector{3,T}(v))

function ×(v::MotionVector)
    out = MMatrix{6,6,T}(undef)
    #Featherstone 2.31, Jain 1.23
    out[i3, i3] = ×(ℛ(v))
    out[i3, i6] = @SMatrix zeros(3, 3)
    out[i6, i3] = ×(𝒯(v))
    out[i6, i6] = ×(ℛ(v))
    SMatrix{6,6,T}(out)
end
function ×(v::ForceVector)
    out = MMatrix{6,6,T}(undef)
    #Featherstone 2.32
    out[i3, i3] = ×(ℛ(v))
    out[i3, i6] = ×(𝒯(v))
    out[i6, i3] = @SMatrix zeros(3, 3)
    out[i6, i6] = ×(ℛ(v))
    SMatrix{6,6,T}(out)
end
function ×(a::ForceVector, b::ForceVector)
    out = MVector{6,T}(undef)
    #Featherstone 2.34
    out[i3] = ℛ(a) × ℛ(b) .+ 𝒯(a) × 𝒯(b)
    out[i6] = ℛ(a) × 𝒯(b)
    SVector{6,T}(out)
end

function ×ᶠ(a::StaticVector{6,T}, b::StaticVector{6,T}) where {T<:AbstractFloat}
    out = MVector{6,T}(undef)
    #Featherstone 2.34
    out[i3] = a[i3] × b[i3] .+ a[i6] × b[i6]
    out[i6] = a[i3] × b[i6]
    SVector{6,T}(out)
end

function crm(v::SVector{6,T}) where {T<:AbstractFloat}
    SA[0 -v[3] v[2] 0 0 0
        v[3] 0 -v[1] 0 0 0
        -v[2] v[1] 0 0 0 0
        0 -v[6] v[5] 0 -v[3] v[2]
        v[6] 0 -v[4] v[3] 0 -v[1]
        -v[5] v[4] 0 -v[2] v[1] 0]
end

function crf(v::SVector{6,T}) where {T<:AbstractFloat}
    -crm(v)'
end

function ×(a::MotionVector, b::MotionVector)
    out = MVector{6,T}(undef)
    #Featherstone 2.33
    out[i3] = ℛ(a) × ℛ(b)
    out[i6] = ℛ(a) × 𝒯(b) .+ 𝒯(a) × ℛ(b)
    SVector{6,T}(out)
end

function ×ᵐ(a::StaticVector{6,T}, b::StaticVector{6,T}) where {T<:AbstractFloat}
    out = MVector{6,T}(undef)
    #Featherstone 2.33
    out[i3] = a[i3] × b[i3]
    out[i6] = a[i3] × b[i6] .+ a[i6] × b[i3]
    SVector{6,T}(out)
end


ℛ(v::SpatialInertia) = v.value[i3, i3]
𝒯(v::SpatialInertia) = v.value[i6, i6]

#current allocates, maybe need to make a method for each ∈ FrameValue


→(v::MotionVector, F::Cartesian) = ℳ(F) * v
→(v::ForceVector, F::Cartesian) = ℱ(F) * v


*(J::SpatialInertia, v::MotionVector) = ForceVector(J.value * v.value)

*(J::SpatialInertia, v::SVector{6,Float64}) = J.value * v
*(X::SpatialTransform, v::SVector{6,Float64}) = X.value * v
*(J::SpatialInertia, v::SMatrix{6,1,Float64,6}) = J.value * v

→(J::SpatialInertia, F::Cartesian) = SpatialInertia(ℱ(F) * J.value * ℳ⁻¹(F))


function skew(x::SVector{3,Float64})
    SA[
        0 -x[3] x[2]
        x[3] 0 -x[1]
        -x[2] x[1] 0
    ]
end
function mcI(m::Float64, cm::SVector{3,Float64}, inertia::SMatrix{3,3,Float64})
    out = MMatrix{6,6,Float64}(undef)
    C = skew(cm)
    out[i3, i3] = inertia + m * C * C'
    out[i3, i6] = m * C
    out[i6, i3] = m * C'
    out[i6, i6] = SMatrix{3,3,Float64}(m, 0, 0, 0, m, 0, 0, 0, m) #m*I(3)
    SMatrix{6,6,Float64}(out)
end


