using StaticArrays, LinearAlgebra

import Base: *,-,/,inv,rand,show,display,getindex,length
import LinearAlgebra: inv,norm

abstract type AbstractRotation end
struct RotationMatrix{T<:AbstractFloat} <: AbstractRotation
    value::SMatrix{3,3,T,9}    
    RotationMatrix(M::AbstractMatrix) = new{Float64}(SMatrix{3,3,Float64,9}(M))        
    RotationMatrix(M::AbstractMatrix{T}) where T<: AbstractFloat = new{T}(SMatrix{3,3,T,9}(M))            
end

function Base.display(R::RotationMatrix)
    println("RotationMatrix:")
    println("$(R[1,1])   $(R[1,2])   $(R[1,3])")
    println("$(R[2,1])   $(R[2,2])   $(R[2,3])")
    println("$(R[3,1])   $(R[3,2])   $(R[3,3])")
end

Base.getindex(R::RotationMatrix, i) = R.value[i]
Base.getindex(R::RotationMatrix, i, j) = R.value[i,j]

*(A::RotationMatrix{T},B::RotationMatrix{T}) where {T<:AbstractFloat} = RotationMatrix{T}(A.value*B.value)
*(A::RotationMatrix{T},v::AbstractVector{T}) where {T<:AbstractFloat} = A.value*v

rand(::Type{RotationMatrix}) = rand(RotationMatrix{Float64})
function rand(::Type{RotationMatrix{T}}) where T<: AbstractFloat
    #https://stackoverflow.com/questions/73365934/create-random-matrix-with-orthonormal-columns
    # inspection shows that z column was always opposite sign of cross(x,y)
    z_col_fix = SMatrix{3,3,T,9}(1,1,1,1,1,1,-1,-1,-1);
    RotationMatrix(SMatrix{3,3,T,9}((qr(2*rand(3).-1)).Q .* z_col_fix))
end


struct Quaternion
    value::SVector{4,Float64}    
    Quaternion(v::AbstractVector) = new(SVector{4,Float64}(normalize(v)))
end
function Base.show(q::Quaternion)
    println("Quaternion:")
    println("$(q[1])")
    println("$(q[2])")
    println("$(q[3])")
    println("$(q[4])")
end
Base.display(q::Quaternion) = show(q)
Base.getindex(q::Quaternion, i) = q.value[i]
rand(::Type{Quaternion}) = Quaternion(2*rand(4).-1)
inv(q::Quaternion) = Quaternion((q.value .* SVector{4,Float64}(-1, -1, -1, 1)))

function *(q1::Quaternion, q2::Quaternion)  
    # Markley, F. Landis, and John L. Crassidis. "Fundamentals of Spacecraft 
    # Attitude Determination and Control." (2014): 978-1.  page 37
    Quaternion(SA[
        q2[4]*q1[1]+q2[3]*q1[2]-q2[2]*q1[3]+q2[1]*q1[4],
        -q2[3]*q1[1]+q2[4]*q1[2]+q2[1]*q1[3]+q2[2]*q1[4],
        q2[2]*q1[1]-q2[1]*q1[2]+q2[4]*q1[3]+q2[3]*q1[4],
        -q2[1]*q1[1]-q2[2]*q1[2]-q2[3]*q1[3]+q2[4]*q1[4]
    ])
end 

function /(q1::Quaternion, q2::Quaternion)
    q2*inv(q1)
end

function convert(::Type{RotationMatrix},q::Quaternion)
    I3 = SMatrix{3,3,Float64}(1, 0, 0, 0, 1, 0, 0, 0, 1)    
    q123 = q.value[SVector{3,Int64}(1, 2, 3)]
    return RotationMatrix(I3 * (q[4]^2 - q123' * q123) + 2 * q123 * q123' - 2 * scross(q123 * q[4]))
end
RotationMatrix(q::Quaternion) = convert(RotationMatrix,q)
scross(x) = SMatrix{3,3,Float64}(0, x[3], -x[2], -x[3], 0, x[1], x[2], -x[1], 0)