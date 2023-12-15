using StaticArrays, LinearAlgebra

import Base: *,-,\,inv,rand,show,getindex,length
import LinearAlgebra: inv,norm

abstract type AbstractRotation end
struct RotationMatrix{T<:AbstractFloat} <: AbstractRotation
    value::SMatrix{3,3,T,9}    
    RotationMatrix(M::AbstractMatrix) = new{Float64}(SMatrix{3,3,Float64,9}(M))        
    RotationMatrix(M::AbstractMatrix{T}) where T<: AbstractFloat = new{T}(SMatrix{3,3,T,9}(M))            
end

*(A::RotationMatrix{T},B::RotationMatrix{T}) where {T<:AbstractFloat} = RotationMatrix{T}(A.value*B.value)
*(A::RotationMatrix{T},v::AbstractVector{T}) where {T<:AbstractFloat} = A.value*v

rand(::Type{RotationMatrix}) = rand(RotationMatrix{Float64})
function rand(::Type{RotationMatrix{T}}) where T<: AbstractFloat
    #https://stackoverflow.com/questions/73365934/create-random-matrix-with-orthonormal-columns
    # inspection shows that z column was always opposite sign of cross(x,y)
    z_col_fix = SMatrix{3,3,T,9}(1,1,1,1,1,1,-1,-1,-1);
    RotationMatrix(SMatrix{3,3,T,9}((qr(2*rand(3).-1)).Q .* z_col_fix))
end

#=
struct Quaternion{T<:AbstractFloat} <: AbstractRotation{T}
    value::SVector{4,Float64}    
    Quaternion(v::AbstractVector) = new(SVector{4,Float64}(normalize(v)))
end

Base.getindex(q::Quaternion, i) = q.value[i]
rand(::Type{Quaternion}) = Quaternion(2*rand(4).-1)

inv(q::Quaternion) = Quaternion((q.value .* SVector{4,Float64}(-1, -1, -1, 1))/norm(q.value)^2)

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

function *(q1::SVector{4,Float64}, q2::SVector{4,Float64})     
    # Markley, F. Landis, and John L. Crassidis. "Fundamentals of Spacecraft 
    # Attitude Determination and Control." (2014): 978-1.  page 37   

    SVector{4,Float64}(
        q2[4]*q1[1]+q2[3]*q1[2]-q2[2]*q1[3]+q2[1]*q1[4],
        -q2[3]*q1[1]+q2[4]*q1[2]+q2[1]*q1[3]+q2[2]*q1[4],
        q2[2]*q1[1]-q2[1]*q1[2]+q2[4]*q1[3]+q2[3]*q1[4],
        -q2[1]*q1[1]-q2[2]*q1[2]-q2[3]*q1[3]+q2[4]*q1[4]
    )    
end 

function \(q1::Quaternion, q2::Quaternion)
    Quaternion(q1.value*inv(q2).value/norm(q2.value)^2)
end

=#