using StaticArrays, LinearAlgebra

import Base: *,-,\,inv,rand,show,getindex,length
import LinearAlgebra: inv,norm

abstract type Rotation end
struct RotationMatrix
    value::SMatrix{3,3,Float64,9}    
    RotationMatrix(M::AbstractMatrix) = new(SMatrix{3,3,Float64,9}(M))        
end

#*(A::RotationMatrix{T},B::RotationMatrix{T}) where {T<:AbstractFloat} = RotationMatrix(A.value*B.value)
*(A::RotationMatrix,B::RotationMatrix) = RotationMatrix(A.value*B.value)

function rand(::Type{RotationMatrix})
    #https://stackoverflow.com/questions/73365934/create-random-matrix-with-orthonormal-columns
    # inspection shows that z column was always opposite sign of cross(x,y)
    z_col_fix = SMatrix{3,3,Float64,9}(1,1,1,1,1,1,-1,-1,-1);
    RotationMatrix(SMatrix{3,3,Float64,9}((qr(2*rand(3).-1)).Q .* z_col_fix))
end

#=
function show(io::IO,R::RotationMatrix)
    println(io, "RotationMatrix:")
    display(io,R.value)    
end
=#
struct Quaternion <: Rotation
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

