using StaticArrays, BenchmarkTools
import Base: *

abstract type M end

struct A <: M
    val::SMatrix
end
*(M1::M,M2::M) = M1.val*M2.val

a1 = A(@SMatrix rand(3,3))
a2 = A(@SMatrix rand(3,3))

#@btime $a1*$a2  # 1 allocation

struct B <: M
    val::SMatrix{3,3}
end
*(M1::B,M2::B) = M1.val*M2.val

b1 = B(@SMatrix rand(3,3))
b2 = B(@SMatrix rand(3,3))

#@btime $b1*$b2 # 1 allocation

struct C <: M
    val::SMatrix{3,3,Float64,9}
end
*(M1::C,M2::C) = M1.val*M2.val

c1 = C(@SMatrix rand(3,3))
c2 = C(@SMatrix rand(3,3))

@btime $c1*$c2 # 1 allocation

struct D{T<:Float64} <: M
    val::SMatrix{3,3,T}
end

*(M1::D,M2::D) = M1.val*M2.val

d1 = D(@SMatrix rand(3,3))
d2 = D(@SMatrix rand(3,3))

#@btime $d1*$d2 #1 allocation


struct E{T<:Float64} <: M
    val::SMatrix{3,3,T}
end

*(M1::E{T},M2::E{T}) where T <: Float64 = M1.val*M2.val

e1 = E(@SMatrix rand(3,3))
e2 = E(@SMatrix rand(3,3))

#@btime $e1*$e2 #1 allocation