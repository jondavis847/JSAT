using StaticArrays, BenchmarkTools

a = @SVector zeros(3)
m = @SMatrix zeros(3, 3)

abstract type AbstractType1 end

struct Type1 <: AbstractType1
    value::Float64
    array::SVector{3,Float64}
    matrix::SMatrix{3,3,Float64}
end

t1 = Type1(1, a, m)

#=
@btime f1($t1)
112.784 ns (2 allocations: 64 bytes)

@code_warntype f1(t1)
MethodInstance for f1(::Type1)
  from f1(in::AbstractType1) @ Main C:\Users\jonda\Documents\julia\JSAT\typetest.jl:67
Arguments
  #self#::Core.Const(f1)
  in::Type1
Body::Any
1 ─ %1 = Base.getproperty(in, :matrix)::SMatrix{3, 3, Float64}
│   %2 = Base.getproperty(in, :array)::SVector{3, Float64}
│   %3 = (%1 * %2)::Any
└──      return %3

@btime f2($t1)
122.829 ns (2 allocations: 64 bytes)

@code_warntype f2(t1)
MethodInstance for f2(::Type1)
  from f2(in::T) where T<:AbstractType1 @ Main C:\Users\jonda\Documents\julia\JSAT\typetest.jl:81
Static Parameters
  T = Type1
Arguments
  #self#::Core.Const(f2)
  in::Type1
Body::Any
1 ─ %1 = Base.getproperty(in, :matrix)::SMatrix{3, 3, Float64}
│   %2 = Base.getproperty(in, :array)::SVector{3, Float64}
│   %3 = (%1 * %2)::Any
└──      return %
=#

struct Type2 <: AbstractType1
    value::AbstractFloat
    array::SVector{3,AbstractFloat}
    matrix::SMatrix{3,3,AbstractFloat}
end

t2 = Type2(1, a, m)

#@btime f1($t2)
#696.552 ns (30 allocations: 496 bytes)

#@btime f2($t2)
#718.116 ns (30 allocations: 496 bytes)

#code_warntype basically same as t1

struct Type3{T<:AbstractFloat} <: AbstractType1
    value::T
    array::SVector{3,T}
    matrix::SMatrix{3,3,T}
end

# before, 1 got converted from int64 to float, but now with parametric type, we need need to convert 1 or method isnt found
t3 = Type3(1.0, a, m)
f1(in::AbstractType1) = in.matrix * in.array
#@btime f1($t3)
#108.630 ns (2 allocations: 64 bytes)

#@btime f2($t3)
#108.611 ns (2 allocations: 64 bytes)

#code_warntype basically same as t1
# I'm a litte surprised t3 didnt do better than t1 and t2

struct Type4{T1<:AbstractFloat,T2<:SVector{3,T1},T3<:SMatrix{3,3,T1}} <: AbstractType1
    value::T1
    array::T2
    matrix::T3
end

# Type4 did convert T1 from an int to a float
t4 = Type1(1, a, m)

#@btime f1($t4)
#108.865 ns (2 allocations: 64 bytes)

#@btime f2($t4)
#124.752 ns (2 allocations: 64 bytes)

#code_warntype basically same as t1

f1(in::AbstractType1) = in.matrix * in.array
f2(in::T) where {T<:AbstractType1} = in.matrix * in.array

abstract type AbstractType2{T} end

struct Type5{T<:AbstractFloat} <: AbstractType2{T}
    value::T
    array::SVector{3,T}
    matrix::SMatrix{3,3,T}
end

# Type5 did not convert, manually convert to float
t5 = Type5(1.0, a, m)

#f3($t5)
#110.509 ns (2 allocations: 64 bytes)

#@btime f4($t5)
# 111.784 ns (2 allocations: 64 bytes)

f3(in::AbstractType2) = in.matrix * in.array
f4(in::T) where {T<:AbstractType2} = in.matrix * in.array

abstract type AbstractType3{T1,T2,T3} end

struct Type6{T1<:AbstractFloat,T2<:SVector{3,T1},T3<:SMatrix{3,3,T1}} <: AbstractType3{T1,T2,T3}
    value::T1
    array::T2
    matrix::T3
end
# interestingly, Type6 did not convert, but Type4 did
t6 = Type6(1.0, a, m)

#@btime f3($t6)
#  2.300 ns (0 allocations: 0 bytes)

# @btime f4($t6)
# 2.200 ns (0 allocations: 0 bytes)

f3(in::AbstractType3) = in.matrix * in.array
f4(in::T) where {T<:AbstractType3} = in.matrix * in.array


struct Type7{T<:AbstractFloat} <: AbstractType2{T} #similar to Type5 but with a 9 in SMatrix
    value::T
    array::SVector{3,T}
    matrix::SMatrix{3,3,T,9}
end

t7 = Type7(1.,a,m)

#@btime f3($t7)
#2.200 ns (0 allocations: 0 bytes)

struct Type8{T<:AbstractFloat} <: AbstractType1 #similar to Type3 but with a 9 in SMatrix
    value::T
    array::SVector{3,T}
    matrix::SMatrix{3,3,T,9}
end

#@btime f1($t8)
#2.300 ns (0 allocations: 0 bytes)

struct Type9 <: AbstractType1
    value::Float64
    array::SVector{3,Float64}
    matrix::SMatrix{3,3,Float64,9}
end
