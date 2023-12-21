#=
abstract types are defined here so that they can be used in structs prior to definitions of other structs
for example, doing 

struct Type1
    field::Type2
end

struct Type2
    field::Type1
end

does not work because Type2 does not exist when creating Type1. However,

abstract type AbstractType1 end
abstract type AbstractType2 end

struct Type1 <: AbstractType1
    field::AbstractType2
end

struct Type2 <: AbstractType2
    field::AbstractType1
end

does work
=#
abstract type AbstractBody end
abstract type AbstractJoint end
abstract type AbstractJointState end
abstract type AbstractActuator end
abstract type AbstractSoftware end
abstract type AbstractSensor end
abstract type AbstractEnvironment end
abstract type AbstractGravity end
