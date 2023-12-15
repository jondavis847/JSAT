struct BaseState
    v::SVector{6,Float64}
    a::SVector{6,Float64}
    BaseState() = new(SVector{6,Float64}(zeros(6)),SVector{6,Float64}(zeros(6)))
end

mutable struct BaseFrame{G<:Vector{AbstractGravity}} <: AbstractBody
    name::Symbol    
    gravity::G
    id::Int64 
    outer_joints::Vector{AbstractJoint} #need to make parametric - dont use abstracts as fields   
    state::BaseState
    function BaseFrame(name)         
        return new{Vector{AbstractGravity}}(name,AbstractGravity[],0,AbstractJoint[],BaseState())
    end
    BaseFrame(name,g::AbstractGravity) = new{Vector{AbstractGravity}}(name,[g],0,AbstractJoint[],BaseState())
    BaseFrame(name,g::Vector{AbstractGravity}) = new{Vector{AbstractGravity}}(name,g,0,AbstractJoint[],BaseState())
    function BaseFrame(name,g::Real)        
        G = GravityConstant(:base_gravity,g)
        return new{Vector{AbstractGravity}}(name,[G],0,AbstractJoint[],BaseState())
    end
end

calculate_fˣ!(body::BaseFrame) = return nothing #lets just get worlds out of the bodies please
calculate_gravity!(body::BaseFrame) = return nothing #lets just get worlds out of the bodies please