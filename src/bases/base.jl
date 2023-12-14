mutable struct BaseFrame{G<:Vector{AbstractGravity}} <: AbstractBody
    name::Symbol    
    gravity::G
    id::Int64    
    function BaseFrame(name)         
        return new{Vector{AbstractGravity}}(name,AbstractGravity[],0)
    end
    BaseFrame(name,g::AbstractGravity) = new{Vector{AbstractGravity}}(name,[g],0)
    BaseFrame(name,g::Vector{AbstractGravity}) = new{Vector{AbstractGravity}}(name,g,0)
    function BaseFrame(name,g::Real)        
        G = GravityConstant(:base_gravity,g)
        return new{Vector{AbstractGravity}}(name,[G],0)
    end
end

calculate_fˣ!(body::BaseFrame) = return nothing #lets just get worlds out of the bodies please
calculate_gravity!(body::BaseFrame) = return nothing #lets just get worlds out of the bodies please