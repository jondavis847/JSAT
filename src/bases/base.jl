mutable struct WorldFrame{G<:Vector{AbstractGravity}} <: AbstractBody
    name::Symbol    
    gravity::G
    id::Int64    
    function WorldFrame(name)         
        return new{Vector{AbstractGravity}}(name,AbstractGravity[],0)
    end
    WorldFrame(name,g::AbstractGravity) = new{Vector{AbstractGravity}}(name,[g],0)
    WorldFrame(name,g::Vector{AbstractGravity}) = new{Vector{AbstractGravity}}(name,g,0)
    function WorldFrame(name,g::Real)        
        G = GravityConstant(:base_gravity,g)
        return new{Vector{AbstractGravity}}(name,[G],0)
    end
end

calculate_fˣ!(body::WorldFrame) = return nothing #lets just get worlds out of the bodies please
calculate_gravity!(body::WorldFrame) = return nothing #lets just get worlds out of the bodies please