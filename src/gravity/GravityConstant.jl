struct GravityConstant <: AbstractGravity
    name::Symbol
    value::SVector{6,Float64}
    function GravityConstant(name,g)
        if length(g) == 1
            return new(name,SVector{6,Float64}(0,0,0,0,g,0))
        elseif length(g) == 3
            return new(name,SVector{6,Float64}(0,0,0,g...))
        else
            return new(name,SVector{6,Float64}(g))
        end 
    end
end

GravityNone() = GravityConstant(:none,0)

calculate_gravity(g::GravityConstant) = g.value