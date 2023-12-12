struct GravityConstant <: AbstractGravity
    value::SVector{6,Float64}
    function GravityConstant(g)
        if length(g) == 1
            return new(SVector{6,Float64}(0,0,0,0,g,0))
        elseif length(g) == 3
            return new(SVector{6,Float64}(0,0,0,g...))
        else
            return new(SVector{6,Float64}(g))
        end 
    end
end

GravityNone() = GravityConstant(0)

calculate_gravity(g::GravityConstant) = g.value