function Plots.plot(t::Vector{T}, x::Vector{SVector{S,T}}) where {S,T<:AbstractFloat}
    p = plot(t, getindex.(x, 1))
    if length(x[1]) > 1
        for i in 2:length(x[1])
            plot!(t, getindex.(x, i))
        end
    end
    return p
end

function plot(sol::DataFrame, col)
    p = plot(sol[!, "t"], sol[!, col])
    return p
end