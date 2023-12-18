struct ZeroReference <: AbstractSoftware    
    value::Float64
    ZeroReference() = new(0)
end

struct ZeroReference3 <: AbstractSoftware    
    value::SVector{3,Float64}
    ZeroReference3() = new(SVector{3,Float64}(zeros(3)))
end

get_callback(zr::ZeroReference,i) = nothing
get_callback(zr3::ZeroReference3,i) = nothing