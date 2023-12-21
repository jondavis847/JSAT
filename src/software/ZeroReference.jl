struct ZeroReference <: AbstractSoftware    
    value::Float64
    connection::SoftwareConnection
    ZeroReference() = new(0)
end

struct ZeroReference3 <: AbstractSoftware    
    value::SVector{3,Float64}
    connection::SoftwareConnection
    ZeroReference3() = new(SVector{3,Float64}(zeros(3)))
end

get_callback(zr::ZeroReference,i) = nothing
function get_callback(zr3::ZeroReference3,i) 
   function affect!(integrator)
        software = integrator.p.sys.software[i]
   end
end