mutable struct SimpleRateSensor
    rate::SVector{4,Float64}    
    body::Body
    SimpleRateSensor() = new()
end