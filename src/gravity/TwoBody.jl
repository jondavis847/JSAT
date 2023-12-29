struct TwoBody <: AbstractGravity    
    name::Symbol
    μ::Float64    
end

#source: https://en.wikipedia.org/wiki/Standard_gravitational_parameter

TwoBodyEarth(name) = TwoBody(name,3.986004418e14)
TwoBodySun(name) = TwoBody(name,1.32712440018e20)
TwoBodyMoon(name) = TwoBody(name,4.9048695e12)
TwoBodyMars(name) = TwoBody(name,4.282837e13)


function calculate_gravity(G::TwoBody, r) 
    g = -G.μ * r /(norm(r))^3
    return SVector{6,Float64}(0,0,0,g[1],g[2],g[3])
end