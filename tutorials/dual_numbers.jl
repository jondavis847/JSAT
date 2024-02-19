using DifferentialEquations
import Base: +,-,*,/
	
# Define the Dual number type

struct Dual{T}
    value::NTuple{2,T}    
end
Dual(real_part,dual_part) = Dual((real_part,dual_part))

# Define the Dual number utility functions

re(a::Dual{T}) where T = a.value[1]
du(a::Dual{T}) where T = a.value[2]

# Define the Dual number math

+(a::Dual{T},b::Dual{T}) where T = Dual((re(a)+re(b), du(a)+du(b)))
-(a::Dual{T},b::Dual{T}) where T = Dual((re(a)-re(b), du(a)-du(b)))

function *(a::Dual{T},b::Dual{T}) where T 
    real_part = re(a) * re(b)
    dual_part = re(a) * du(b) + du(a) * re(b)
    return Dual(real_part,dual_part)
end

function /(a::Dual{T},b::Dual{T}) where T
    real_part = re(a) / re(b)
    dual_part = (du(a) * re(a) - re(a) * du(b)) / re(b)^2
    return Dual(real_part,dual_part)
end

sin(a::Dual{T}) = Dual(sin(re(a)), du(a)*cos(re*(a)))

## Test the Dual Number math

t1 = Dual(2.0,1.0);
t2 = Dual(3.0,-1.0);

println(t1+t2)
println(t1-t2)
println(t1*t2)
println(t1/t2)

t = range(0,10,100)
y = sin.(t)
ẏ = cos.(t)

