include("..\\blue42.jl")

N = WorldFrame()

B₁ = Body(name=:B₁,m=100,I=SMatrix{3,3,Float64}(I(3) * 100),cm=zeros(3))
ᴺGᴮ¹ = DOF6(name = :ᴺGᴮ¹)
ᴺUᴮ¹ = Connection(Bᵢ = N,Bₒ = B₁,G = ᴺGᴮ¹)

B₃ = Body(name=:B₃,m=100,I=SMatrix{3,3,Float64}(I(3) * 100),cm=zeros(3))
ᴮ¹Gᴮ³ = Revolute(name = :ᴮ¹Gᴮ³)
ᴮ¹Uᴮ³ = Connection(Bᵢ = B₁,Bₒ = B₃,G = ᴮ¹Gᴮ³)

B₅ = Body(name=:B₅,m=100,I=SMatrix{3,3,Float64}(I(3) * 100),cm=zeros(3))
ᴮ³Gᴮ⁵ = Revolute(name = :ᴮ³Gᴮ⁵)
ᴮ³Uᴮ⁵ = Connection(Bᵢ = B₃,Bₒ = B₅,G = ᴮ³Gᴮ⁵)

B₆ = Body(name=:B₆,m=100,I=SMatrix{3,3,Float64}(I(3) * 100),cm=zeros(3))
ᴮ³Gᴮ⁶ = Revolute(name = :ᴮ³Gᴮ⁶)
ᴮ³Uᴮ⁶ = Connection(Bᵢ = B₃,Bₒ = B₆,G = ᴮ³Gᴮ⁶)


B₂ = Body(name=:B₂,m=100,I=SMatrix{3,3,Float64}(I(3) * 100),cm=zeros(3))
ᴺGᴮ² = DOF6(name = :ᴺGᴮ²)
ᴺUᴮ² = Connection(Bᵢ = N,Bₒ = B₂,G = ᴺGᴮ²)

B₄ = Body(name=:B₄,m=100,I=SMatrix{3,3,Float64}(I(3) * 100),cm=zeros(3))
ᴮ²Gᴮ⁴ = Revolute(name = :ᴮ²Gᴮ⁴)
ᴮ²Uᴮ⁴ = Connection(Bᵢ = B₂,Bₒ = B₄,G = ᴮ²Gᴮ⁴)


sys = System(
    name=:oneBody6DOF,
    world = N,
    bodies=[B₁,B₂,B₃,B₄,B₅,B₆],
    joints=[ᴺGᴮ¹,ᴺGᴮ²,ᴮ¹Gᴮ³,ᴮ³Gᴮ⁵,ᴮ³Gᴮ⁶,ᴮ²Gᴮ⁴ ],
    connections= [ᴺUᴮ¹,ᴮ¹Uᴮ³,ᴮ³Uᴮ⁵,ᴮ³Uᴮ⁶,ᴺUᴮ²,ᴮ²Uᴮ⁴]
)

get_tree!(sys)
