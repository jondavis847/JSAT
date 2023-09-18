include("..\\main.jl")

N = WorldFrame()

# main body
B = Body(name=:B, m=1000, I=1000*I(3), cm=[0,0.1,0])
ᴺGᴮ = DOF6(:ᴺGᴮ)
ᴺFᴮ = FrameRef(
    r = [0,5,0],
    Φ = [0 -1 0; 1 0 0; 0 0 1] # xᴮ = yᴺ, yᴮ = -xᴺ, zᴮ = zᴺ
    ) 
ᴺUᴮ = Connection(Bᵢ=N, Bₒ=B, Fₒ=ᴺFᴮ, G=ᴺGᴮ)

# simple wheel
W = Body(name=:W, m=1, I=I(3), cm=[0,0,0])
ᴮGᵂ = Revolute(:ᴮGᵂ)
ᴮFᴳ = FrameRef(
    r = [1,1,0],
    Φ = [0 -1 0; 1 0 0; 0 0 1] # xʷ = xᴮ = yᴺ, zʷ = yᴮ = -xᴺ, -yʷ = zᴮ = zᴺ (rotates around zʷ)
    ) 
 ᴳFᵂ = FrameRef(
    r = zeros(3),
    Φ = I(3) #seems to make sense to keep wheel coincident with joint frame, otherwise its a pendulum or something
    ) 

ᴮUᵂ = Connection(Bᵢ=B, Fᵢ=ᴮFᴳ, Bₒ=W, Fₒ=ᴳFᵂ, G=ᴮGᵂ)

sys = System(
    name=:bodywheel,
    N = N,
    B = [B,W],
    G = [ᴺGᴮ,ᴮGᵂ],
    U = [ᴺUᴮ,ᴮUᵂ]
)
