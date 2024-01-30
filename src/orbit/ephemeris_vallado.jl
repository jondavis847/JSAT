using StaticArrays, Dates

"""
    sun_position_v(jd::Float64)
    sun_position_v(jd::DateTime)

Calculates the position in meters of the sun wrt earth in the J2000 frame based on the julian date (`jd`).

Based on the implementation from Vallado Algorithm 29.

# Examples
```julia-repl
julia> sun_position_v(2.4494705e6)
3-element SVector{3, Float64} with indices SOneTo(3):
 1.1950851722081454e11
 8.408771651701007e10
 3.645777437746923e10

julia> @btime sun_position_v(\$jd)
 140.909 ns (0 allocations: 0 bytes)
```
"""
function sun_position_v(jd::Float64)
    AU2m = 149597870691
    T_ut1 = (jd - 2451545.0) / 36525.0
    e = 0.016708617 - 0.000042037 * T_ut1 - 0.0000001236 * T_ut1^2
    λm = 280.4606184 + 36000.77005361 * T_ut1    
    M = 357.5277233 + 35999.05034 * T_ut1    

    λe = λm + 1.914666471 * sind(M) + 0.019994643 * sind(2 * M)
    rmag = 1.000140612 - 0.016708617 * cosd(M) - 0.000139589 * cosd(2 * M)
    ϵ = 23.439291 - 0.0130042 * T_ut1
    r1 = rmag * cosd(λe) * AU2m
    r2 = rmag * cosd(ϵ) * sind(λe) * AU2m
    r3 = rmag * sind(ϵ) * sind(λe) * AU2m
    r = SVector{3,Float64}(r1, r2, r3)
    return r
end

sun_position_v(jd::DateTime) = sun_position_v(datetime2julian(jd))

"""
    moon_position_v(jd::Float64)
    moon_position_v(jd::DateTime)

Calculates the position in meters of the moon wrt earth in the J2000 frame based on the julian date (`jd`).

Based on the implementation from Vallado Algorithm 31.

# Examples
```julia-repl
julia> moon_position_v(2.4494705e6)
3-element SVector{3, Float64} with indices SOneTo(3):
 -1.3418105380693362e9
 -3.114333240566382e9 
 -1.2663756172114737e9

julia> moon_position_v(DateTime(1994,4,28,0,0,0))
3-element SVector{3, Float64} with indices SOneTo(3):
 -1.3418105380693362e9
 -3.114333240566382e9
 -1.2663756172114737e9

julia> @btime moon_position_v(\$jd)
 320.000 ns (0 allocations: 0 bytes)
```
"""
function moon_position_v(jd)
    er = 63781363
    T_ut1 = (jd - 2451545.0) / 36525.0
    λe = (218.32 + (481267.8813 * T_ut1) + 6.29 * sind(134.9 + 477198.85 * T_ut1)
          -
          1.27 * sind(259.2 - 413335.38 * T_ut1) + 0.66 * sind(235.7 + 890534.23 * T_ut1)
          + 0.21 * sind(269.9 + 954397.70 * T_ut1) - 0.19 * sind(357.5 + 35999.05 * T_ut1)
          -
          0.11 * sind(186.6 + 966404.05 * T_ut1))
    ϕe = (5.13 * sind(93.3 + 483202.03 * T_ut1) + 0.28 * sind(228.2 + 960400.87 * T_ut1)
          -
          0.28 * sind(318.3 + 6003.18 * T_ut1) - 0.17 * sind(217.6 - 407332.20 * T_ut1))

    parallax = (0.9508 + 0.0518 * cosd(134.9 + 477198.85 * T_ut1)
                + 0.0095 * cosd(259.2 - 413335.38 * T_ut1) + 0.0078 * cosd(235.7 + 890534.23 * T_ut1)
                + 0.0028 * cosd(269.9 + 954397.70))

    ϵ = 23.439291 - 0.0130042 * T_ut1
    rmag = er / sind(parallax)

    cϕ = cosd(ϕe)
    sλ = sind(λe)
    sϕ = sind(ϕe)
    sϵ = sind(ϵ)
    cϵ = cosd(ϵ)

    r1 = rmag * cϕ * cosd(λe)
    r2 = rmag * (cϵ * cϕ * sλ - sϵ * sϕ)
    r3 = rmag * (sϵ * cϕ * sλ + cϵ * sϕ)
    return SVector{3,Float64}(r1, r2, r3)
end

moon_position_v(jd::DateTime) = moon_position_v(datetime2julian(jd))