function TOD_to_J2000(JD)
    DAYS_PER_CENTURY = 36525
    DEG_TO_RAD = pi / 180
    ARCSEC_TO_RAD = 1 / 3600 * pi / 180

    PrecZetaCf1 = 2306.2181
    PrecZetaCf2 = 0.30188
    PrecZetaCf3 = 0.017998
    PrecZCf1 = 2306.2181
    PrecZCf2 = 1.09468
    PrecZCf3 = 0.018203
    PrecThetaCf1 = 2004.3109
    PrecThetaCf2 = -0.42665
    PrecThetaCf3 = -0.041833
    NutAscNodeMoonCf0 = 125.04452
    NutAscNodeMoonCf1 = -1934.136261
    NutAscNodeMoonCf2 = 0.0020708
    NutAscNodeMoonCf3 = 1 / 450000
    NutLongSunCf0 = 280.4665
    NutLongSunCf1 = 36000.7698
    NutLongMoonCf0 = 218.3165
    NutLongMoonCf1 = 481267.8813
    NutMeanOblCf0 = 23.43929111
    NutMeanOblCf1 = -0.0130047
    NutMeanOblCf2 = -0.0000001639
    NutMeanOblCf3 = 0.0000005036
    NutDeltaOblCf1 = 9.2
    NutDeltaOblCf2 = 0.57
    NutDeltaOblCf3 = 0.10
    NutDeltaOblCf4 = -0.09
    NutLongCf1 = -17.20
    NutLongCf2 = -1.32
    NutLongCf3 = -0.23
    NutLongCf4 = 0.21

    T = (JD - 2451545.0) / DAYS_PER_CENTURY

    # Precession matrix (P) calculation
    precessionMtx = MMatrix{3,3,Float64}(undef)
    zeta = (PrecZetaCf1 * T + PrecZetaCf2 * T^2 + PrecZetaCf3 * T^3) * ARCSEC_TO_RAD
    Z = (PrecZCf1 * T + PrecZCf2 * T^2 + PrecZCf3 * T^3) * ARCSEC_TO_RAD
    theta = (PrecThetaCf1 * T + PrecThetaCf2 * T^2 + PrecThetaCf3 * T^3) * ARCSEC_TO_RAD

    precessionMtx[1, 1] = -sin(zeta) * sin(Z) + cos(zeta) * cos(Z) * cos(theta)
    precessionMtx[2, 1] = -cos(zeta) * sin(Z) - sin(zeta) * cos(Z) * cos(theta)
    precessionMtx[3, 1] = -cos(Z) * sin(theta)
    precessionMtx[1, 2] = sin(zeta) * cos(Z) + cos(zeta) * sin(Z) * cos(theta)
    precessionMtx[2, 2] = cos(zeta) * cos(Z) - sin(zeta) * sin(Z) * cos(theta)
    precessionMtx[3, 2] = -sin(Z) * sin(theta)
    precessionMtx[1, 3] = cos(zeta) * sin(theta)
    precessionMtx[2, 3] = -sin(zeta) * sin(theta)
    precessionMtx[3, 3] = cos(theta)
    precessionMtx = SMatrix{3,3,Float64}(precessionMtx)

    # Nutation matrix (N) calculation
    nutationMtx = MMatrix{3,3,Float64}(undef)
    Omega = (NutAscNodeMoonCf0 + NutAscNodeMoonCf1 * T + NutAscNodeMoonCf2 * T^2 + NutAscNodeMoonCf3 * T^3) * DEG_TO_RAD
    L = (NutLongSunCf0 + NutLongSunCf1 * T) * DEG_TO_RAD
    Lprime = (NutLongMoonCf0 + NutLongMoonCf1 * T) * DEG_TO_RAD
    eps_m = (NutMeanOblCf0 + NutMeanOblCf1 * T + NutMeanOblCf2 * T^2 + NutMeanOblCf3 * T^3) * DEG_TO_RAD
    deps = (NutDeltaOblCf1 * cos(Omega) + NutDeltaOblCf2 * cos(2 * L) + NutDeltaOblCf3 * cos(2 * Lprime) + NutDeltaOblCf4 * cos(2 * Omega)) * ARCSEC_TO_RAD
    eps_t = eps_m + deps
    dPsi = (NutLongCf1 * sin(Omega) + NutLongCf2 * sin(2 * L) + NutLongCf3 * sin(2 * Lprime) + NutLongCf4 * sin(2 * Omega)) * ARCSEC_TO_RAD
    nutationMtx[1, 1] = cos(dPsi)
    nutationMtx[2, 1] = -sin(dPsi) * cos(eps_m)
    nutationMtx[3, 1] = -sin(dPsi) * sin(eps_m)
    nutationMtx[1, 2] = sin(dPsi) * cos(eps_t)
    nutationMtx[2, 2] = cos(eps_t) * cos(eps_m) * cos(dPsi) + sin(eps_t) * sin(eps_m)
    nutationMtx[3, 2] = cos(eps_t) * sin(eps_m) * cos(dPsi) - sin(eps_t) * cos(eps_m)
    nutationMtx[1, 3] = sin(dPsi) * sin(eps_t)
    nutationMtx[2, 3] = sin(eps_t) * cos(eps_m) * cos(dPsi) - cos(eps_t) * sin(eps_m)
    nutationMtx[3, 3] = sin(eps_t) * sin(eps_m) * cos(dPsi) + cos(eps_t) * cos(eps_m)
    nutationMtx = SMatrix{3,3,Float64}(nutationMtx)

    R_TodToJ2000 = precessionMtx * nutationMtx

    # calculate equation of the equinoxes from the nutation in
    # longitude and obliquity in radians
    equation_of_equinoxes = dPsi * cos(eps_t)

    return R_TodToJ2000, equation_of_equinoxes
end