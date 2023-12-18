function ECEF_to_ECI(JD)
    R_TodToJ2000, equation_of_equinoxes = TOD_to_J2000(JD)
    return ECEF_to_ECI(JD, R_TodToJ2000, equation_of_equinoxes)
end

function ECEF_to_ECI(JD, R_TodToJ2000, equation_of_equinoxes)
    DAYS_PER_CENTURY = 36525.0
    JulTime2000 = 2451545.0
    GmstCf0 = 6.697374558
    GmstCfD0 = 0.06570982441908
    GmstCfH = 1.00273790935
    GmstCfT2 = 0.000026
    DEG_TO_RAD = pi / 180
    GHA_DEG_PER_HOUR = 15.0
    ## ECEF To ECI (J2000) DCM computation 
    # compute Greenwich Mean Sidereal Time (GMST)
    # Ref: https://aa.usno.navy.mil/faq/docs/GAST.php
    # This simple algorithm for computing apparent sidereal time  
    # to an accuracy of ~0.1 sec, equiv to ~1.5 arcsec on the sky. 
    D = JD - JulTime2000
    T = D / DAYS_PER_CENTURY  # number of centuries since the year 2000
    JD_frac = mod(JD, 1)
    if JD_frac >= 0.5 # 0h~12h
        JD0 = (JD - JD_frac) + 0.5
    else # 12h~24h
        JD0 = (JD - JD_frac) - 0.5
    end
    D0 = JD0 - JulTime2000
    H = (JD - JD0) * 24
    #GMST = 6.697374558 + 0.06570982441908*D0 + 1.00273790935*H + 0.000026*T^2; # [hr]
    # Calculate the Greenwhich Mean Sidereal Time
    GMST = GmstCf0 + GmstCfD0 * D0 + GmstCfH * H + GmstCfT2 * T^2 # [hr]

    # Calculate Greenwhich Apparent Sidereal Time
    # (apply equation of equinoxes correction)
    GAST = GMST + equation_of_equinoxes / (DEG_TO_RAD / GHA_DEG_PER_HOUR)

    GHA = mod(GAST, 24) * GHA_DEG_PER_HOUR * DEG_TO_RAD # [rad]

    # compute earth rotation angle transformation to ECEF
    R_TodToEcef = MMatrix{3,3,Float64}(undef)
    R_TodToEcef[1, 1] = cos(GHA)
    R_TodToEcef[1, 2] = sin(GHA)
    R_TodToEcef[1, 3] = 0.0
    R_TodToEcef[2, 1] = -sin(GHA)
    R_TodToEcef[2, 2] = cos(GHA)
    R_TodToEcef[2, 3] = 0.0
    R_TodToEcef[3, 1] = 0.0
    R_TodToEcef[3, 2] = 0.0
    R_TodToEcef[3, 3] = 1.0
    R_TodToEcef = SMatrix{3,3,Float64}(R_TodToEcef)

    # Convert from ECEF to ECI(J2000)
    R_EcefToEci = R_TodToJ2000 * R_TodToEcef'
    return R_EcefToEci
end