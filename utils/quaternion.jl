using LinearAlgebra, StaticArrays

#inverse quaternion,  if q is SVector, btime 31.584 ns (1 allocations: 48 bytes)
qinv(q) = normalize(q .* SVector{4,Float64}(-1, -1, -1, 1))# / norm(q)^2

#quaternion to rotation vector,  #btime 109.594 ns (2 allocations: 160 bytes)
qtov(q) = 2 * atan(norm(view(q, Base.OneTo(3))), q[4]) * normalize(view(q, Base.OneTo(3)))

#qtov(q) = 2 * acos(q[4]) * normalize(view(q,Base.OneTo(3)))

#rotation matrix to quaternion
function atoq(A) # if A is a SMatrix btime 43.016 ns (1 allocations: 48 bytes)   
    # From Markley, singular when q[4] gets close to 0
    imax = findmax(abs.(SVector{4,Float64}(
        A[1, 1], A[2, 2], A[3, 3], tr(A)
    )))[2]
    if imax == 1
        q = SVector{4,Float64}(
            1 + 2 * A[1, 1] - tr(A),
            A[1, 2] + A[2, 1],
            A[1, 3] + A[3, 1],
            A[2, 3] - A[3, 2]
        )
    elseif imax == 2
        q = SVector{4,Float64}(
            A[2, 1] + A[1, 2],
            1 + 2 * A[2, 2] - tr(A),
            A[2, 3] + A[3, 2],
            A[3, 1] - A[1, 3]
        )
    elseif imax == 3
        q = SVector{4,Float64}(
            A[3, 1] + A[1, 3],
            A[3, 2] + A[2, 3],
            1 + 2 * A[3, 3] - tr(A),
            A[1, 2] - A[2, 1]
        )
    else # imax == 4
        q = SVector{4,Float64}(
            A[2, 3] - A[3, 2],
            A[3, 1] - A[1, 3],
            A[1, 2] - A[2, 1],
            1 + tr(A)
        )
    end
end


function qtoe(q)
    # returns the euler angles (in radians) corresponding to a quaternions
    w = q[4]
    x = q[1]
    y = q[2]
    z = q[3]

    ysqr = y * y

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + ysqr)
    x_ang = atan(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    if (t2 > 1.0)
        t2 = 1.0
    end
    if (t2 < -1.0)
        t2 = -1.0
    end
    y_ang = asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (ysqr + z * z)
    z_ang = atan(t3, t4)

    return @SVector [x_ang, y_ang, z_ang]

end

function qmult(q_A2B, q_B2C) # 162.533 ns (3 allocations: 288 bytes)
    # q_A2C = qmult(q_A2B,q_B2C, varargin)
    #
    # Quaternion multiplication:   q_A2B * q_B2C    = q_A2C 
    #                           A(q_B2C) * A(q_A2B) = A(q_A2C)
    #
    # this multiplication implementation follows the \odot convention from 
    # Markley, F. Landis, and John L. Crassidis. "Fundamentals of Spacecraft 
    # Attitude Determination and Control." (2014): 978-1.  page 37
    #
    # let quaternions be denoted q = [qv; qs] and p = [pv; ps]
    # then   q \odot p = [ ps*qv + qs*pv + cross(qv,pv);...
    #                              qs*ps - dot(qv,pv)  ];
    #

    q_A2C = SA[
        q_B2C[4]*q_A2B[1]+q_B2C[3]*q_A2B[2]-q_B2C[2]*q_A2B[3]+q_B2C[1]*q_A2B[4],
        -q_B2C[3]*q_A2B[1]+q_B2C[4]*q_A2B[2]+q_B2C[1]*q_A2B[3]+q_B2C[2]*q_A2B[4],
        q_B2C[2]*q_A2B[1]-q_B2C[1]*q_A2B[2]+q_B2C[4]*q_A2B[3]+q_B2C[3]*q_A2B[4],
        -q_B2C[1]*q_A2B[1]-q_B2C[2]*q_A2B[2]-q_B2C[3]*q_A2B[3]+q_B2C[4]*q_A2B[4]
    ]


    q_A2C = normalize(q_A2C)
    #= This messed up solar vec body frame calculation
        if q_A2C[4] < 0
            q_A2C = -q_A2C
        end
    =#
    return q_A2C
end # qmult()

#qvrot(q, v, transform = true) = transform ? inv(QuatRotation(q[[4,1,2,3]]))*v : QuatRotation(q[[4,1,2,3]])*v

function qvrot(q, v, transform=true)

    # initalize value
    v_tmp = normalize(v)
    vMag = norm(v)

    v_aug = [v_tmp..., 0]

    q = transform ? q : qinv(q)

    q_tmp = qinv(q)
    q_tmp = qmult(q_tmp, v_aug)
    q_tmp = qmult(q_tmp, q)

    return view(q_tmp, Base.OneTo(3)) * vMag

end

qtoa(q::Vector{Float64}) = qtoa(SVector{4,Float64}(q))
function qtoa(q::SVector{4,Float64}) #0 allocations 20 ns
    I3 = SMatrix{3,3,Float64}(1,0,0,0,1,0,0,0,1); #allocation free identity matrix
    i123 = SVector{3,Int64}(1,2,3) 
    q123 = q[i123] #q[1:3]

    I3*(q[4]^2 - q123' * q123) + 2 * q123 * q123' - 2 * scross(q123 * q[4])
end

scross(x::Vector{Float64}) = scross(SVector{3,Float64}(x))
scross(x) = SMatrix{3,3,Float64}(0,x[3],-x[2],-x[3],0,x[1],x[2],-x[1],0)