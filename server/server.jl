using HTTP, Sockets
includet("..\\src\\blue42.jl")
function jsat_server()    
    ROUTER = HTTP.Router()

    routerIndex(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\index.html"))
    routerCss(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\css\\jsat.css"))
    routerJs(req::HTTP.Request) = HTTP.Response(200, read("server\\public\\js\\jsat.js"))
    function routerSimulate(req::HTTP.Request) 
        N = WorldFrame()
        j1 = Revolute(:j1,pi/4,0.)
        p1 = Body(:p1,1,I(3),[0,-1,0])
        fp1 = eye(Cartesian)
        fs1 = eye(Cartesian)

        j2 = Revolute(:j2,pi/4,0.)
        p2 = Body(:p2,1,I(3),[0,-1,0])
        fp2 = Cartesian(I(3),[0,-1,0])
        fs2 = eye(Cartesian)

        connect!(j1,N,p1,fp1,fs1)
        connect!(j2,p1,p2,fp2,fs2)

        B = OffsetArray([N,p1,p2], 0:2) #offset array lets us do 0 based indexing which matches featherstone notation
        G = [j1,j2]

        sys = MultibodySystem(:doublepend,B,G)

        sol = simulate(sys,(0,10))
        dump(sol)
        HTTP.Response(200, "success")
    end

    HTTP.register!(ROUTER, "GET", "/", routerIndex)
    HTTP.register!(ROUTER, "GET", "/css/jsat.css", routerCss)
    HTTP.register!(ROUTER, "GET", "/js/jsat.js", routerJs)

    HTTP.register!(ROUTER, "GET", "/simulate", routerSimulate)

    server = HTTP.serve!(ROUTER, ip"127.0.0.1", 80)    
    printstyled("servers up! ctrl+click url to go : http://127.0.0.1:80", color=:light_yellow)
    return server

end