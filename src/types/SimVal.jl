mutable struct SimVal  
    nominal::Float64
    dispersion::Sampleable
    value::Float64
    function SimVal(nominal) 
        x = new()
        x.nominal = nominal
        x.value = nominal
        return x
    end
    function SimVal(nominal,dispersion)
        x = new()
        x.nominal = nominal
        x.dispersion = dispersion
        x.value = nominal
        return x
    end
end