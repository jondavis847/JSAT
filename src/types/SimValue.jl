struct SimValue{T}
    nominal::T
    dispersion::Sampleable
    SimValue(nominal) = new(nominal)
end