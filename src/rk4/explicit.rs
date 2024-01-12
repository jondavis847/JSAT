
// follows notation from Dormand, J. R. and P. J. Prince, 
// “A family of embedded Runge-Kutta formulae,” 
// J. Comp. Appl. Math., Vol. 6, 1980, pp 19-26.
#[derive(Debug)]
pub struct ExplicitRK {    
    pub c1: f64,
    pub c2: f64,
    pub c3: f64,
    pub c4: f64,
    pub c5: f64,
    pub c6: f64,
    pub c7: f64,
    pub a21: f64,
    pub a31: f64,
    pub a32: f64,
    pub a41: f64,
    pub a42: f64,
    pub a43: f64,
    pub a51: f64,
    pub a52: f64,
    pub a53: f64,
    pub a54: f64,
    pub a61: f64,
    pub a62: f64,
    pub a63: f64,
    pub a64: f64,
    pub a65: f64,
    pub a71: f64,
    pub a72: f64,
    pub a73: f64,
    pub a74: f64,
    pub a75: f64,
    pub a76: f64,    
    pub b1: f64,
    pub b2: f64,
    pub b3: f64,
    pub b4: f64,
    pub b5: f64,
    pub b6: f64,
    pub b7: f64,
}