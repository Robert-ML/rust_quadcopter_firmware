/// throws compile time error if static assertion fails

#[allow(dead_code)]
pub struct True<const A: bool>;
impl<const A: bool> True<A> {
    #[allow(dead_code)]
    const OK: () = assert!(A, "failed compile time assertion (A == true)");
}

#[allow(dead_code)]
pub struct Le<const A: usize, const B: usize>;
impl<const A: usize, const B: usize> Le<A, B> {
    #[allow(dead_code)]
    const OK: () = assert!(A < B, "failed compile time assertion (A < B)");
}

#[allow(dead_code)]
pub struct LeEq<const A: usize, const B: usize>;
impl<const A: usize, const B: usize> LeEq<A, B> {
    #[allow(dead_code)]
    const OK: () = assert!(A <= B, "failed compile time assertion (A <= B)");
}

#[allow(dead_code)]
pub struct Ge<const A: usize, const B: usize>;
impl<const A: usize, const B: usize> Ge<A, B> {
    #[allow(dead_code)]
    const OK: () = assert!(A > B, "failed compile time assertion (A > B)");
}

#[allow(dead_code)]
pub struct GeEq<const A: usize, const B: usize>;
impl<const A: usize, const B: usize> GeEq<A, B> {
    #[allow(dead_code)]
    const OK: () = assert!(A >= B, "failed compile time assertion (A >= B)");
}
