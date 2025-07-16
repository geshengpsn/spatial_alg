#![allow(non_camel_case_types)]
use num_traits::{FloatConst, NumAssignOps, real::Real as NumReal};
use std::fmt::Debug;

/// # real number trait
/// support ops: +, -, *, /, %, +=, -=, *=, /=, %=
///
/// consts: 0, 1, π, 1/π, ln2, ......
///
/// compare ops: >, <, <=, >=
pub trait Real: NumReal + Debug + NumAssignOps + FloatConst + 'static {}

impl<T> Real for T where T: NumReal + Debug + NumAssignOps + FloatConst + 'static {}

mod nalgebra_impl;
mod utils;

pub trait se3 {
    type SE3;
    type Adj;
    fn exp(&self) -> Self::SE3;
    fn adj(&self) -> Self::Adj;
}

pub trait SE3<se3> {
    // type se3;
    type adjoint;
    fn log(&self) -> se3;
    fn inv(&self) -> Self;
    fn lee_adjoint(&self) -> Self::adjoint;
}

pub trait so3 {
    type SO3;
    fn exp(&self) -> Self::SO3;
}

pub trait SO3<so3> {
    // type so3;
    fn log(&self) -> so3;
    fn inv(&self) -> Self;
}

pub mod prelude {
    pub use super::{SE3, SO3, se3, so3};
}
