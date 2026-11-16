pub mod lqi;
pub mod lqr;
pub mod pid;

pub use lqi::LqiController;
pub use lqr::LqrController;
pub use pid::PidController;

pub trait Controller {
    fn name(&self) -> &'static str;
    fn reset(&mut self);
    fn command(&mut self, reference_c: f64, measured_c: f64) -> f64;
}
