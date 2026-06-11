pub mod bouncing_balls;
pub mod ball_shower;
pub mod mixed_shower;
pub mod pyramid;
pub mod stairs;
pub mod zero_gravity;

pub trait Scene {
    fn name(&self) -> &str;
    fn setup(&self, world: &mut World, width: f32, height: f32);
    fn update(&mut self, world: &mut World, dt: f32);
}

pub use bouncing_balls::BouncingBalls;
pub use ball_shower::BallShower;
use kinetica::World;
pub use mixed_shower::MixedShower;
pub use pyramid::Pyramid;
pub use stairs::Stairs;
pub use zero_gravity::ZeroGravity;
