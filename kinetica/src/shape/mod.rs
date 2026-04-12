pub mod circle;

pub use circle::CircleShape;

#[derive(Clone, Debug)]
pub enum Shape {
    Circle(CircleShape),
}