use crate::math::{AABB, Vec2};

#[derive(Clone, Debug)]
pub enum Shape {
    Circle(f32),
    Rectangle(Vec2),
}

impl Shape {
    pub fn circle(radius: f32) -> Self {
        Self::Circle(radius)
    }

    pub fn rectangle(width: f32, height: f32) -> Self {
        Self::Rectangle(Vec2::new(width, height))
    }

    pub fn compute_aabb(&self, position: Vec2) -> AABB {
        match self {
            Shape::Circle(radius) => AABB::new(position, Vec2::splat(*radius)),
            Shape::Rectangle(extents) => AABB::new(position, *extents / 2.0),
        }
    }
}
