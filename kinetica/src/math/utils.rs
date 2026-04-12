use crate::math::Vec2;
use crate::math::constants::EPSILON;

pub fn cross(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

pub fn perp(v: Vec2) -> Vec2 {
    Vec2::new(-v.y, v.x)
}

pub fn distance_squared(a: Vec2, b: Vec2) -> f32 {
    (a - b).length_squared()
}

pub fn distance(a: Vec2, b: Vec2) -> f32 {
    (a - b).length()
}

pub fn normalize(v: Vec2) -> Vec2 {
    let len2 = v.length_squared();
    if len2 < EPSILON * EPSILON {
        Vec2::ZERO
    } else {
        v / len2.sqrt()
    }
}