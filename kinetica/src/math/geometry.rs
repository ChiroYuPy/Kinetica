use crate::math::Vec2;

pub fn project(a: Vec2, b: Vec2) -> Vec2 {
    let denom = b.length_squared();
    if denom == 0.0 {
        return Vec2::ZERO;
    }
    b * (a.dot(b) / denom)
}

pub fn reflect(v: Vec2, normal: Vec2) -> Vec2 {
    v - 2.0 * v.dot(normal) * normal
}

pub fn clamp_length(v: Vec2, max: f32) -> Vec2 {
    let len2 = v.length_squared();
    if len2 > max * max {
        v.normalize() * max
    } else {
        v
    }
}

pub fn angle(a: Vec2, b: Vec2) -> f32 {
    let denom = a.length() * b.length();
    if denom == 0.0 {
        return 0.0;
    }
    (a.dot(b) / denom).clamp(-1.0, 1.0).acos()
}