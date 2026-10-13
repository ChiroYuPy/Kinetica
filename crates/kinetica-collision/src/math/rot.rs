use std::ops::Mul;

use glam::{Vec2, vec2};

/// A 2D rotation, stored as the cosine and sine of its angle.
///
/// Storing the rotation this way avoids calling trigonometric functions while stepping the
/// simulation, which keeps the step fast and reproducible.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Rot {
    cos: f32,
    sin: f32,
}

impl Rot {
    /// The rotation by zero radians.
    pub const IDENTITY: Self = Self { cos: 1.0, sin: 0.0 };

    /// Creates a rotation from an angle in radians (counter-clockwise).
    #[must_use]
    pub fn from_angle(angle: f32) -> Self {
        let (sin, cos) = angle.sin_cos();
        Self { cos, sin }
    }

    /// Returns the angle of this rotation in radians, in `[-π, π]`.
    #[must_use]
    pub fn angle(self) -> f32 {
        self.sin.atan2(self.cos)
    }

    /// Cosine of the rotation angle.
    #[must_use]
    pub fn cos(self) -> f32 {
        self.cos
    }

    /// Sine of the rotation angle.
    #[must_use]
    pub fn sin(self) -> f32 {
        self.sin
    }

    /// Returns the opposite rotation.
    #[must_use]
    pub fn inverse(self) -> Self {
        Self { cos: self.cos, sin: -self.sin }
    }

    /// Rotates a vector by this rotation.
    #[must_use]
    pub fn rotate(self, v: Vec2) -> Vec2 {
        vec2(self.cos * v.x - self.sin * v.y, self.sin * v.x + self.cos * v.y)
    }

    /// Rotates a vector by the inverse of this rotation.
    #[must_use]
    pub fn inverse_rotate(self, v: Vec2) -> Vec2 {
        vec2(self.cos * v.x + self.sin * v.y, -self.sin * v.x + self.cos * v.y)
    }

    /// Advances the rotation by a small angle without using trigonometry.
    ///
    /// This is a first-order integration followed by a renormalisation, so it slightly
    /// under-rotates for large angles. It is accurate for the small angles of a time step.
    #[must_use]
    pub fn integrate(self, delta_angle: f32) -> Self {
        Self { cos: self.cos - delta_angle * self.sin, sin: self.sin + delta_angle * self.cos }.normalized()
    }

    fn normalized(self) -> Self {
        let length = (self.cos * self.cos + self.sin * self.sin).sqrt();
        if length > 0.0 { Self { cos: self.cos / length, sin: self.sin / length } } else { Self::IDENTITY }
    }
}

impl Default for Rot {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// Composes two rotations: `a * b` rotates by `b`, then by `a`.
impl Mul for Rot {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self {
        Self { cos: self.cos * rhs.cos - self.sin * rhs.sin, sin: self.sin * rhs.cos + self.cos * rhs.sin }
    }
}

impl Mul<Vec2> for Rot {
    type Output = Vec2;

    fn mul(self, rhs: Vec2) -> Vec2 {
        self.rotate(rhs)
    }
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_2;

    use super::*;

    #[test]
    fn quarter_turn_maps_x_to_y() {
        let rotated = Rot::from_angle(FRAC_PI_2).rotate(Vec2::X);
        assert!(rotated.abs_diff_eq(Vec2::Y, 1e-6));
    }

    #[test]
    fn inverse_rotate_undoes_rotate() {
        let rot = Rot::from_angle(0.7);
        let v = vec2(3.0, -2.0);
        assert!(rot.inverse_rotate(rot.rotate(v)).abs_diff_eq(v, 1e-5));
    }

    #[test]
    fn composition_adds_angles() {
        let composed = Rot::from_angle(0.3) * Rot::from_angle(0.5);
        assert!((composed.angle() - 0.8).abs() < 1e-6);
    }

    #[test]
    fn integration_stays_normalised() {
        let mut rot = Rot::IDENTITY;
        for _ in 0..1000 {
            rot = rot.integrate(0.05);
        }
        let length = (rot.cos() * rot.cos() + rot.sin() * rot.sin()).sqrt();
        assert!((length - 1.0).abs() < 1e-5);
    }
}
