use std::f32::consts::PI;

use super::MassProperties;
use crate::math::{Aabb, Transform, Vec2};

/// A solid disc.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle {
    center: Vec2,
    radius: f32,
}

impl Circle {
    /// Creates a circle centered on the body origin.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not strictly positive and finite.
    #[must_use]
    pub fn new(radius: f32) -> Self {
        assert!(radius.is_finite() && radius > 0.0, "circle radius must be positive, got {radius}");
        Self { center: Vec2::ZERO, radius }
    }

    /// Moves the circle center away from the body origin; colliders do it with
    /// [`ColliderDesc::offset`](crate::ColliderDesc::offset).
    pub(crate) fn with_center(mut self, center: Vec2) -> Self {
        self.center = center;
        self
    }

    /// Center of the circle, in the local frame.
    #[must_use]
    pub fn center(&self) -> Vec2 {
        self.center
    }

    /// Radius of the circle.
    #[must_use]
    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub(crate) fn mass_properties(&self, density: f32) -> MassProperties {
        let radius_squared = self.radius * self.radius;
        let mass = density * PI * radius_squared;
        MassProperties { mass, center: self.center, inertia: 0.5 * mass * radius_squared }
    }

    /// Ray cast in the local frame; returns the fraction and the local normal.
    pub(crate) fn cast_ray(&self, origin: Vec2, translation: Vec2, max_fraction: f32) -> Option<(f32, Vec2)> {
        cast_ray_at_disc(self.center, self.radius, origin, translation, max_fraction)
    }

    pub(crate) fn compute_aabb(&self, transform: &Transform) -> Aabb {
        let center = transform.transform_point(self.center);
        Aabb::new(center - self.radius, center + self.radius)
    }
}

/// Ray cast against a disc; rays starting inside do not hit it.
pub(super) fn cast_ray_at_disc(
    center: Vec2,
    radius: f32,
    origin: Vec2,
    translation: Vec2,
    max_fraction: f32,
) -> Option<(f32, Vec2)> {
    // Solve |origin + t·translation − center|² = radius² for the smallest t.
    let offset = origin - center;
    let c = offset.length_squared() - radius * radius;
    if c < 0.0 {
        return None;
    }
    let a = translation.length_squared();
    let b = offset.dot(translation);
    let discriminant = b * b - a * c;
    if a == 0.0 || discriminant < 0.0 {
        return None;
    }
    let fraction = (-b - discriminant.sqrt()) / a;
    if !(0.0..=max_fraction).contains(&fraction) {
        return None;
    }
    Some((fraction, (offset + fraction * translation).normalize()))
}
