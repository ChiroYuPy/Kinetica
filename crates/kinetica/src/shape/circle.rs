use std::f32::consts::PI;

use super::MassProperties;
use crate::math::{Aabb, Transform, Vec2};

/// A solid disc.
#[derive(Debug, Clone, Copy, PartialEq)]
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

    /// Moves the circle center away from the body origin.
    #[must_use]
    pub fn with_center(mut self, center: Vec2) -> Self {
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

    pub(crate) fn compute_aabb(&self, transform: &Transform) -> Aabb {
        let center = transform.transform_point(self.center);
        Aabb::new(center - self.radius, center + self.radius)
    }
}
