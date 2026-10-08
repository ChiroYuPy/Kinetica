//! Collision shapes and their mass properties.

mod circle;
mod polygon;

pub use circle::Circle;
pub use polygon::{MAX_POLYGON_VERTICES, Polygon, PolygonError};

use crate::math::{Aabb, Transform, Vec2};

/// The geometry of a collider, expressed in the local frame of its body.
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// A solid disc.
    Circle(Circle),
    /// A solid convex polygon.
    Polygon(Polygon),
}

impl Shape {
    /// Computes mass, center of mass and rotational inertia for a uniform density.
    #[must_use]
    pub fn mass_properties(&self, density: f32) -> MassProperties {
        match self {
            Self::Circle(circle) => circle.mass_properties(density),
            Self::Polygon(polygon) => polygon.mass_properties(density),
        }
    }

    /// Returns `true` if `point` (in world space) is inside the shape placed at `transform`.
    #[must_use]
    pub fn contains_point(&self, transform: &Transform, point: Vec2) -> bool {
        let local = transform.inverse_transform_point(point);
        match self {
            Self::Circle(circle) => local.distance_squared(circle.center()) <= circle.radius() * circle.radius(),
            Self::Polygon(polygon) => polygon
                .vertices()
                .iter()
                .zip(polygon.normals())
                .all(|(vertex, normal)| normal.dot(local - *vertex) <= 0.0),
        }
    }

    /// Intersects the segment from `origin` to `origin + max_fraction * translation` with the
    /// shape placed at `transform`.
    ///
    /// A segment starting inside the shape does not hit it.
    #[must_use]
    pub fn cast_ray(
        &self,
        transform: &Transform,
        origin: Vec2,
        translation: Vec2,
        max_fraction: f32,
    ) -> Option<RayIntersection> {
        let local_origin = transform.inverse_transform_point(origin);
        let local_translation = transform.rotation.inverse_rotate(translation);
        let (fraction, local_normal) = match self {
            Self::Circle(circle) => circle.cast_ray(local_origin, local_translation, max_fraction),
            Self::Polygon(polygon) => polygon.cast_ray(local_origin, local_translation, max_fraction),
        }?;
        Some(RayIntersection { fraction, normal: transform.rotation.rotate(local_normal) })
    }

    /// Computes the world-space bounding box of the shape placed at `transform`.
    #[must_use]
    pub fn compute_aabb(&self, transform: &Transform) -> Aabb {
        match self {
            Self::Circle(circle) => circle.compute_aabb(transform),
            Self::Polygon(polygon) => polygon.compute_aabb(transform),
        }
    }
}

impl From<Circle> for Shape {
    fn from(circle: Circle) -> Self {
        Self::Circle(circle)
    }
}

impl From<Polygon> for Shape {
    fn from(polygon: Polygon) -> Self {
        Self::Polygon(polygon)
    }
}

/// Where a ray enters a shape.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayIntersection {
    /// Fraction of the ray translation travelled before the hit.
    pub fraction: f32,
    /// Outward surface normal at the hit, in world space.
    pub normal: Vec2,
}

/// Mass distribution of a shape or a body.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct MassProperties {
    /// Mass, in kilograms.
    pub mass: f32,
    /// Center of mass, in the local frame.
    pub center: Vec2,
    /// Rotational inertia about the center of mass, in kg·m².
    pub inertia: f32,
}
