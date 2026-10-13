//! Collision shapes and their mass properties.

mod capsule;
mod circle;
mod polygon;
mod segment;

pub use capsule::Capsule;
pub use circle::Circle;
pub use polygon::{MAX_POLYGON_VERTICES, Polygon, PolygonError};
pub use segment::{ChainSegment, Segment};

use crate::constants::LINEAR_SLOP;
use crate::math::{Aabb, Transform, Vec2};

/// The geometry of a collider, expressed in the local frame of its body.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Shape {
    /// A solid disc.
    Circle(Circle),
    /// A solid capsule: a segment with a radius.
    Capsule(Capsule),
    /// A solid convex polygon.
    Polygon(Polygon),
    /// A two-sided segment without thickness or mass.
    Segment(Segment),
    /// A one-sided segment of a chain, as built by `World::add_chain` in `kinetica`.
    ChainSegment(ChainSegment),
}

impl Shape {
    /// Computes mass, center of mass and rotational inertia for a uniform density.
    #[must_use]
    pub fn mass_properties(&self, density: f32) -> MassProperties {
        match self {
            Self::Circle(circle) => circle.mass_properties(density),
            Self::Capsule(capsule) => capsule.mass_properties(density),
            Self::Polygon(polygon) => polygon.mass_properties(density),
            Self::Segment(segment) => segment.mass_properties(),
            Self::ChainSegment(chain) => chain.segment().mass_properties(),
        }
    }

    /// Distance from the center of the shape to its closest edge: how far it can move before
    /// it could pass through something thin.
    #[must_use]
    pub fn min_extent(&self) -> f32 {
        match self {
            Self::Circle(circle) => circle.radius(),
            Self::Capsule(capsule) => capsule.radius(),
            // No thickness at all: any motion may cross something.
            Self::Segment(_) | Self::ChainSegment(_) => LINEAR_SLOP,
            Self::Polygon(polygon) => polygon
                .vertices()
                .iter()
                .zip(polygon.normals())
                .map(|(&vertex, normal)| normal.dot(vertex - polygon.centroid()))
                .fold(f32::INFINITY, f32::min),
        }
    }

    /// Returns `true` if `point` (in world space) is inside the shape placed at `transform`.
    /// Segments have no inside.
    #[must_use]
    pub fn contains_point(&self, transform: &Transform, point: Vec2) -> bool {
        let local = transform.inverse_transform_point(point);
        match self {
            Self::Circle(circle) => local.distance_squared(circle.center()) <= circle.radius() * circle.radius(),
            Self::Capsule(capsule) => capsule.contains_point(local),
            Self::Segment(_) | Self::ChainSegment(_) => false,
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
    /// A segment starting inside the shape does not hit it. Segments and chains are hit from
    /// both sides.
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
            Self::Capsule(capsule) => capsule.cast_ray(local_origin, local_translation, max_fraction),
            Self::Polygon(polygon) => polygon.cast_ray(local_origin, local_translation, max_fraction),
            Self::Segment(segment) => segment.cast_ray(local_origin, local_translation, max_fraction),
            Self::ChainSegment(chain) => chain.segment().cast_ray(local_origin, local_translation, max_fraction),
        }?;
        Some(RayIntersection { fraction, normal: transform.rotation.rotate(local_normal) })
    }

    /// The same shape, rotated then moved by `placement`, both in the local frame.
    #[must_use]
    pub fn transformed(&self, placement: &Transform) -> Self {
        let point = |point: Vec2| placement.transform_point(point);
        let segment = |segment: &Segment| Segment::new(point(segment.start()), point(segment.end()));
        match self {
            Self::Circle(circle) => Circle::new(circle.radius()).with_center(point(circle.center())).into(),
            Self::Capsule(capsule) => {
                Capsule::new(point(capsule.start()), point(capsule.end()), capsule.radius()).into()
            }
            Self::Polygon(polygon) => polygon.transformed(placement).into(),
            Self::Segment(original) => segment(original).into(),
            Self::ChainSegment(chain) => {
                ChainSegment::new(segment(chain.segment()), chain.previous().map(point), chain.next().map(point)).into()
            }
        }
    }

    /// Computes the world-space bounding box of the shape placed at `transform`.
    #[must_use]
    pub fn compute_aabb(&self, transform: &Transform) -> Aabb {
        match self {
            Self::Circle(circle) => circle.compute_aabb(transform),
            Self::Capsule(capsule) => capsule.compute_aabb(transform),
            Self::Polygon(polygon) => polygon.compute_aabb(transform),
            Self::Segment(segment) => segment.compute_aabb(transform),
            Self::ChainSegment(chain) => chain.segment().compute_aabb(transform),
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

impl From<Capsule> for Shape {
    fn from(capsule: Capsule) -> Self {
        Self::Capsule(capsule)
    }
}

impl From<Segment> for Shape {
    fn from(segment: Segment) -> Self {
        Self::Segment(segment)
    }
}

impl From<ChainSegment> for Shape {
    fn from(chain: ChainSegment) -> Self {
        Self::ChainSegment(chain)
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MassProperties {
    /// Mass, in kilograms.
    pub mass: f32,
    /// Center of mass, in the local frame.
    pub center: Vec2,
    /// Rotational inertia about the center of mass, in kg·m².
    pub inertia: f32,
}
