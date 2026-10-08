//! Colliders: shapes attached to bodies, with their surface material.

use crate::arena::Index;
use crate::body::BodyHandle;
use crate::math::{Aabb, Transform, Vec2};
use crate::shape::{Circle, MassProperties, Polygon, PolygonError, Shape};

/// Handle to a [`Collider`] stored in a [`World`](crate::World).
///
/// A handle stays valid until its collider (or the body owning it) is removed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ColliderHandle(pub(crate) Index);

/// Description of a collider to attach to a body, built with chained setters.
///
/// ```
/// use kinetica::ColliderDesc;
///
/// let desc = ColliderDesc::circle(0.5).density(2.0).restitution(0.8);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct ColliderDesc {
    /// Geometry, in the local frame of the body.
    pub shape: Shape,
    /// Mass per square meter, in kg/m².
    pub density: f32,
    /// Friction coefficient, usually between `0` (ice) and `1` (rubber).
    pub friction: f32,
    /// Bounciness, between `0` (no bounce) and `1` (perfect bounce).
    pub restitution: f32,
}

impl ColliderDesc {
    /// Describes a collider with the given shape and default material.
    #[must_use]
    pub fn new(shape: impl Into<Shape>) -> Self {
        Self { shape: shape.into(), density: 1.0, friction: 0.6, restitution: 0.0 }
    }

    /// Describes a circle centered on the body origin.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not strictly positive and finite.
    #[must_use]
    pub fn circle(radius: f32) -> Self {
        Self::new(Circle::new(radius))
    }

    /// Describes a rectangle centered on the body origin.
    ///
    /// # Panics
    ///
    /// Panics if a half extent is not strictly positive and finite.
    #[must_use]
    pub fn rectangle(half_width: f32, half_height: f32) -> Self {
        Self::new(Polygon::rectangle(half_width, half_height))
    }

    /// Describes a regular polygon centered on the body origin.
    ///
    /// # Panics
    ///
    /// See [`Polygon::regular`].
    #[must_use]
    pub fn regular_polygon(sides: usize, radius: f32) -> Self {
        Self::new(Polygon::regular(sides, radius))
    }

    /// Describes the convex hull of the given points.
    ///
    /// # Errors
    ///
    /// See [`Polygon::new`].
    pub fn convex_hull(points: &[Vec2]) -> Result<Self, PolygonError> {
        Polygon::new(points).map(Self::new)
    }

    /// Sets the density.
    #[must_use]
    pub fn density(mut self, density: f32) -> Self {
        self.density = density;
        self
    }

    /// Sets the friction coefficient.
    #[must_use]
    pub fn friction(mut self, friction: f32) -> Self {
        self.friction = friction;
        self
    }

    /// Sets the restitution.
    #[must_use]
    pub fn restitution(mut self, restitution: f32) -> Self {
        self.restitution = restitution;
        self
    }
}

/// A shape attached to a body.
#[derive(Debug, Clone)]
pub struct Collider {
    body: BodyHandle,
    shape: Shape,
    density: f32,
    friction: f32,
    restitution: f32,
    aabb: Aabb,
}

impl Collider {
    pub(crate) fn new(body: BodyHandle, desc: ColliderDesc, body_transform: &Transform) -> Self {
        let aabb = desc.shape.compute_aabb(body_transform);
        Self {
            body,
            shape: desc.shape,
            density: desc.density,
            friction: desc.friction,
            restitution: desc.restitution,
            aabb,
        }
    }

    /// Body this collider is attached to.
    #[must_use]
    pub fn body(&self) -> BodyHandle {
        self.body
    }

    /// Geometry, in the local frame of the body.
    #[must_use]
    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    /// Mass per square meter, in kg/m².
    #[must_use]
    pub fn density(&self) -> f32 {
        self.density
    }

    /// Friction coefficient.
    #[must_use]
    pub fn friction(&self) -> f32 {
        self.friction
    }

    /// Bounciness.
    #[must_use]
    pub fn restitution(&self) -> f32 {
        self.restitution
    }

    /// World-space bounding box, as of the last step.
    #[must_use]
    pub fn aabb(&self) -> Aabb {
        self.aabb
    }

    /// Sets the friction coefficient.
    pub fn set_friction(&mut self, friction: f32) {
        self.friction = friction;
    }

    /// Sets the restitution.
    pub fn set_restitution(&mut self, restitution: f32) {
        self.restitution = restitution;
    }

    pub(crate) fn mass_properties(&self) -> MassProperties {
        self.shape.mass_properties(self.density)
    }

    pub(crate) fn update_aabb(&mut self, body_transform: &Transform) {
        self.aabb = self.shape.compute_aabb(body_transform);
    }
}
