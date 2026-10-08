//! Colliders: shapes attached to bodies, with their surface material.

use crate::arena::Index;
use crate::body::BodyHandle;
use crate::collision::broad_phase::ProxyId;
use crate::math::{Aabb, Rot, Transform, Vec2};
use crate::shape::{Circle, MassProperties, Polygon, PolygonError, Shape};

/// Handle to a [`Collider`] stored in a [`World`](crate::World).
///
/// A handle stays valid until its collider (or the body owning it) is removed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ColliderHandle(pub(crate) Index);

/// Which colliders can touch each other, as two 32-bit masks.
///
/// Two colliders interact when each one belongs to a layer the other collides with:
/// `a.memberships & b.filter != 0` and `b.memberships & a.filter != 0`. By default a collider
/// belongs to every layer and collides with every layer.
///
/// ```
/// use kinetica::{ColliderDesc, CollisionLayers};
///
/// const PLAYER: u32 = 1 << 0;
/// const ENEMY: u32 = 1 << 1;
/// const WORLD: u32 = 1 << 2;
///
/// // Players go through each other but hit enemies and the world.
/// let player = ColliderDesc::circle(0.4).layers(CollisionLayers::new(PLAYER, ENEMY | WORLD));
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CollisionLayers {
    /// Layers this collider belongs to.
    pub memberships: u32,
    /// Layers this collider collides with.
    pub filter: u32,
}

impl CollisionLayers {
    /// Belongs to every layer and collides with every layer.
    pub const ALL: Self = Self::new(u32::MAX, u32::MAX);
    /// Belongs to no layer and collides with nothing.
    pub const NONE: Self = Self::new(0, 0);

    /// Creates layers from the two masks.
    #[must_use]
    pub const fn new(memberships: u32, filter: u32) -> Self {
        Self { memberships, filter }
    }

    /// Returns `true` if colliders with these layers and `other` can touch.
    #[must_use]
    pub const fn interacts_with(&self, other: &Self) -> bool {
        self.memberships & other.filter != 0 && other.memberships & self.filter != 0
    }
}

impl Default for CollisionLayers {
    fn default() -> Self {
        Self::ALL
    }
}

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
    /// Which colliders this one can touch.
    pub layers: CollisionLayers,
    /// A sensor detects overlapping colliders without pushing them, see
    /// [`World::sensor_events`](crate::World::sensor_events). It has no mass.
    pub sensor: bool,
    /// Makes the collider a one-way platform: it only stops things coming from this side,
    /// given as a direction in the body's local frame. See [`ColliderDesc::one_way`].
    pub one_way: Option<Vec2>,
}

impl ColliderDesc {
    /// Describes a collider with the given shape and default material.
    #[must_use]
    pub fn new(shape: impl Into<Shape>) -> Self {
        Self {
            shape: shape.into(),
            density: 1.0,
            friction: 0.6,
            restitution: 0.0,
            layers: CollisionLayers::ALL,
            sensor: false,
            one_way: None,
        }
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

    /// Describes a rectangle placed at `center` and rotated by `angle`, relative to the body origin.
    ///
    /// # Panics
    ///
    /// Panics if a half extent is not strictly positive and finite.
    #[must_use]
    pub fn offset_rectangle(half_width: f32, half_height: f32, center: Vec2, angle: f32) -> Self {
        Self::new(Polygon::offset_rectangle(half_width, half_height, center, angle))
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

    /// Sets which colliders this one can touch.
    #[must_use]
    pub fn layers(mut self, layers: CollisionLayers) -> Self {
        self.layers = layers;
        self
    }

    /// Makes the collider a sensor, or a solid collider again.
    #[must_use]
    pub fn sensor(mut self, sensor: bool) -> Self {
        self.sensor = sensor;
        self
    }

    /// Makes the collider a one-way platform that only stops colliders coming from the
    /// `direction` side, given in the body's local frame (usually [`Vec2::Y`]).
    ///
    /// Whether a contact blocks is decided when it starts: it blocks if the other collider
    /// arrives within 45° of `direction`, and keeps that decision until the colliders separate.
    /// A body jumping through from below is therefore not caught halfway.
    ///
    /// # Panics
    ///
    /// Panics if `direction` is zero or not finite.
    #[must_use]
    pub fn one_way(mut self, direction: Vec2) -> Self {
        let direction = direction.try_normalize().expect("a one-way direction must be non-zero and finite");
        self.one_way = Some(direction);
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
    layers: CollisionLayers,
    sensor: bool,
    one_way: Option<Vec2>,
    aabb: Aabb,
    proxy: ProxyId,
}

impl Collider {
    pub(crate) fn new(body: BodyHandle, desc: ColliderDesc, aabb: Aabb, proxy: ProxyId) -> Self {
        Self {
            body,
            shape: desc.shape,
            density: desc.density,
            friction: desc.friction,
            restitution: desc.restitution,
            layers: desc.layers,
            sensor: desc.sensor,
            one_way: desc.one_way,
            aabb,
            proxy,
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

    /// Which colliders this one can touch.
    #[must_use]
    pub fn layers(&self) -> CollisionLayers {
        self.layers
    }

    /// Returns `true` for a sensor, which detects overlaps without pushing.
    #[must_use]
    pub fn is_sensor(&self) -> bool {
        self.sensor
    }

    /// For a one-way platform, the side it blocks colliders from, in the body's local frame.
    #[must_use]
    pub fn one_way(&self) -> Option<Vec2> {
        self.one_way
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

    /// Changes which colliders this one can touch, from the next step.
    ///
    /// Contacts between sleeping bodies are not updated: wake the body up with
    /// [`Body::wake_up`](crate::Body::wake_up) if it may be asleep.
    pub fn set_layers(&mut self, layers: CollisionLayers) {
        self.layers = layers;
    }

    /// Sensors have no mass, so adding one never changes how a body moves.
    pub(crate) fn mass_properties(&self) -> MassProperties {
        if self.sensor { MassProperties::default() } else { self.shape.mass_properties(self.density) }
    }

    /// Returns `false` if this one-way platform must let through a collider touching it along
    /// `normal`, the contact normal pointing from the platform to the other collider.
    pub(crate) fn blocks_along(&self, rotation: Rot, normal: Vec2) -> bool {
        /// cos 45°: contacts more tilted than this from the blocking direction pass through.
        const MIN_ALIGNMENT: f32 = std::f32::consts::FRAC_1_SQRT_2;
        self.one_way.is_none_or(|direction| rotation.rotate(direction).dot(normal) >= MIN_ALIGNMENT)
    }

    pub(crate) fn proxy(&self) -> ProxyId {
        self.proxy
    }

    pub(crate) fn update_aabb(&mut self, body_transform: &Transform) {
        self.aabb = self.shape.compute_aabb(body_transform);
    }
}
