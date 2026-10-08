//! The simulation world: owns every body and collider and advances time.

use crate::arena::Arena;
use crate::body::{Body, BodyDesc, BodyHandle};
use crate::collider::{Collider, ColliderDesc, ColliderHandle};
use crate::math::{Vec2, vec2};

/// Earth gravity, pointing down the Y axis.
pub const DEFAULT_GRAVITY: Vec2 = vec2(0.0, -9.81);

/// A physics world.
///
/// Units are meters, kilograms and seconds, and the Y axis points up. Bodies and colliders are
/// owned by the world and accessed through handles.
///
/// Cloning a world copies its whole state, which can be used to save and restore it.
///
/// ```
/// use kinetica::{BodyDesc, ColliderDesc, World, vec2};
///
/// let mut world = World::new();
/// let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 10.0)));
/// world.add_collider(ball, ColliderDesc::circle(0.5));
///
/// for _ in 0..60 {
///     world.step(1.0 / 60.0);
/// }
/// assert!(world.body(ball).unwrap().position().y < 10.0);
/// ```
#[derive(Debug, Clone)]
pub struct World {
    gravity: Vec2,
    bodies: Arena<Body>,
    colliders: Arena<Collider>,
}

impl World {
    /// Creates an empty world with [`DEFAULT_GRAVITY`].
    #[must_use]
    pub fn new() -> Self {
        Self { gravity: DEFAULT_GRAVITY, bodies: Arena::default(), colliders: Arena::default() }
    }

    /// Gravity acceleration applied to dynamic bodies, in m/s².
    #[must_use]
    pub fn gravity(&self) -> Vec2 {
        self.gravity
    }

    /// Sets the gravity acceleration.
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.gravity = gravity;
    }

    /// Creates a body and returns its handle.
    pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle {
        BodyHandle(self.bodies.insert(Body::new(desc)))
    }

    /// Removes a body and all its colliders, returning the body if it existed.
    pub fn remove_body(&mut self, handle: BodyHandle) -> Option<Body> {
        let body = self.bodies.remove(handle.0)?;
        for collider in body.colliders() {
            self.colliders.remove(collider.0);
        }
        Some(body)
    }

    /// Returns the body, or `None` if it was removed.
    #[must_use]
    pub fn body(&self, handle: BodyHandle) -> Option<&Body> {
        self.bodies.get(handle.0)
    }

    /// Returns the body for modification, or `None` if it was removed.
    pub fn body_mut(&mut self, handle: BodyHandle) -> Option<&mut Body> {
        self.bodies.get_mut(handle.0)
    }

    /// Iterates over all bodies, in a deterministic order.
    pub fn bodies(&self) -> impl Iterator<Item = (BodyHandle, &Body)> {
        self.bodies.iter().map(|(index, body)| (BodyHandle(index), body))
    }

    /// Number of bodies.
    #[must_use]
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Attaches a new collider to a body and updates the body mass.
    ///
    /// # Panics
    ///
    /// Panics if the body was removed.
    pub fn add_collider(&mut self, body: BodyHandle, desc: ColliderDesc) -> ColliderHandle {
        let owner = self.bodies.get_mut(body.0).expect("cannot add a collider to a removed body");
        let handle = ColliderHandle(self.colliders.insert(Collider::new(body, desc, owner.transform())));
        owner.attach_collider(handle);
        owner.update_mass(&self.colliders);
        handle
    }

    /// Removes a collider and updates the mass of its body, returning the collider if it existed.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Option<Collider> {
        let collider = self.colliders.remove(handle.0)?;
        if let Some(owner) = self.bodies.get_mut(collider.body().0) {
            owner.detach_collider(handle);
            owner.update_mass(&self.colliders);
        }
        Some(collider)
    }

    /// Returns the collider, or `None` if it was removed.
    #[must_use]
    pub fn collider(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle.0)
    }

    /// Returns the collider for modification, or `None` if it was removed.
    pub fn collider_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle.0)
    }

    /// Iterates over all colliders, in a deterministic order.
    pub fn colliders(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter().map(|(index, collider)| (ColliderHandle(index), collider))
    }

    /// Number of colliders.
    #[must_use]
    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    /// Advances the simulation by `dt` seconds.
    ///
    /// Use a fixed `dt` (typically `1.0 / 60.0`) for stable and reproducible results. A
    /// non-positive or non-finite `dt` does nothing.
    pub fn step(&mut self, dt: f32) {
        if !(dt.is_finite() && dt > 0.0) {
            return;
        }

        let gravity = self.gravity;
        for body in self.bodies.values_mut() {
            body.integrate_velocity(gravity, dt);
        }
        for body in self.bodies.values_mut() {
            body.integrate_position(dt);
        }
        self.update_collider_aabbs();
    }

    fn update_collider_aabbs(&mut self) {
        for collider in self.colliders.values_mut() {
            if let Some(body) = self.bodies.get(collider.body().0) {
                collider.update_aabb(body.transform());
            }
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}
