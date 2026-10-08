//! The simulation world: owns every body and collider and advances time.

use crate::arena::Arena;
use crate::body::{Body, BodyDesc, BodyHandle};
use crate::collider::{Collider, ColliderDesc, ColliderHandle};
use crate::collision::broad_phase::BroadPhase;
use crate::constants::DEFAULT_SUBSTEP_COUNT;
use crate::contact::{CollisionEvent, Contact, ContactSet};
use crate::joint::{Joint, JointDesc, JointHandle};
use crate::math::{Vec2, vec2};
use crate::solver::{self, StepSettings};

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
    substep_count: u32,
    bodies: Arena<Body>,
    colliders: Arena<Collider>,
    joints: Arena<Joint>,
    broad_phase: BroadPhase,
    contacts: ContactSet,
    collision_events: Vec<CollisionEvent>,
}

impl World {
    /// Creates an empty world with [`DEFAULT_GRAVITY`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            gravity: DEFAULT_GRAVITY,
            substep_count: DEFAULT_SUBSTEP_COUNT,
            bodies: Arena::default(),
            colliders: Arena::default(),
            joints: Arena::default(),
            broad_phase: BroadPhase::default(),
            contacts: ContactSet::default(),
            collision_events: Vec::new(),
        }
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

    /// Number of substeps each [`step`](Self::step) is divided into. Defaults to 4.
    #[must_use]
    pub fn substep_count(&self) -> u32 {
        self.substep_count
    }

    /// Sets the number of substeps. More substeps make stacks and fast bodies more accurate, at
    /// a proportional cost.
    ///
    /// # Panics
    ///
    /// Panics if `count` is zero.
    pub fn set_substep_count(&mut self, count: u32) {
        assert!(count > 0, "a step needs at least one substep");
        self.substep_count = count;
    }

    /// Creates a body and returns its handle.
    pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle {
        BodyHandle(self.bodies.insert(Body::new(desc)))
    }

    /// Removes a body with all its colliders and joints, returning the body if it existed.
    pub fn remove_body(&mut self, handle: BodyHandle) -> Option<Body> {
        let body = self.bodies.remove(handle.0)?;
        for &collider in body.colliders() {
            self.destroy_collider(collider);
        }
        for &joint in body.joints() {
            self.remove_joint(joint);
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
        let aabb = desc.shape.compute_aabb(owner.transform());
        let broad_phase = &mut self.broad_phase;
        let handle = ColliderHandle(self.colliders.insert_with(|index| {
            let proxy = broad_phase.create_proxy(aabb, ColliderHandle(index));
            Collider::new(body, desc, aabb, proxy)
        }));
        owner.attach_collider(handle);
        owner.update_mass(&self.colliders);
        handle
    }

    /// Removes a collider and updates the mass of its body, returning the collider if it existed.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Option<Collider> {
        let collider = self.destroy_collider(handle)?;
        if let Some(owner) = self.bodies.get_mut(collider.body().0) {
            owner.detach_collider(handle);
            owner.update_mass(&self.colliders);
        }
        Some(collider)
    }

    /// Removes a collider from storage, the broad phase and the contacts, but not from its body.
    fn destroy_collider(&mut self, handle: ColliderHandle) -> Option<Collider> {
        let collider = self.colliders.remove(handle.0)?;
        self.broad_phase.destroy_proxy(collider.proxy());
        self.contacts.remove_collider(handle);
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

    /// Connects two bodies with a joint.
    ///
    /// Unless the joint allows it, the two bodies stop colliding with each other.
    ///
    /// # Panics
    ///
    /// Panics if a body was removed, or if both bodies are the same.
    pub fn add_joint(&mut self, desc: impl Into<JointDesc>) -> JointHandle {
        let desc = desc.into();
        let (body_a, body_b) = desc.bodies();
        assert_ne!(body_a, body_b, "a joint needs two different bodies");
        let (a, b) = self.bodies.get2_mut(body_a.0, body_b.0).expect("cannot add a joint to a removed body");
        let joint = Joint::new(&desc, a, b);
        let collide_connected = joint.collide_connected();
        let handle = JointHandle(self.joints.insert(joint));
        a.attach_joint(handle);
        b.attach_joint(handle);
        if !collide_connected {
            self.contacts.remove_between(body_a, body_b);
        }
        handle
    }

    /// Removes a joint, returning it if it existed.
    ///
    /// The bodies it connected may collide again from the next step.
    pub fn remove_joint(&mut self, handle: JointHandle) -> Option<Joint> {
        let joint = self.joints.remove(handle.0)?;
        for body in [joint.body_a(), joint.body_b()] {
            if let Some(body) = self.bodies.get_mut(body.0) {
                body.detach_joint(handle);
            }
        }
        // Let the broad phase find the pair again.
        if let Some(body) = self.bodies.get(joint.body_a().0) {
            for collider in body.colliders() {
                if let Some(collider) = self.colliders.get(collider.0) {
                    self.broad_phase.touch_proxy(collider.proxy());
                }
            }
        }
        Some(joint)
    }

    /// Returns the joint, or `None` if it was removed.
    #[must_use]
    pub fn joint(&self, handle: JointHandle) -> Option<&Joint> {
        self.joints.get(handle.0)
    }

    /// Returns the joint for modification, or `None` if it was removed.
    pub fn joint_mut(&mut self, handle: JointHandle) -> Option<&mut Joint> {
        self.joints.get_mut(handle.0)
    }

    /// Iterates over all joints, in a deterministic order.
    pub fn joints(&self) -> impl Iterator<Item = (JointHandle, &Joint)> {
        self.joints.iter().map(|(index, joint)| (JointHandle(index), joint))
    }

    /// Number of joints.
    #[must_use]
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Contacts between touching colliders, as of the last step.
    ///
    /// Shapes count as touching slightly before they overlap: points with a small positive
    /// [`separation`](crate::collision::ManifoldPoint::separation) are included.
    pub fn contacts(&self) -> impl Iterator<Item = &Contact> {
        self.contacts.touching()
    }

    /// Contacts involving one of the colliders of `body`, as of the last step.
    pub fn body_contacts(&self, body: BodyHandle) -> impl Iterator<Item = &Contact> {
        self.contacts().filter(move |contact| contact.pair().involves_body(body))
    }

    /// Contacts that started or stopped during the last step, in a deterministic order.
    #[must_use]
    pub fn collision_events(&self) -> &[CollisionEvent] {
        &self.collision_events
    }

    pub(crate) fn broad_phase(&self) -> &BroadPhase {
        &self.broad_phase
    }

    /// Advances the simulation by `dt` seconds.
    ///
    /// Use a fixed `dt` (typically `1.0 / 60.0`) for stable and reproducible results. A
    /// non-positive or non-finite `dt` does nothing.
    pub fn step(&mut self, dt: f32) {
        if !(dt.is_finite() && dt > 0.0) {
            return;
        }
        self.collision_events.clear();

        // Contact geometry for the current positions.
        self.contacts.update(&self.colliders, &self.bodies, &self.broad_phase, &mut self.collision_events);

        let settings = StepSettings { gravity: self.gravity, dt, substep_count: self.substep_count };
        solver::step(&mut self.bodies, &self.colliders, &mut self.contacts, &mut self.joints, &settings);
        for body in self.bodies.values_mut() {
            body.clear_forces();
        }

        // Find the pairs the motion brought together; they are collided at the next step.
        self.synchronize_colliders();
        let new_pairs = self.broad_phase.find_new_pairs();
        self.contacts.add_pairs(&new_pairs, &self.colliders, &self.bodies, &self.joints);
    }

    /// Moves collider bounding boxes and broad phase proxies to their bodies' new placement.
    fn synchronize_colliders(&mut self) {
        for collider in self.colliders.values_mut() {
            if let Some(body) = self.bodies.get(collider.body().0) {
                collider.update_aabb(body.transform());
                self.broad_phase.update_proxy(collider.proxy(), collider.aabb());
            }
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}
