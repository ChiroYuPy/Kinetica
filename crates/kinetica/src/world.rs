//! The simulation world: owns every body and collider and advances time.

use crate::arena::Arena;
use crate::body::{Body, BodyDesc, BodyHandle, BodyType};
use crate::collider::{ChainDesc, Collider, ColliderDesc, ColliderHandle};
use crate::collision::broad_phase::BroadPhase;
use crate::constants::{DEFAULT_SUBSTEP_COUNT, TIME_TO_SLEEP};
use crate::contact::{CollisionEvent, Contact, ContactSet, SensorEvent, SensorPair};
use crate::continuous;
use crate::island::Islands;
use crate::joint::{Joint, JointDesc, JointHandle};
use crate::math::{Vec2, vec2};
use crate::profile::{StepProfile, Timer};
use crate::shape::{ChainSegment, Segment};
use crate::solver::{self, StepSettings};

/// Earth gravity, pointing down the Y axis.
pub const DEFAULT_GRAVITY: Vec2 = vec2(0.0, -9.81);

/// A physics world.
///
/// Units are meters, kilograms and seconds, and the Y axis points up. Bodies and colliders are
/// owned by the world and accessed through handles.
///
/// Cloning a world copies its whole state, which can be used to save and restore it. With the
/// `serde` feature, a world can also be serialized, for save files or replays: a deserialized
/// world continues exactly like the original. Only deserialize data that was serialized by
/// Kinetica, since it is trusted to be consistent.
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct World {
    gravity: Vec2,
    substep_count: u32,
    sleeping_enabled: bool,
    continuous_enabled: bool,
    bodies: Arena<Body>,
    colliders: Arena<Collider>,
    joints: Arena<Joint>,
    broad_phase: BroadPhase,
    contacts: ContactSet,
    collision_events: Vec<CollisionEvent>,
    sensor_events: Vec<SensorEvent>,
    #[cfg_attr(feature = "serde", serde(skip))]
    profile: StepProfile,
}

impl World {
    /// Creates an empty world with [`DEFAULT_GRAVITY`].
    #[must_use]
    pub fn new() -> Self {
        Self {
            gravity: DEFAULT_GRAVITY,
            substep_count: DEFAULT_SUBSTEP_COUNT,
            sleeping_enabled: true,
            continuous_enabled: true,
            bodies: Arena::default(),
            colliders: Arena::default(),
            joints: Arena::default(),
            broad_phase: BroadPhase::default(),
            contacts: ContactSet::default(),
            collision_events: Vec::new(),
            sensor_events: Vec::new(),
            profile: StepProfile::default(),
        }
    }

    /// Gravity acceleration applied to dynamic bodies, in m/s².
    #[must_use]
    pub fn gravity(&self) -> Vec2 {
        self.gravity
    }

    /// Sets the gravity acceleration, and wakes every body up.
    pub fn set_gravity(&mut self, gravity: Vec2) {
        self.gravity = gravity;
        self.wake_up_all();
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

    /// Returns `true` if resting bodies may fall asleep, see [`Body::is_sleeping`]. Enabled by
    /// default.
    #[must_use]
    pub fn sleeping_enabled(&self) -> bool {
        self.sleeping_enabled
    }

    /// Allows or prevents bodies from falling asleep. Disabling it wakes every body up.
    ///
    /// Sleeping saves most of the cost of resting bodies, and removes the tiny residual motion
    /// of large stacks.
    pub fn set_sleeping_enabled(&mut self, enabled: bool) {
        self.sleeping_enabled = enabled;
        if !enabled {
            self.wake_up_all();
        }
    }

    /// Returns `true` if fast bodies are kept from tunneling through others. Enabled by default.
    #[must_use]
    pub fn continuous_enabled(&self) -> bool {
        self.continuous_enabled
    }

    /// Enables or disables continuous collision detection.
    ///
    /// When enabled, a dynamic body moving more than half its thickness in a step is swept
    /// along its motion and stopped at the first fixed or kinematic collider it hits (and at
    /// dynamic colliders too for [bullets](crate::BodyDesc::bullet)).
    pub fn set_continuous_enabled(&mut self, enabled: bool) {
        self.continuous_enabled = enabled;
    }

    /// Creates a body and returns its handle.
    pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle {
        BodyHandle(self.bodies.insert(Body::new(desc)))
    }

    /// Removes a body with all its colliders and joints, returning the body if it existed.
    ///
    /// Bodies it touched wake up.
    pub fn remove_body(&mut self, handle: BodyHandle) -> Option<Body> {
        self.wake_up_touching(handle);
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

    /// Attaches a new collider to a body, updates the body mass and wakes the body up.
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
        owner.wake_up();
        handle
    }

    /// Attaches a chain of segments to a body, one collider per segment, and returns them in
    /// order.
    ///
    /// Shapes slide smoothly across the joints between segments, which a row of separate
    /// boxes or segments does not guarantee.
    ///
    /// # Panics
    ///
    /// Panics if the body was removed, if the chain has fewer than two points (three when
    /// closed), or if two consecutive points are equal.
    pub fn add_chain(&mut self, body: BodyHandle, desc: &ChainDesc) -> Vec<ColliderHandle> {
        let points = &desc.points;
        let count = points.len();
        assert!(count >= if desc.closed { 3 } else { 2 }, "a chain needs more points");
        let segment_count = if desc.closed { count } else { count - 1 };
        let point = |index: usize| points[index % count];
        (0..segment_count)
            .map(|i| {
                let previous = (i > 0 || desc.closed).then(|| point(i + count - 1));
                let next = (i + 2 < count || desc.closed).then(|| point(i + 2));
                let segment = ChainSegment::new(Segment::new(point(i), point(i + 1)), previous, next);
                let collider = ColliderDesc::new(segment)
                    .friction(desc.friction)
                    .restitution(desc.restitution)
                    .layers(desc.layers);
                self.add_collider(body, collider)
            })
            .collect()
    }

    /// Removes a collider and updates the mass of its body, returning the collider if it existed.
    ///
    /// Its body, and the bodies it touched, wake up.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Option<Collider> {
        let body = self.colliders.get(handle.0)?.body();
        self.wake_up_touching(body);
        let collider = self.destroy_collider(handle)?;
        if let Some(owner) = self.bodies.get_mut(body.0) {
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

    /// Connects two bodies with a joint, and wakes them up.
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
        a.wake_up();
        b.wake_up();
        if !collide_connected {
            self.contacts.remove_between(body_a, body_b);
        }
        handle
    }

    /// Removes a joint, returning it if it existed.
    ///
    /// The bodies it connected wake up, and may collide again from the next step.
    pub fn remove_joint(&mut self, handle: JointHandle) -> Option<Joint> {
        let joint = self.joints.remove(handle.0)?;
        for body in [joint.body_a(), joint.body_b()] {
            if let Some(body) = self.bodies.get_mut(body.0) {
                body.detach_joint(handle);
                body.wake_up();
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

    /// Returns the joint for modification, or `None` if it was removed. Wakes up the bodies
    /// it connects, so they react to the change.
    pub fn joint_mut(&mut self, handle: JointHandle) -> Option<&mut Joint> {
        let joint = self.joints.get_mut(handle.0)?;
        for body in [joint.body_a(), joint.body_b()] {
            if let Some(body) = self.bodies.get_mut(body.0) {
                body.wake_up();
            }
        }
        Some(joint)
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

    /// Sensors overlaps that started or stopped during the last step, in a deterministic order.
    #[must_use]
    pub fn sensor_events(&self) -> &[SensorEvent] {
        &self.sensor_events
    }

    /// Colliders overlapping the sensor `sensor`, as of the last step.
    pub fn sensor_overlaps(&self, sensor: ColliderHandle) -> impl Iterator<Item = &SensorPair> {
        self.contacts.sensor_overlaps(sensor)
    }

    /// Wakes every body up.
    pub fn wake_up_all(&mut self) {
        for body in self.bodies.values_mut() {
            body.wake_up();
        }
    }

    /// Wakes up `body` and the bodies touching it or joined to it.
    fn wake_up_touching(&mut self, handle: BodyHandle) {
        let mut neighbours: Vec<BodyHandle> =
            self.contacts.touching().filter_map(|contact| contact.pair().other_body(handle)).collect();
        if let Some(body) = self.bodies.get(handle.0) {
            neighbours.extend(body.joints().iter().filter_map(|&joint| self.joints.get(joint.0)?.other_body(handle)));
        }
        neighbours.push(handle);
        for handle in neighbours {
            if let Some(body) = self.bodies.get_mut(handle.0) {
                body.wake_up();
            }
        }
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
        let mut timer = Timer::start();
        let mut profile = StepProfile::default();
        self.collision_events.clear();
        self.sensor_events.clear();

        // Contact geometry for the current positions.
        self.contacts.update(
            &self.colliders,
            &self.bodies,
            &self.broad_phase,
            &mut self.collision_events,
            &mut self.sensor_events,
        );
        profile.collide = timer.lap();

        // Bodies touched by awake ones wake up before solving, so they react at once.
        let islands = self.sleeping_enabled.then(|| Islands::build(&self.bodies, &self.contacts, &self.joints));
        if let Some(islands) = &islands {
            islands.wake_up(&mut self.bodies);
        }
        profile.islands = timer.lap();

        let settings = StepSettings { gravity: self.gravity, dt, substep_count: self.substep_count };
        solver::step(&mut self.bodies, &self.colliders, &mut self.contacts, &mut self.joints, &settings);
        for body in self.bodies.values_mut() {
            body.clear_forces();
        }
        profile.solve = timer.lap();

        self.synchronize_colliders();
        profile.synchronize = timer.lap();
        if self.continuous_enabled {
            let moved = continuous::solve(&mut self.bodies, &self.colliders, &self.joints, &self.broad_phase);
            for body in moved {
                self.synchronize_body_colliders(body);
            }
        }
        profile.continuous = timer.lap();

        if let Some(islands) = &islands {
            islands.fall_asleep(&mut self.bodies, dt);
            self.put_kinematic_bodies_to_sleep(dt);
        }
        profile.sleep = timer.lap();

        // Find the pairs the motion brought together; they are collided at the next step.
        let new_pairs = self.broad_phase.find_new_pairs();
        self.contacts.add_pairs(&new_pairs, &self.colliders, &self.bodies, &self.joints);
        profile.find_pairs = timer.lap();
        self.profile = profile;
    }

    /// How long each stage of the last step took. Only measured with the `profile` feature;
    /// zero otherwise.
    #[must_use]
    pub fn profile(&self) -> &StepProfile {
        &self.profile
    }

    /// Moves collider bounding boxes and broad phase proxies to their bodies' new placement.
    fn synchronize_colliders(&mut self) {
        for collider in self.colliders.values_mut() {
            if let Some(body) = self.bodies.get(collider.body().0)
                && !body.is_sleeping()
            {
                collider.update_aabb(body.transform());
                self.broad_phase.update_proxy(collider.proxy(), collider.aabb());
            }
        }
    }

    fn synchronize_body_colliders(&mut self, handle: BodyHandle) {
        let Some(body) = self.bodies.get(handle.0) else { return };
        for collider in body.colliders() {
            if let Some(collider) = self.colliders.get_mut(collider.0) {
                collider.update_aabb(body.transform());
                self.broad_phase.update_proxy(collider.proxy(), collider.aabb());
            }
        }
    }

    /// Kinematic bodies are not part of islands; each one sleeps on its own once still.
    fn put_kinematic_bodies_to_sleep(&mut self, dt: f32) {
        for body in self.bodies.values_mut() {
            if body.body_type() == BodyType::Kinematic
                && !body.is_sleeping()
                && body.update_sleep_time(dt) >= TIME_TO_SLEEP
            {
                body.fall_asleep();
            }
        }
    }
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}
