//! Rigid bodies: the moving (or not) things of the simulation.

use crate::arena::{Arena, Index};
use crate::collider::{Collider, ColliderHandle};
use crate::constants::{SLEEP_ANGULAR_SPEED, SLEEP_LINEAR_SPEED};
use crate::joint::JointHandle;
use crate::math::{Rot, Transform, Vec2};

/// Handle to a [`Body`] stored in a [`World`](crate::World).
///
/// A handle stays valid until its body is removed; afterwards, lookups with it return `None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BodyHandle(pub(crate) Index);

/// How a body reacts to forces and contacts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BodyType {
    /// Never moves. Used for the ground and walls.
    Fixed,
    /// Moves with the velocity you give it, ignoring forces and contacts. Used for moving
    /// platforms and doors.
    Kinematic,
    /// Fully simulated: affected by gravity, forces and contacts.
    #[default]
    Dynamic,
}

/// Description of a body to create, built with chained setters.
///
/// ```
/// use kinetica::{BodyDesc, vec2};
///
/// let desc = BodyDesc::dynamic().position(vec2(0.0, 4.0)).angle(0.3);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyDesc {
    /// How the body reacts to forces and contacts.
    pub body_type: BodyType,
    /// World position of the body origin.
    pub position: Vec2,
    /// Initial orientation.
    pub rotation: Rot,
    /// Initial velocity of the center of mass, in m/s.
    pub linear_velocity: Vec2,
    /// Initial angular velocity, in rad/s (counter-clockwise).
    pub angular_velocity: f32,
    /// Slows down linear motion over time; `0` means no damping.
    pub linear_damping: f32,
    /// Slows down rotation over time; `0` means no damping.
    pub angular_damping: f32,
    /// Multiplier applied to the world gravity for this body.
    pub gravity_scale: f32,
    /// Prevents the body from rotating, for example for a character.
    pub fixed_rotation: bool,
    /// Lets the body fall asleep after resting for a while, see [`Body::is_sleeping`].
    pub can_sleep: bool,
    /// Continuous collision against other dynamic bodies too, see [`BodyDesc::bullet`].
    pub bullet: bool,
}

impl BodyDesc {
    /// Describes a body of the given type at the origin, at rest.
    #[must_use]
    pub fn new(body_type: BodyType) -> Self {
        Self {
            body_type,
            position: Vec2::ZERO,
            rotation: Rot::IDENTITY,
            linear_velocity: Vec2::ZERO,
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            fixed_rotation: false,
            can_sleep: true,
            bullet: false,
        }
    }

    /// Describes a [`BodyType::Dynamic`] body.
    #[must_use]
    pub fn dynamic() -> Self {
        Self::new(BodyType::Dynamic)
    }

    /// Describes a [`BodyType::Kinematic`] body.
    #[must_use]
    pub fn kinematic() -> Self {
        Self::new(BodyType::Kinematic)
    }

    /// Describes a [`BodyType::Fixed`] body.
    #[must_use]
    pub fn fixed() -> Self {
        Self::new(BodyType::Fixed)
    }

    /// Sets the world position of the body origin.
    #[must_use]
    pub fn position(mut self, position: Vec2) -> Self {
        self.position = position;
        self
    }

    /// Sets the initial orientation, in radians.
    #[must_use]
    pub fn angle(mut self, angle: f32) -> Self {
        self.rotation = Rot::from_angle(angle);
        self
    }

    /// Sets the initial linear velocity.
    #[must_use]
    pub fn linear_velocity(mut self, velocity: Vec2) -> Self {
        self.linear_velocity = velocity;
        self
    }

    /// Sets the initial angular velocity.
    #[must_use]
    pub fn angular_velocity(mut self, velocity: f32) -> Self {
        self.angular_velocity = velocity;
        self
    }

    /// Sets the linear damping.
    #[must_use]
    pub fn linear_damping(mut self, damping: f32) -> Self {
        self.linear_damping = damping;
        self
    }

    /// Sets the angular damping.
    #[must_use]
    pub fn angular_damping(mut self, damping: f32) -> Self {
        self.angular_damping = damping;
        self
    }

    /// Sets the gravity multiplier.
    #[must_use]
    pub fn gravity_scale(mut self, scale: f32) -> Self {
        self.gravity_scale = scale;
        self
    }

    /// Prevents or allows rotation.
    #[must_use]
    pub fn fixed_rotation(mut self, fixed: bool) -> Self {
        self.fixed_rotation = fixed;
        self
    }

    /// Allows or prevents falling asleep.
    #[must_use]
    pub fn can_sleep(mut self, can_sleep: bool) -> Self {
        self.can_sleep = can_sleep;
        self
    }

    /// Makes the body a bullet, or a regular body again.
    ///
    /// Fast dynamic bodies never tunnel through fixed and kinematic bodies. Bullets do not
    /// tunnel through other dynamic bodies either, at a higher cost: use it for small fast
    /// projectiles.
    #[must_use]
    pub fn bullet(mut self, bullet: bool) -> Self {
        self.bullet = bullet;
        self
    }
}

impl Default for BodyDesc {
    fn default() -> Self {
        Self::dynamic()
    }
}

/// A rigid body living in a [`World`](crate::World).
///
/// The body *origin* is the point given by [`position`](Self::position); shapes are placed
/// relative to it. The body rotates around its *center of mass*, computed from its colliders.
#[derive(Debug, Clone)]
#[allow(clippy::struct_excessive_bools, reason = "independent flags")]
pub struct Body {
    body_type: BodyType,
    /// Placement of the body origin.
    transform: Transform,
    /// World-space center of mass, kept in sync with `transform`.
    center: Vec2,
    /// Center of mass relative to the body origin, in the local frame.
    local_center: Vec2,
    /// World center of mass and rotation when the current step started; the contact solver
    /// measures how far the body moved since then.
    center_at_step_start: Vec2,
    rotation_at_step_start: Rot,
    linear_velocity: Vec2,
    angular_velocity: f32,
    force: Vec2,
    torque: f32,
    mass: f32,
    inverse_mass: f32,
    inertia: f32,
    inverse_inertia: f32,
    linear_damping: f32,
    angular_damping: f32,
    gravity_scale: f32,
    fixed_rotation: bool,
    can_sleep: bool,
    sleeping: bool,
    /// How long the body has been nearly still, in seconds.
    sleep_time: f32,
    bullet: bool,
    /// Thickness of the thinnest collider; moving more than half of it in a step risks
    /// tunneling.
    min_extent: f32,
    colliders: Vec<ColliderHandle>,
    joints: Vec<JointHandle>,
}

impl Body {
    pub(crate) fn new(desc: BodyDesc) -> Self {
        let transform = Transform::new(desc.position, desc.rotation);
        let is_fixed = desc.body_type == BodyType::Fixed;
        let mut body = Self {
            body_type: desc.body_type,
            transform,
            center: transform.position,
            local_center: Vec2::ZERO,
            center_at_step_start: transform.position,
            rotation_at_step_start: transform.rotation,
            linear_velocity: if is_fixed { Vec2::ZERO } else { desc.linear_velocity },
            angular_velocity: if is_fixed { 0.0 } else { desc.angular_velocity },
            force: Vec2::ZERO,
            torque: 0.0,
            mass: 0.0,
            inverse_mass: 0.0,
            inertia: 0.0,
            inverse_inertia: 0.0,
            linear_damping: desc.linear_damping,
            angular_damping: desc.angular_damping,
            gravity_scale: desc.gravity_scale,
            fixed_rotation: desc.fixed_rotation,
            can_sleep: desc.can_sleep,
            sleeping: false,
            sleep_time: 0.0,
            bullet: desc.bullet,
            min_extent: f32::INFINITY,
            colliders: Vec::new(),
            joints: Vec::new(),
        };
        body.update_mass(&Arena::default());
        body
    }

    /// How the body reacts to forces and contacts.
    #[must_use]
    pub fn body_type(&self) -> BodyType {
        self.body_type
    }

    /// World position of the body origin.
    #[must_use]
    pub fn position(&self) -> Vec2 {
        self.transform.position
    }

    /// Orientation of the body.
    #[must_use]
    pub fn rotation(&self) -> Rot {
        self.transform.rotation
    }

    /// Orientation of the body, in radians.
    #[must_use]
    pub fn angle(&self) -> f32 {
        self.transform.rotation.angle()
    }

    /// Placement of the body origin.
    #[must_use]
    pub fn transform(&self) -> &Transform {
        &self.transform
    }

    /// World position of the center of mass.
    #[must_use]
    pub fn center_of_mass(&self) -> Vec2 {
        self.center
    }

    /// Center of mass relative to the body origin, in the local frame.
    #[must_use]
    pub fn local_center_of_mass(&self) -> Vec2 {
        self.local_center
    }

    /// Velocity of the center of mass, in m/s.
    #[must_use]
    pub fn linear_velocity(&self) -> Vec2 {
        self.linear_velocity
    }

    /// Angular velocity, in rad/s (counter-clockwise).
    #[must_use]
    pub fn angular_velocity(&self) -> f32 {
        self.angular_velocity
    }

    /// Velocity of the body material at a world point.
    #[must_use]
    pub fn velocity_at_point(&self, world_point: Vec2) -> Vec2 {
        self.linear_velocity + self.angular_velocity * (world_point - self.center).perp()
    }

    /// Mass in kilograms; `0` for fixed and kinematic bodies.
    #[must_use]
    pub fn mass(&self) -> f32 {
        self.mass
    }

    /// Rotational inertia about the center of mass; `0` when the body cannot rotate.
    ///
    /// A dynamic body without colliders gets a mass of 1 kg but no rotational inertia, so it
    /// cannot rotate. Give it a collider if it must turn, for example when it is part of a
    /// mechanism that rotates.
    #[must_use]
    pub fn inertia(&self) -> f32 {
        self.inertia
    }

    /// Multiplier applied to the world gravity for this body.
    #[must_use]
    pub fn gravity_scale(&self) -> f32 {
        self.gravity_scale
    }

    /// Returns `true` if the body is asleep.
    ///
    /// Bodies resting for a while fall asleep, together with everything they touch or are
    /// joined to, and are skipped by the simulation until something wakes them: a contact with
    /// an awake body, a change made through one of the setters below, or the removal of
    /// something they touch. Fixed bodies never sleep.
    #[must_use]
    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }

    /// Returns `true` if the body may fall asleep.
    #[must_use]
    pub fn can_sleep(&self) -> bool {
        self.can_sleep
    }

    /// Returns `true` for a bullet, see [`BodyDesc::bullet`].
    #[must_use]
    pub fn is_bullet(&self) -> bool {
        self.bullet
    }

    /// Colliders attached to this body.
    #[must_use]
    pub fn colliders(&self) -> &[ColliderHandle] {
        &self.colliders
    }

    /// Joints connected to this body.
    #[must_use]
    pub fn joints(&self) -> &[JointHandle] {
        &self.joints
    }

    /// Converts a point from the body's local frame to world space.
    #[must_use]
    pub fn world_point(&self, local_point: Vec2) -> Vec2 {
        self.transform.transform_point(local_point)
    }

    /// Converts a world point to the body's local frame.
    #[must_use]
    pub fn local_point(&self, world_point: Vec2) -> Vec2 {
        self.transform.inverse_transform_point(world_point)
    }

    /// Converts a direction from the body's local frame to world space.
    #[must_use]
    pub fn world_vector(&self, local_vector: Vec2) -> Vec2 {
        self.transform.rotation.rotate(local_vector)
    }

    /// Converts a world direction to the body's local frame.
    #[must_use]
    pub fn local_vector(&self, world_vector: Vec2) -> Vec2 {
        self.transform.rotation.inverse_rotate(world_vector)
    }

    /// Teleports the body origin to `position`, keeping its orientation.
    pub fn set_position(&mut self, position: Vec2) {
        self.set_transform(Transform::new(position, self.transform.rotation));
    }

    /// Sets the orientation, in radians, rotating around the body origin.
    pub fn set_angle(&mut self, angle: f32) {
        self.set_transform(Transform::new(self.transform.position, Rot::from_angle(angle)));
    }

    /// Teleports the body to a new placement, and wakes it up.
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        self.center = transform.transform_point(self.local_center);
        self.wake_up();
    }

    /// Sets the linear velocity, and wakes the body up. Ignored for fixed bodies.
    pub fn set_linear_velocity(&mut self, velocity: Vec2) {
        if self.body_type != BodyType::Fixed {
            self.linear_velocity = velocity;
            self.wake_up();
        }
    }

    /// Sets the angular velocity, and wakes the body up. Ignored for fixed bodies and bodies
    /// with fixed rotation.
    pub fn set_angular_velocity(&mut self, velocity: f32) {
        if self.body_type != BodyType::Fixed && !self.fixed_rotation {
            self.angular_velocity = velocity;
            self.wake_up();
        }
    }

    /// Sets the gravity multiplier, and wakes the body up.
    pub fn set_gravity_scale(&mut self, scale: f32) {
        self.gravity_scale = scale;
        self.wake_up();
    }

    /// Allows or prevents falling asleep. Preventing it wakes the body up.
    pub fn set_can_sleep(&mut self, can_sleep: bool) {
        self.can_sleep = can_sleep;
        if !can_sleep {
            self.wake_up();
        }
    }

    /// Makes the body a bullet, or a regular body again; see [`BodyDesc::bullet`].
    pub fn set_bullet(&mut self, bullet: bool) {
        self.bullet = bullet;
    }

    /// Wakes the body up. Bodies it touches or is joined to wake up at the next step.
    pub fn wake_up(&mut self) {
        self.sleeping = false;
        self.sleep_time = 0.0;
    }

    /// Applies a force at the center of mass until the next step, and wakes the body up. Only
    /// affects dynamic bodies.
    pub fn apply_force(&mut self, force: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.force += force;
            self.wake_up();
        }
    }

    /// Applies a force at a world point until the next step, which may also make the body spin.
    pub fn apply_force_at_point(&mut self, force: Vec2, world_point: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.force += force;
            self.torque += (world_point - self.center).perp_dot(force);
            self.wake_up();
        }
    }

    /// Applies a torque until the next step.
    pub fn apply_torque(&mut self, torque: f32) {
        if self.body_type == BodyType::Dynamic {
            self.torque += torque;
            self.wake_up();
        }
    }

    /// Instantly changes the velocity by `impulse / mass`, as if hit at the center of mass.
    pub fn apply_linear_impulse(&mut self, impulse: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.linear_velocity += self.inverse_mass * impulse;
            self.wake_up();
        }
    }

    /// Instantly changes the velocity as if hit at a world point.
    pub fn apply_linear_impulse_at_point(&mut self, impulse: Vec2, world_point: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.linear_velocity += self.inverse_mass * impulse;
            self.angular_velocity += self.inverse_inertia * (world_point - self.center).perp_dot(impulse);
            self.wake_up();
        }
    }

    /// Instantly changes the angular velocity by `impulse / inertia`.
    pub fn apply_angular_impulse(&mut self, impulse: f32) {
        if self.body_type == BodyType::Dynamic {
            self.angular_velocity += self.inverse_inertia * impulse;
            self.wake_up();
        }
    }

    pub(crate) fn attach_collider(&mut self, handle: ColliderHandle) {
        self.colliders.push(handle);
    }

    pub(crate) fn detach_collider(&mut self, handle: ColliderHandle) {
        self.colliders.retain(|&attached| attached != handle);
    }

    pub(crate) fn attach_joint(&mut self, handle: JointHandle) {
        self.joints.push(handle);
    }

    pub(crate) fn detach_joint(&mut self, handle: JointHandle) {
        self.joints.retain(|&attached| attached != handle);
    }

    /// Recomputes mass, inertia, center of mass and thickness from the attached colliders.
    pub(crate) fn update_mass(&mut self, colliders: &Arena<Collider>) {
        self.min_extent = self
            .colliders
            .iter()
            .filter_map(|handle| colliders.get(handle.0))
            .filter(|collider| !collider.is_sensor())
            .map(|collider| collider.shape().min_extent())
            .fold(f32::INFINITY, f32::min);

        self.mass = 0.0;
        self.inverse_mass = 0.0;
        self.inertia = 0.0;
        self.inverse_inertia = 0.0;
        self.local_center = Vec2::ZERO;

        if self.body_type == BodyType::Dynamic {
            let parts: Vec<_> = self
                .colliders
                .iter()
                .filter_map(|handle| colliders.get(handle.0))
                .map(Collider::mass_properties)
                .collect();

            let mass: f32 = parts.iter().map(|part| part.mass).sum();
            if mass > 0.0 {
                self.local_center = parts.iter().map(|part| part.mass * part.center).sum::<Vec2>() / mass;
                // Parallel axis theorem: each part's inertia moved to the shared center of mass.
                self.inertia = parts
                    .iter()
                    .map(|part| part.inertia + part.mass * part.center.distance_squared(self.local_center))
                    .sum();
                self.mass = mass;
            } else {
                // A dynamic body always needs some mass to respond to forces.
                self.mass = 1.0;
            }
            self.inverse_mass = 1.0 / self.mass;

            if self.fixed_rotation {
                self.inertia = 0.0;
                self.angular_velocity = 0.0;
            } else if self.inertia > 0.0 {
                self.inverse_inertia = 1.0 / self.inertia;
            }
        }

        self.center = self.transform.transform_point(self.local_center);
    }

    /// Fixed bodies and sleeping bodies are left alone by the simulation.
    pub(crate) fn is_active(&self) -> bool {
        self.body_type != BodyType::Fixed && !self.sleeping
    }

    /// Stops the body until it is woken up.
    pub(crate) fn fall_asleep(&mut self) {
        if self.body_type != BodyType::Fixed {
            self.sleeping = true;
            self.linear_velocity = Vec2::ZERO;
            self.angular_velocity = 0.0;
            self.clear_forces();
        }
    }

    /// Adds `dt` to the time the body has been nearly still, or resets it if the body moves,
    /// and returns it. A body that cannot sleep never accumulates time.
    pub(crate) fn update_sleep_time(&mut self, dt: f32) -> f32 {
        let still = if self.body_type == BodyType::Kinematic {
            // Its velocity is chosen by the user: however slow, it must keep it.
            self.linear_velocity == Vec2::ZERO && self.angular_velocity == 0.0
        } else {
            self.linear_velocity.length_squared() <= SLEEP_LINEAR_SPEED * SLEEP_LINEAR_SPEED
                && self.angular_velocity.abs() <= SLEEP_ANGULAR_SPEED
        };
        self.sleep_time = if self.can_sleep && still { self.sleep_time + dt } else { 0.0 };
        self.sleep_time
    }

    pub(crate) fn min_extent(&self) -> f32 {
        self.min_extent
    }

    /// World center of mass when the current step started.
    pub(crate) fn center_at_step_start(&self) -> Vec2 {
        self.center_at_step_start
    }

    /// Moves the center of mass to `center`, keeping the orientation and velocities.
    pub(crate) fn move_center_to(&mut self, center: Vec2) {
        self.center = center;
        self.transform.position = center - self.transform.rotation.rotate(self.local_center);
    }

    pub(crate) fn inverse_mass(&self) -> f32 {
        self.inverse_mass
    }

    pub(crate) fn inverse_inertia(&self) -> f32 {
        self.inverse_inertia
    }

    /// Changes the velocities as if `impulse` were applied at `anchor`, a point relative to the
    /// center of mass. Has no effect on fixed and kinematic bodies, whose inverse mass is zero.
    pub(crate) fn apply_solver_impulse(&mut self, impulse: Vec2, anchor: Vec2) {
        self.linear_velocity += self.inverse_mass * impulse;
        self.angular_velocity += self.inverse_inertia * anchor.perp_dot(impulse);
    }

    /// Changes the angular velocity as if an angular impulse were applied.
    pub(crate) fn apply_solver_angular_impulse(&mut self, impulse: f32) {
        self.angular_velocity += self.inverse_inertia * impulse;
    }

    /// Records the placement at the start of a step, see [`Self::delta_position`].
    pub(crate) fn begin_step(&mut self) {
        self.center_at_step_start = self.center;
        self.rotation_at_step_start = self.transform.rotation;
    }

    /// Displacement of the center of mass since [`Self::begin_step`].
    pub(crate) fn delta_position(&self) -> Vec2 {
        self.center - self.center_at_step_start
    }

    /// Rotation since [`Self::begin_step`].
    pub(crate) fn delta_rotation(&self) -> Rot {
        self.transform.rotation * self.rotation_at_step_start.inverse()
    }

    /// Applies gravity, forces and damping to the velocities.
    pub(crate) fn integrate_velocity(&mut self, gravity: Vec2, dt: f32) {
        if self.body_type != BodyType::Dynamic || self.sleeping {
            return;
        }
        let linear_acceleration = self.gravity_scale * gravity + self.inverse_mass * self.force;
        let angular_acceleration = self.inverse_inertia * self.torque;
        self.linear_velocity += dt * linear_acceleration;
        self.angular_velocity += dt * angular_acceleration;

        // Implicit damping: unconditionally stable, even with a large damping or time step.
        self.linear_velocity /= 1.0 + dt * self.linear_damping;
        self.angular_velocity /= 1.0 + dt * self.angular_damping;
    }

    /// Moves the body according to its velocities.
    pub(crate) fn integrate_position(&mut self, dt: f32) {
        if !self.is_active() {
            return;
        }
        self.center += dt * self.linear_velocity;
        self.transform.rotation = self.transform.rotation.integrate(dt * self.angular_velocity);
        self.transform.position = self.center - self.transform.rotation.rotate(self.local_center);
    }

    pub(crate) fn clear_forces(&mut self) {
        self.force = Vec2::ZERO;
        self.torque = 0.0;
    }
}
