//! Rigid bodies: the moving (or not) things of the simulation.

use crate::arena::{Arena, Index};
use crate::collider::{Collider, ColliderHandle};
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
    colliders: Vec<ColliderHandle>,
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
            colliders: Vec::new(),
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
    #[must_use]
    pub fn inertia(&self) -> f32 {
        self.inertia
    }

    /// Multiplier applied to the world gravity for this body.
    #[must_use]
    pub fn gravity_scale(&self) -> f32 {
        self.gravity_scale
    }

    /// Colliders attached to this body.
    #[must_use]
    pub fn colliders(&self) -> &[ColliderHandle] {
        &self.colliders
    }

    /// Teleports the body origin to `position`, keeping its orientation.
    pub fn set_position(&mut self, position: Vec2) {
        self.set_transform(Transform::new(position, self.transform.rotation));
    }

    /// Sets the orientation, in radians, rotating around the body origin.
    pub fn set_angle(&mut self, angle: f32) {
        self.set_transform(Transform::new(self.transform.position, Rot::from_angle(angle)));
    }

    /// Teleports the body to a new placement.
    pub fn set_transform(&mut self, transform: Transform) {
        self.transform = transform;
        self.center = transform.transform_point(self.local_center);
    }

    /// Sets the linear velocity. Ignored for fixed bodies.
    pub fn set_linear_velocity(&mut self, velocity: Vec2) {
        if self.body_type != BodyType::Fixed {
            self.linear_velocity = velocity;
        }
    }

    /// Sets the angular velocity. Ignored for fixed bodies and bodies with fixed rotation.
    pub fn set_angular_velocity(&mut self, velocity: f32) {
        if self.body_type != BodyType::Fixed && !self.fixed_rotation {
            self.angular_velocity = velocity;
        }
    }

    /// Sets the gravity multiplier.
    pub fn set_gravity_scale(&mut self, scale: f32) {
        self.gravity_scale = scale;
    }

    /// Applies a force at the center of mass until the next step. Only affects dynamic bodies.
    pub fn apply_force(&mut self, force: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.force += force;
        }
    }

    /// Applies a force at a world point until the next step, which may also make the body spin.
    pub fn apply_force_at_point(&mut self, force: Vec2, world_point: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.force += force;
            self.torque += (world_point - self.center).perp_dot(force);
        }
    }

    /// Applies a torque until the next step.
    pub fn apply_torque(&mut self, torque: f32) {
        if self.body_type == BodyType::Dynamic {
            self.torque += torque;
        }
    }

    /// Instantly changes the velocity by `impulse / mass`, as if hit at the center of mass.
    pub fn apply_linear_impulse(&mut self, impulse: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.linear_velocity += self.inverse_mass * impulse;
        }
    }

    /// Instantly changes the velocity as if hit at a world point.
    pub fn apply_linear_impulse_at_point(&mut self, impulse: Vec2, world_point: Vec2) {
        if self.body_type == BodyType::Dynamic {
            self.linear_velocity += self.inverse_mass * impulse;
            self.angular_velocity += self.inverse_inertia * (world_point - self.center).perp_dot(impulse);
        }
    }

    /// Instantly changes the angular velocity by `impulse / inertia`.
    pub fn apply_angular_impulse(&mut self, impulse: f32) {
        if self.body_type == BodyType::Dynamic {
            self.angular_velocity += self.inverse_inertia * impulse;
        }
    }

    pub(crate) fn attach_collider(&mut self, handle: ColliderHandle) {
        self.colliders.push(handle);
    }

    pub(crate) fn detach_collider(&mut self, handle: ColliderHandle) {
        self.colliders.retain(|&attached| attached != handle);
    }

    /// Recomputes mass, inertia and center of mass from the attached colliders.
    pub(crate) fn update_mass(&mut self, colliders: &Arena<Collider>) {
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
        if self.body_type != BodyType::Dynamic {
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
        if self.body_type == BodyType::Fixed {
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
