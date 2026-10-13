//! Velocity constraints keeping touching bodies apart, with friction and restitution.

use crate::arena::Arena;
use crate::body::Body;
use crate::collider::Collider;
use crate::collision::Manifold;
use crate::constants::{MAX_CONTACT_PUSH_SPEED, RESTITUTION_THRESHOLD};
use crate::contact::{Contact, ContactPair};
use crate::math::{Vec2, vec2};
use crate::solver::Softness;

use super::{Pass, SolverContext};

/// Everything the solver needs about one contact, prepared once per step.
#[derive(Debug, Clone)]
pub(crate) struct ContactConstraint {
    pair: ContactPair,
    normal: Vec2,
    friction: f32,
    restitution: f32,
    softness: Softness,
    points: [ConstraintPoint; 2],
    point_count: usize,
}

#[derive(Debug, Clone, Copy, Default)]
struct ConstraintPoint {
    /// Contact point relative to each body's center of mass, at the start of the step.
    anchor_a: Vec2,
    anchor_b: Vec2,
    /// Separation at the start of the step, minus the anchor offset along the normal, so the
    /// current separation can be updated cheaply from body displacements.
    base_separation: f32,
    /// Inverse of the effective mass along the normal and along the tangent.
    normal_mass: f32,
    tangent_mass: f32,
    /// Accumulated impulses, warm started from the previous step.
    normal_impulse: f32,
    tangent_impulse: f32,
    /// Largest normal impulse during the step; zero means the point never pushed.
    max_normal_impulse: f32,
    /// Fastest normal velocity seen before the point pushed, used for restitution: measured as
    /// late as possible, so the speed gained while falling during the step is not lost.
    approach_speed: f32,
}

impl ContactConstraint {
    pub(crate) fn new(
        contact: &Contact,
        colliders: &Arena<Collider>,
        bodies: &Arena<Body>,
        context: &SolverContext,
    ) -> Option<Self> {
        let pair = *contact.pair();
        let (collider_a, collider_b) = (colliders.get(pair.collider_a.0)?, colliders.get(pair.collider_b.0)?);
        let (body_a, body_b) = (bodies.get(pair.body_a.0)?, bodies.get(pair.body_b.0)?);
        let (mass_a, inertia_a) = (body_a.inverse_mass(), body_a.inverse_inertia());
        let (mass_b, inertia_b) = (body_b.inverse_mass(), body_b.inverse_inertia());

        let manifold = contact.manifold();
        let normal = manifold.normal();
        let tangent = tangent_of(normal);
        let mut constraint = Self {
            pair,
            normal,
            // Friction is dominated by the slipperier surface, bounce by the bouncier one.
            friction: (collider_a.friction() * collider_b.friction()).sqrt(),
            restitution: collider_a.restitution().max(collider_b.restitution()),
            // Contacts against an immovable body can be stiffer without becoming unstable.
            softness: if mass_a == 0.0 || mass_b == 0.0 {
                context.fixed_contact_softness
            } else {
                context.contact_softness
            },
            points: Default::default(),
            point_count: manifold.points().len(),
        };

        for (point, manifold_point) in constraint.points.iter_mut().zip(manifold.points()) {
            let anchor_a = manifold_point.position() - body_a.center_of_mass();
            let anchor_b = manifold_point.position() - body_b.center_of_mass();
            let effective_mass = |direction: Vec2| {
                let (arm_a, arm_b) = (anchor_a.perp_dot(direction), anchor_b.perp_dot(direction));
                let inverse = mass_a + mass_b + inertia_a * arm_a * arm_a + inertia_b * arm_b * arm_b;
                if inverse > 0.0 { 1.0 / inverse } else { 0.0 }
            };
            *point = ConstraintPoint {
                anchor_a,
                anchor_b,
                base_separation: manifold_point.separation() - (anchor_b - anchor_a).dot(normal),
                normal_mass: effective_mass(normal),
                tangent_mass: effective_mass(tangent),
                normal_impulse: manifold_point.normal_impulse(),
                tangent_impulse: manifold_point.tangent_impulse(),
                max_normal_impulse: 0.0,
                approach_speed: relative_velocity(body_a, body_b, anchor_a, anchor_b).dot(normal),
            };
        }
        Some(constraint)
    }

    pub(crate) fn pair(&self) -> &ContactPair {
        &self.pair
    }

    /// Reapplies last step's impulses, so the solver starts close to the solution.
    pub(crate) fn warm_start(&self, bodies: &mut Arena<Body>) {
        let Some((body_a, body_b)) = bodies.get2_mut(self.pair.body_a.0, self.pair.body_b.0) else { return };
        let tangent = tangent_of(self.normal);
        for point in self.points() {
            let impulse = point.normal_impulse * self.normal + point.tangent_impulse * tangent;
            apply_impulse(body_a, body_b, impulse, point);
        }
    }

    /// Updates the approach speed of the points that have not pushed yet, with the velocities
    /// gravity just gave the bodies, before any constraint changes them.
    pub(crate) fn track_approach_speed(&mut self, bodies: &Arena<Body>) {
        if self.restitution == 0.0 {
            return;
        }
        let (Some(body_a), Some(body_b)) = (bodies.get(self.pair.body_a.0), bodies.get(self.pair.body_b.0)) else {
            return;
        };
        for point in &mut self.points[..self.point_count] {
            if point.max_normal_impulse == 0.0 {
                let normal_speed = relative_velocity(body_a, body_b, point.anchor_a, point.anchor_b).dot(self.normal);
                point.approach_speed = point.approach_speed.min(normal_speed);
            }
        }
    }

    /// Solves the contact velocities once: pushes the bodies apart without pulling them
    /// together, then applies friction.
    pub(crate) fn solve(&mut self, bodies: &mut Arena<Body>, context: &SolverContext, pass: Pass) {
        let Some((body_a, body_b)) = bodies.get2_mut(self.pair.body_a.0, self.pair.body_b.0) else { return };
        let normal = self.normal;
        let softness = self.softness;
        let (moved_a, turned_a) = (body_a.delta_position(), body_a.delta_rotation());
        let (moved_b, turned_b) = (body_b.delta_position(), body_b.delta_rotation());

        for point in &mut self.points[..self.point_count] {
            // Current separation, from how far the anchors moved since the step started.
            let displacement =
                (moved_b + turned_b.rotate(point.anchor_b)) - (moved_a + turned_a.rotate(point.anchor_a));
            let separation = displacement.dot(normal) + point.base_separation;

            let (bias, mass_scale, impulse_scale) = if separation > 0.0 {
                // Not touching yet: allow closing exactly the gap during this substep.
                (separation * context.inverse_substep, 1.0, 0.0)
            } else if pass == Pass::Solve {
                let push = (softness.bias_rate * separation).max(-MAX_CONTACT_PUSH_SPEED);
                (push, softness.mass_scale, softness.impulse_scale)
            } else {
                (0.0, 1.0, 0.0)
            };

            let normal_speed = relative_velocity(body_a, body_b, point.anchor_a, point.anchor_b).dot(normal);
            let impulse =
                -point.normal_mass * mass_scale * (normal_speed + bias) - impulse_scale * point.normal_impulse;
            // Contacts can push but never pull.
            let total = (point.normal_impulse + impulse).max(0.0);
            let applied = total - point.normal_impulse;
            point.normal_impulse = total;
            point.max_normal_impulse = point.max_normal_impulse.max(total);
            apply_impulse(body_a, body_b, applied * normal, point);
        }

        let tangent = tangent_of(normal);
        for point in &mut self.points[..self.point_count] {
            let tangent_speed = relative_velocity(body_a, body_b, point.anchor_a, point.anchor_b).dot(tangent);
            let impulse = -point.tangent_mass * tangent_speed;
            // Coulomb friction: bounded by the normal force.
            let limit = self.friction * point.normal_impulse;
            let total = (point.tangent_impulse + impulse).clamp(-limit, limit);
            let applied = total - point.tangent_impulse;
            point.tangent_impulse = total;
            apply_impulse(body_a, body_b, applied * tangent, point);
        }
    }

    /// Makes points that were hit fast enough bounce back.
    pub(crate) fn apply_restitution(&mut self, bodies: &mut Arena<Body>) {
        if self.restitution == 0.0 {
            return;
        }
        let Some((body_a, body_b)) = bodies.get2_mut(self.pair.body_a.0, self.pair.body_b.0) else { return };
        let normal = self.normal;
        for point in &mut self.points[..self.point_count] {
            if point.approach_speed > -RESTITUTION_THRESHOLD || point.max_normal_impulse == 0.0 {
                continue;
            }
            let normal_speed = relative_velocity(body_a, body_b, point.anchor_a, point.anchor_b).dot(normal);
            // Target a separating speed equal to `restitution` times the approach speed.
            let impulse = -point.normal_mass * (normal_speed + self.restitution * point.approach_speed);
            let total = (point.normal_impulse + impulse).max(0.0);
            let applied = total - point.normal_impulse;
            point.normal_impulse = total;
            point.max_normal_impulse = point.max_normal_impulse.max(total);
            apply_impulse(body_a, body_b, applied * normal, point);
        }
    }

    /// Saves the impulses in the manifold, to warm start the next step.
    pub(crate) fn store_impulses(&self, manifold: &mut Manifold) {
        for (manifold_point, point) in manifold.points_mut().iter_mut().zip(self.points()) {
            manifold_point.set_impulses(point.normal_impulse, point.tangent_impulse);
        }
    }

    fn points(&self) -> &[ConstraintPoint] {
        &self.points[..self.point_count]
    }
}

/// The tangent direction used for friction: the normal turned a quarter clockwise.
fn tangent_of(normal: Vec2) -> Vec2 {
    vec2(normal.y, -normal.x)
}

/// Velocity of B's material relative to A's at the contact point.
fn relative_velocity(body_a: &Body, body_b: &Body, anchor_a: Vec2, anchor_b: Vec2) -> Vec2 {
    let velocity_a = body_a.linear_velocity() + body_a.angular_velocity() * anchor_a.perp();
    let velocity_b = body_b.linear_velocity() + body_b.angular_velocity() * anchor_b.perp();
    velocity_b - velocity_a
}

/// Applies `impulse` to B and its opposite to A.
fn apply_impulse(body_a: &mut Body, body_b: &mut Body, impulse: Vec2, point: &ConstraintPoint) {
    body_a.apply_solver_impulse(-impulse, point.anchor_a);
    body_b.apply_solver_impulse(impulse, point.anchor_b);
}
