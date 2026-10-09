//! The constraint solver: moves bodies while keeping contacts satisfied.
//!
//! Each step is split into substeps. Every substep integrates velocities, solves the contacts
//! with a soft push-out, integrates positions, then relaxes the contacts to remove the push-out
//! velocity. Restitution is applied once at the end. This is the "soft step" solver of Box2D v3.

mod contact_constraint;
mod softness;

use contact_constraint::ContactConstraint;
pub(crate) use softness::Softness;

use crate::arena::Arena;
use crate::body::{Body, BodyHandle};
use crate::collider::Collider;
use crate::constants::{CONTACT_DAMPING_RATIO, CONTACT_HERTZ, JOINT_DAMPING_RATIO, JOINT_HERTZ_RATIO};
use crate::contact::ContactSet;
use crate::joint::Joint;
use crate::math::Vec2;

/// What [`step`] needs from the world.
pub(crate) struct StepSettings {
    pub(crate) gravity: Vec2,
    pub(crate) dt: f32,
    pub(crate) substep_count: u32,
}

/// The two kinds of constraint passes of a substep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Pass {
    /// Removes velocity errors and pushes position errors (overlaps, drifted anchors) away.
    Solve,
    /// Removes velocity errors only, after the positions moved: this takes out the extra
    /// velocity the push added, so bodies separate without flying apart.
    Relax,
}

/// Values shared by every constraint during a step.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SolverContext {
    pub(crate) substep: f32,
    pub(crate) inverse_substep: f32,
    pub(crate) contact_softness: Softness,
    /// Contacts against fixed or kinematic bodies, which can be stiffer.
    pub(crate) fixed_contact_softness: Softness,
    pub(crate) joint_softness: Softness,
}

impl SolverContext {
    fn new(settings: &StepSettings) -> Self {
        let substep = settings.dt / settings.substep_count as f32;
        let inverse_substep = 1.0 / substep;
        let contact_hertz = CONTACT_HERTZ.min(0.25 * inverse_substep);
        Self {
            substep,
            inverse_substep,
            contact_softness: Softness::new(contact_hertz, CONTACT_DAMPING_RATIO, substep),
            fixed_contact_softness: Softness::new(2.0 * contact_hertz, CONTACT_DAMPING_RATIO, substep),
            joint_softness: Softness::new(JOINT_HERTZ_RATIO * contact_hertz, JOINT_DAMPING_RATIO, substep),
        }
    }

    /// Softness of a joint constraint that keeps a position error at zero.
    pub(crate) fn equality_softness(&self, pass: Pass) -> Softness {
        match pass {
            Pass::Solve => self.joint_softness,
            Pass::Relax => Softness::RIGID,
        }
    }

    /// Softness of a joint limit with `gap` of room left, negative once crossed.
    ///
    /// While there is room, the limit only stops the motion that would cross it during this
    /// substep; once crossed, it pushes back softly like an equality.
    pub(crate) fn limit_softness(&self, gap: f32, pass: Pass) -> Softness {
        if gap > 0.0 { Softness::closing_within(self.inverse_substep) } else { self.equality_softness(pass) }
    }
}

/// Advances every body by one step, resolving joints and contacts.
pub(crate) fn step(
    bodies: &mut Arena<Body>,
    colliders: &Arena<Collider>,
    contacts: &mut ContactSet,
    joints: &mut Arena<Joint>,
    settings: &StepSettings,
) {
    let context = SolverContext::new(settings);
    let is_active = |handle: BodyHandle| bodies.get(handle.0).is_some_and(Body::is_active);

    // Sleeping bodies are skipped, along with the constraints between them.
    let awake_joints: Vec<bool> =
        joints.iter().map(|(_, joint)| is_active(joint.body_a()) || is_active(joint.body_b())).collect();
    let mut constraints: Vec<_> = contacts
        .touching()
        .filter(|contact| is_active(contact.pair().body_a) || is_active(contact.pair().body_b))
        .filter_map(|contact| ContactConstraint::new(contact, colliders, bodies, &context))
        .collect();
    for joint in awake(joints, &awake_joints) {
        joint.prepare(bodies, &context);
    }

    for body in bodies.values_mut() {
        body.begin_step();
    }

    for _ in 0..settings.substep_count {
        for body in bodies.values_mut() {
            body.integrate_velocity(settings.gravity, context.substep);
        }

        for joint in awake(joints, &awake_joints) {
            joint.warm_start(bodies);
        }
        for constraint in &constraints {
            constraint.warm_start(bodies);
        }

        // Joints first: contacts are solved last because overlap is the most visible error.
        for joint in awake(joints, &awake_joints) {
            joint.solve(bodies, &context, Pass::Solve);
        }
        for constraint in &mut constraints {
            constraint.solve(bodies, &context, Pass::Solve);
        }

        for body in bodies.values_mut() {
            body.integrate_position(context.substep);
        }

        for joint in awake(joints, &awake_joints) {
            joint.solve(bodies, &context, Pass::Relax);
        }
        for constraint in &mut constraints {
            constraint.solve(bodies, &context, Pass::Relax);
        }
    }

    for constraint in &mut constraints {
        constraint.apply_restitution(bodies);
    }
    // Constraints were made in the order of the contacts: walking both together pairs them up.
    let mut constraints = constraints.iter().peekable();
    for contact in contacts.touching_mut() {
        if let Some(constraint) = constraints.next_if(|constraint| constraint.pair() == contact.pair()) {
            constraint.store_impulses(contact.manifold_mut());
        }
    }
}

/// The joints flagged in `awake`, which follows the arena order.
fn awake<'a>(joints: &'a mut Arena<Joint>, awake: &'a [bool]) -> impl Iterator<Item = &'a mut Joint> {
    joints.values_mut().zip(awake).filter_map(|(joint, &awake)| awake.then_some(joint))
}
