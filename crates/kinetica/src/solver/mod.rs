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
use crate::body::Body;
use crate::collider::Collider;
use crate::constants::{CONTACT_DAMPING_RATIO, CONTACT_HERTZ, JOINT_DAMPING_RATIO, JOINT_HERTZ_RATIO};
use crate::contact::ContactSet;
use crate::joint::Joint;
use crate::math::Vec2;

pub(crate) struct StepSettings {
    pub(crate) gravity: Vec2,
    pub(crate) dt: f32,
    pub(crate) substep_count: u32,
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

    for joint in joints.values_mut() {
        joint.prepare(bodies, &context);
    }
    let mut constraints: Vec<_> = contacts
        .touching()
        .filter_map(|contact| ContactConstraint::new(contact, colliders, bodies, &context))
        .collect();

    for body in bodies.values_mut() {
        body.begin_step();
    }

    for _ in 0..settings.substep_count {
        for body in bodies.values_mut() {
            body.integrate_velocity(settings.gravity, context.substep);
        }

        for joint in joints.values_mut() {
            joint.warm_start(bodies);
        }
        for constraint in &constraints {
            constraint.warm_start(bodies);
        }

        // Joints first: contacts are solved last because overlap is the most visible error.
        for joint in joints.values_mut() {
            joint.solve(bodies, &context, true);
        }
        for constraint in &mut constraints {
            constraint.solve(bodies, context.inverse_substep, true);
        }

        for body in bodies.values_mut() {
            body.integrate_position(context.substep);
        }

        for joint in joints.values_mut() {
            joint.solve(bodies, &context, false);
        }
        for constraint in &mut constraints {
            constraint.solve(bodies, context.inverse_substep, false);
        }
    }

    for constraint in &mut constraints {
        constraint.apply_restitution(bodies);
    }
    for constraint in &constraints {
        if let Some(contact) = contacts.get_mut(constraint.pair()) {
            constraint.store_impulses(contact.manifold_mut());
        }
    }
}
