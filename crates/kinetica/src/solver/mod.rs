//! The constraint solver: moves bodies while keeping contacts satisfied.
//!
//! Each step is split into substeps. Every substep integrates velocities, solves the contacts
//! with a soft push-out, integrates positions, then relaxes the contacts to remove the push-out
//! velocity. Restitution is applied once at the end. This is the "soft step" solver of Box2D v3.

mod contact_constraint;
mod softness;

use contact_constraint::ContactConstraint;
use softness::Softness;

use crate::arena::Arena;
use crate::body::Body;
use crate::collider::Collider;
use crate::constants::{CONTACT_DAMPING_RATIO, CONTACT_HERTZ};
use crate::contact::ContactSet;
use crate::math::Vec2;

pub(crate) struct StepSettings {
    pub(crate) gravity: Vec2,
    pub(crate) dt: f32,
    pub(crate) substep_count: u32,
}

/// Advances every body by one step, resolving contacts.
pub(crate) fn step(
    bodies: &mut Arena<Body>,
    colliders: &Arena<Collider>,
    contacts: &mut ContactSet,
    settings: &StepSettings,
) {
    let substep = settings.dt / settings.substep_count as f32;
    let inverse_substep = 1.0 / substep;
    let contact_hertz = CONTACT_HERTZ.min(0.25 * inverse_substep);
    let softness = Softness::new(contact_hertz, CONTACT_DAMPING_RATIO, substep);
    let fixed_softness = Softness::new(2.0 * contact_hertz, CONTACT_DAMPING_RATIO, substep);

    let mut constraints: Vec<_> = contacts
        .touching()
        .filter_map(|contact| ContactConstraint::new(contact, colliders, bodies, softness, fixed_softness))
        .collect();

    for body in bodies.values_mut() {
        body.begin_step();
    }

    for _ in 0..settings.substep_count {
        for body in bodies.values_mut() {
            body.integrate_velocity(settings.gravity, substep);
        }
        for constraint in &constraints {
            constraint.warm_start(bodies);
        }
        for constraint in &mut constraints {
            constraint.solve(bodies, inverse_substep, true);
        }
        for body in bodies.values_mut() {
            body.integrate_position(substep);
        }
        for constraint in &mut constraints {
            constraint.solve(bodies, inverse_substep, false);
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
