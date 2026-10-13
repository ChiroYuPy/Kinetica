//! Continuous collision: stops fast bodies before they pass through thin obstacles.
//!
//! Contacts are only found between steps, so a body moving further than its own thickness in a
//! single step could jump over a wall. After the solver, each fast dynamic body sweeps its
//! colliders from where the step started to where it ended. If they hit something on the way,
//! the body is moved back to the first time of impact, keeping its velocity; the contact found
//! at the next step then stops it.
//!
//! The sweep follows the motion of the center of mass with the final orientation, so a fast
//! spinning body may still clip a corner.

use crate::arena::Arena;
use crate::body::{Body, BodyHandle, BodyType};
use crate::broad_phase::BroadPhase;
use crate::collider::Collider;
use crate::collision::{separation_lower_bound, time_of_impact};
use crate::constants::CONTINUOUS_THRESHOLD;
use crate::contact::{ContactPair, should_collide};
use crate::joint::Joint;
use crate::math::{Transform, Vec2};
use crate::shape::Shape;
use kinetica_collision::SPECULATIVE_DISTANCE;

/// Moves fast bodies back to where they first hit something, and returns them.
pub(crate) fn solve(
    bodies: &mut Arena<Body>,
    colliders: &Arena<Collider>,
    joints: &Arena<Joint>,
    broad_phase: &BroadPhase,
) -> Vec<BodyHandle> {
    let fast: Vec<BodyHandle> = bodies
        .iter()
        .filter(|(_, body)| {
            body.body_type() == BodyType::Dynamic
                && body.is_active()
                && body.delta_position().length() > CONTINUOUS_THRESHOLD * body.min_extent()
        })
        .map(|(index, _)| BodyHandle(index))
        .collect();

    let mut moved = Vec::new();
    for handle in fast {
        let Some(body) = bodies.get(handle.0) else { continue };
        let fraction = first_impact(handle, body, bodies, colliders, joints, broad_phase);
        if fraction < 1.0 {
            let start = body.center_at_step_start();
            let end = body.center_of_mass();
            if let Some(body) = bodies.get_mut(handle.0) {
                body.move_center_to(start.lerp(end, fraction));
            }
            moved.push(handle);
        }
    }
    moved
}

/// Fraction of its last motion at which `body` first touches something it must not cross.
fn first_impact(
    handle: BodyHandle,
    body: &Body,
    bodies: &Arena<Body>,
    colliders: &Arena<Collider>,
    joints: &Arena<Joint>,
    broad_phase: &BroadPhase,
) -> f32 {
    let translation = body.delta_position();
    let end = *body.transform();
    let start = Transform::new(end.position - translation, end.rotation);

    let mut first = 1.0_f32;
    for &collider_handle in body.colliders() {
        let Some(collider) = colliders.get(collider_handle.0) else { continue };
        if collider.is_sensor() {
            continue;
        }
        let shape = collider.shape();
        let swept = shape.compute_aabb(&start).union(&shape.compute_aabb(&end));
        broad_phase.query(&swept, |other_handle| {
            let Some(other) = colliders.get(other_handle.0) else { return };
            let Some(other_body) = bodies.get(other.body().0) else { return };
            let pair = ContactPair {
                collider_a: collider_handle,
                collider_b: other_handle,
                body_a: handle,
                body_b: other.body(),
            };
            if !blocks(body, collider, other_body, other, translation)
                || !should_collide(&pair, bodies, joints)
                // Already touching: the regular contact takes care of it.
                || separation_lower_bound(shape, &start, other.shape(), other_body.transform()) < SPECULATIVE_DISTANCE
            {
                return;
            }
            if let Some(fraction) =
                time_of_impact(shape, &start, translation, other.shape(), other_body.transform(), first)
            {
                first = first.min(fraction);
            }
        });
    }
    first
}

/// Whether the sweep of `collider` must stop at `other`.
fn blocks(body: &Body, collider: &Collider, other_body: &Body, other: &Collider, translation: Vec2) -> bool {
    let target_type_ok = match other_body.body_type() {
        BodyType::Fixed | BodyType::Kinematic => true,
        // Bullets are checked against regular dynamic bodies, which have finished moving.
        BodyType::Dynamic => body.is_bullet() && !other_body.is_bullet(),
    };
    // A one-way platform or a chain only stops what comes from its blocking side.
    let blocking_side = match other.shape() {
        Shape::ChainSegment(chain) => Some(chain.segment().normal()),
        _ => other.one_way(),
    };
    let one_way_ok = blocking_side.is_none_or(|direction| other_body.world_vector(direction).dot(translation) < 0.0);
    target_type_ok
        && one_way_ok
        && !other.is_sensor()
        && collider.body() != other.body()
        && collider.layers().interacts_with(&other.layers())
}
