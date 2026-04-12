use crate::constraints::ContactManifold;
use crate::core::RigidBody;
use crate::math::Vec2;

pub fn circle_vs_circle(a: &RigidBody, b: &RigidBody, radius_a: f32, radius_b: f32) -> Option<ContactManifold> {
    let diff = b.state.position - a.state.position;
    let dist_sq = diff.length_squared();
    let radius_sum = radius_a + radius_b;

    if dist_sq >= radius_sum * radius_sum {
        return None;
    }

    let dist = diff.length();

    if dist < 1e-6 {
        return Some(ContactManifold {
            body_a: 0,
            body_b: 0,
            normal: Vec2::new(0.0, 1.0),
            penetration: radius_sum,
        });
    }

    Some(ContactManifold {
        body_a: 0,
        body_b: 0,
        normal: diff / dist,
        penetration: radius_sum - dist,
    })
}
