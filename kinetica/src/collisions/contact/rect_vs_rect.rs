use crate::constraints::ContactManifold;
use crate::core::RigidBody;
use crate::math::Vec2;

pub fn test(a: &RigidBody, b: &RigidBody, size_a: Vec2, size_b: Vec2) -> Option<ContactManifold> {
    let half_a = size_a / 2.0;
    let half_b = size_b / 2.0;

    let min_a = a.state.position - half_a;
    let max_a = a.state.position + half_a;
    let min_b = b.state.position - half_b;
    let max_b = b.state.position + half_b;

    // Check for overlap on each axis
    if max_a.x < min_b.x || min_a.x > max_b.x {
        return None;
    }
    if max_a.y < min_b.y || min_a.y > max_b.y {
        return None;
    }

    // Calculate overlap amounts
    let overlap_x = max_a.x.min(max_b.x) - min_a.x.max(min_b.x);
    let overlap_y = max_a.y.min(max_b.y) - min_a.y.max(min_b.y);

    // Use the smallest overlap for the collision normal
    let (normal, penetration) = if overlap_x < overlap_y {
        let normal = if a.state.position.x < b.state.position.x {
            Vec2::new(1.0, 0.0)
        } else {
            Vec2::new(-1.0, 0.0)
        };
        (normal, overlap_x)
    } else {
        let normal = if a.state.position.y < b.state.position.y {
            Vec2::new(0.0, 1.0)
        } else {
            Vec2::new(0.0, -1.0)
        };
        (normal, overlap_y)
    };

    Some(ContactManifold {
        body_a: 0,
        body_b: 0,
        normal,
        penetration,
    })
}
