use crate::constraints::ContactManifold;
use crate::math::Vec2;
use crate::RigidBody;

pub fn test(circle: &RigidBody, rect: &RigidBody, radius: f32, rect_size: Vec2) -> Option<ContactManifold> {
    let half = rect_size / 2.0;
    let min = rect.state.position - half;
    let max = rect.state.position + half;

    // Find closest point on the rectangle to the circle center
    let closest = Vec2::new(
        circle.state.position.x.clamp(min.x, max.x),
        circle.state.position.y.clamp(min.y, max.y),
    );

    let diff = circle.state.position - closest;
    let dist_sq = diff.length_squared();

    if dist_sq > radius * radius {
        return None;
    }

    let dist = dist_sq.sqrt();

    // If center is inside the rectangle (deep penetration)
    if dist < 1e-6 {
        // Find which edge is closest
        let to_left = circle.state.position.x - min.x;
        let to_right = max.x - circle.state.position.x;
        let to_top = circle.state.position.y - min.y;
        let to_bottom = max.y - circle.state.position.y;

        let min_dist = to_left.min(to_right).min(to_top).min(to_bottom);

        // Normal points from rect to circle (push circle out)
        let normal = if min_dist == to_left {
            Vec2::new(-1.0, 0.0)
        } else if min_dist == to_right {
            Vec2::new(1.0, 0.0)
        } else if min_dist == to_top {
            Vec2::new(0.0, -1.0)
        } else {
            Vec2::new(0.0, 1.0)
        };

        Some(ContactManifold {
            body_a: 0,
            body_b: 0,
            normal,
            penetration: min_dist + radius,
        })
    } else {
        // Normal case: circle center is outside, touching edge/corner
        // Normal points from rect to circle
        Some(ContactManifold {
            body_a: 0,
            body_b: 0,
            normal: diff / dist,
            penetration: radius - dist,
        })
    }
}
