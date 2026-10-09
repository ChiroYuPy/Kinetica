//! Contacts involving a circle.

use super::convex::Rounded;
use crate::collision::{Manifold, ManifoldPoint};
use crate::constants::SPECULATIVE_DISTANCE;
use crate::math::{Transform, Vec2};
use crate::shape::Circle;

pub(super) fn collide_circles(a: &Circle, transform_a: &Transform, b: &Circle, transform_b: &Transform) -> Manifold {
    let center_a = transform_a.transform_point(a.center());
    let center_b = transform_b.transform_point(b.center());
    let offset = center_b - center_a;
    let separation = offset.length() - a.radius() - b.radius();
    if separation > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }

    // Concentric circles have no preferred direction; push them apart vertically.
    let normal = offset.try_normalize().unwrap_or(Vec2::Y);
    let surface_a = center_a + a.radius() * normal;
    let surface_b = center_b - b.radius() * normal;

    let mut manifold = Manifold::new(normal);
    manifold.push(ManifoldPoint::new(0.5 * (surface_a + surface_b), separation, 0));
    manifold
}

/// Contact between a rounded convex shape (A) and a circle (B).
pub(super) fn collide_rounded_circle(shape: &Rounded, circle: &Circle, transform_b: &Transform) -> Manifold {
    let center = transform_b.transform_point(circle.center());
    let radius = circle.radius() + shape.radius();
    let ClosestPoint { point: closest, normal, distance } = closest_point_on_core(shape, center);
    let separation = distance - radius;
    if separation > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }

    let surface_a = closest + shape.radius() * normal;
    let surface_b = center - circle.radius() * normal;
    let mut manifold = Manifold::new(normal);
    manifold.push(ManifoldPoint::new(0.5 * (surface_a + surface_b), separation, 0));
    manifold
}

/// Distance between a rounded convex shape and a circle; negative when they overlap.
pub(super) fn rounded_circle_separation(shape: &Rounded, circle: &Circle, transform_b: &Transform) -> f32 {
    let center = transform_b.transform_point(circle.center());
    closest_point_on_core(shape, center).distance - circle.radius() - shape.radius()
}

/// The point of a core's boundary closest to a point.
struct ClosestPoint {
    point: Vec2,
    /// Outward direction from `point` towards the query point (or the face normal).
    normal: Vec2,
    /// Signed distance from the boundary to the query point, negative inside the core.
    distance: f32,
}

fn closest_point_on_core(shape: &Rounded, point: Vec2) -> ClosestPoint {
    let vertices = shape.vertices();
    let normals = shape.normals();

    // The edge the point is farthest in front of.
    let mut edge = 0;
    let mut face_distance = f32::NEG_INFINITY;
    for (i, (vertex, normal)) in vertices.iter().zip(normals).enumerate() {
        let distance = normal.dot(point - *vertex);
        if distance > face_distance {
            edge = i;
            face_distance = distance;
        }
    }

    let start = vertices[edge];
    let end = vertices[(edge + 1) % vertices.len()];
    let face_normal = normals[edge];
    let towards_vertex = |vertex: Vec2| {
        let offset = point - vertex;
        ClosestPoint { point: vertex, normal: offset.try_normalize().unwrap_or(face_normal), distance: offset.length() }
    };
    // On the line of the edge counts as outside: for a segment core, the line goes on past it.
    if face_distance >= 0.0 && (point - start).dot(end - start) < 0.0 {
        towards_vertex(start)
    } else if face_distance >= 0.0 && (point - end).dot(start - end) < 0.0 {
        towards_vertex(end)
    } else {
        // In front of the edge, or inside the core.
        ClosestPoint { point: point - face_distance * face_normal, normal: face_normal, distance: face_distance }
    }
}

#[cfg(test)]
mod tests {
    use crate::math::{Rot, vec2};
    use crate::shape::Polygon;

    use super::*;

    fn collide_polygon_circle(
        polygon: &Polygon,
        transform_a: &Transform,
        circle: &Circle,
        transform_b: &Transform,
    ) -> Manifold {
        collide_rounded_circle(&Rounded::polygon(polygon, transform_a), circle, transform_b)
    }

    fn at(x: f32, y: f32) -> Transform {
        Transform::new(vec2(x, y), Rot::IDENTITY)
    }

    #[test]
    fn overlapping_circles() {
        let manifold = collide_circles(&Circle::new(1.0), &at(0.0, 0.0), &Circle::new(0.5), &at(1.4, 0.0));
        assert_eq!(manifold.normal(), Vec2::X);
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() + 0.1).abs() < 1e-5);
        assert!(point.position().abs_diff_eq(vec2(0.95, 0.0), 1e-5));
    }

    #[test]
    fn distant_circles_do_not_touch() {
        let manifold = collide_circles(&Circle::new(1.0), &at(0.0, 0.0), &Circle::new(1.0), &at(2.5, 0.0));
        assert!(manifold.is_empty());
    }

    #[test]
    fn nearly_touching_circles_get_a_speculative_point() {
        let gap = 0.5 * SPECULATIVE_DISTANCE;
        let manifold = collide_circles(&Circle::new(1.0), &at(0.0, 0.0), &Circle::new(1.0), &at(2.0 + gap, 0.0));
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() - gap).abs() < 1e-5);
    }

    #[test]
    fn circle_resting_on_box_face() {
        let ground = Polygon::rectangle(4.0, 1.0);
        let manifold = collide_polygon_circle(&ground, &at(0.0, 0.0), &Circle::new(0.5), &at(0.3, 0.95));
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-6));
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() + 0.05).abs() < 1e-5);
        assert!(point.position().abs_diff_eq(vec2(0.3, 0.475), 1e-5));
    }

    #[test]
    fn circle_against_box_corner() {
        let block = Polygon::rectangle(2.0, 2.0);
        let manifold = collide_polygon_circle(&block, &at(0.0, 0.0), &Circle::new(0.5), &at(1.3, 1.3));
        let diagonal = vec2(1.0, 1.0).normalize();
        assert!(manifold.normal().abs_diff_eq(diagonal, 1e-5));
        let [point] = manifold.points() else { panic!("expected one point") };
        let expected = 0.3 * std::f32::consts::SQRT_2 - 0.5;
        assert!((point.separation() - expected).abs() < 1e-5);
    }

    #[test]
    fn circle_center_inside_rotated_box() {
        let block = Polygon::rectangle(2.0, 2.0);
        let transform = Transform::new(Vec2::ZERO, Rot::from_angle(std::f32::consts::FRAC_PI_2));
        let manifold = collide_polygon_circle(&block, &transform, &Circle::new(0.25), &at(0.0, 0.8));
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-5));
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() + 0.45).abs() < 1e-5);
    }
}

#[cfg(test)]
mod capsule_tests {
    use crate::math::{Rot, vec2};
    use crate::shape::Capsule;

    use super::*;

    #[test]
    fn circle_on_capsule_side_and_end() {
        let capsule = Rounded::capsule(&Capsule::new(vec2(-1.0, 0.0), vec2(1.0, 0.0), 0.5), &Transform::IDENTITY);
        let side = collide_rounded_circle(&capsule, &Circle::new(0.5), &Transform::new(vec2(0.2, 0.9), Rot::IDENTITY));
        assert!(side.normal().abs_diff_eq(Vec2::Y, 1e-6));
        assert!((side.points()[0].separation() + 0.1).abs() < 1e-5);

        let end = collide_rounded_circle(&capsule, &Circle::new(0.5), &Transform::new(vec2(2.0, 0.0), Rot::IDENTITY));
        assert!(end.normal().abs_diff_eq(Vec2::X, 1e-6));
        assert!(end.points()[0].separation().abs() < 1e-5);
    }
}
