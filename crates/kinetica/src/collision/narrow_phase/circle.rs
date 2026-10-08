//! Contacts involving a circle.

use crate::collision::{Manifold, ManifoldPoint};
use crate::constants::SPECULATIVE_DISTANCE;
use crate::math::{Transform, Vec2};
use crate::shape::{Circle, Polygon};

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

pub(super) fn collide_polygon_circle(
    polygon: &Polygon,
    transform_a: &Transform,
    circle: &Circle,
    transform_b: &Transform,
) -> Manifold {
    // Work in the polygon's local frame.
    let center = transform_a.inverse_transform_point(transform_b.transform_point(circle.center()));
    let radius = circle.radius();
    let vertices = polygon.vertices();
    let normals = polygon.normals();

    // The edge the circle center is farthest in front of.
    let mut edge = 0;
    let mut face_separation = f32::NEG_INFINITY;
    for (i, (vertex, normal)) in vertices.iter().zip(normals).enumerate() {
        let separation = normal.dot(center - *vertex);
        if separation > face_separation {
            edge = i;
            face_separation = separation;
        }
    }
    if face_separation - radius > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }

    // Closest point of the polygon boundary to the circle center, and the direction from it
    // to the center.
    let start = vertices[edge];
    let end = vertices[(edge + 1) % vertices.len()];
    let face_normal = normals[edge];
    let (closest, local_normal) = if face_separation > 0.0 && (center - start).dot(end - start) < 0.0 {
        (start, (center - start).try_normalize().unwrap_or(face_normal))
    } else if face_separation > 0.0 && (center - end).dot(start - end) < 0.0 {
        (end, (center - end).try_normalize().unwrap_or(face_normal))
    } else {
        // The center is in front of the edge, or inside the polygon.
        (center - face_separation * face_normal, face_normal)
    };

    let separation = (center - closest).dot(local_normal) - radius;
    if separation > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }

    let normal = transform_a.rotation.rotate(local_normal);
    let surface_a = transform_a.transform_point(closest);
    let surface_b = transform_b.transform_point(circle.center()) - radius * normal;

    let mut manifold = Manifold::new(normal);
    manifold.push(ManifoldPoint::new(0.5 * (surface_a + surface_b), separation, 0));
    manifold
}

#[cfg(test)]
mod tests {
    use crate::math::{Rot, vec2};

    use super::*;

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
        let ground = Polygon::rectangle(2.0, 0.5);
        let manifold = collide_polygon_circle(&ground, &at(0.0, 0.0), &Circle::new(0.5), &at(0.3, 0.95));
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-6));
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() + 0.05).abs() < 1e-5);
        assert!(point.position().abs_diff_eq(vec2(0.3, 0.475), 1e-5));
    }

    #[test]
    fn circle_against_box_corner() {
        let block = Polygon::rectangle(1.0, 1.0);
        let manifold = collide_polygon_circle(&block, &at(0.0, 0.0), &Circle::new(0.5), &at(1.3, 1.3));
        let diagonal = vec2(1.0, 1.0).normalize();
        assert!(manifold.normal().abs_diff_eq(diagonal, 1e-5));
        let [point] = manifold.points() else { panic!("expected one point") };
        let expected = 0.3 * std::f32::consts::SQRT_2 - 0.5;
        assert!((point.separation() - expected).abs() < 1e-5);
    }

    #[test]
    fn circle_center_inside_rotated_box() {
        let block = Polygon::rectangle(1.0, 1.0);
        let transform = Transform::new(Vec2::ZERO, Rot::from_angle(std::f32::consts::FRAC_PI_2));
        let manifold = collide_polygon_circle(&block, &transform, &Circle::new(0.25), &at(0.0, 0.8));
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-5));
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() + 0.45).abs() < 1e-5);
    }
}
