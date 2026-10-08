//! Contacts between two convex polygons, using the separating axis theorem and clipping.
//!
//! 1. Find the edge normal (of either polygon) along which the polygons are the most separated.
//!    If they are separated along it, they do not touch.
//! 2. The polygon owning that edge is the *reference*; the other one's edge facing it most
//!    directly is the *incident* edge.
//! 3. Clip the incident edge to the sides of the reference edge. The remaining end points are
//!    the contact points.

use crate::collision::{Manifold, ManifoldPoint};
use crate::constants::{LINEAR_SLOP, SPECULATIVE_DISTANCE};
use crate::math::{Transform, Vec2, vec2};
use crate::shape::{MAX_POLYGON_VERTICES, Polygon};

/// Polygon B only becomes the reference when its axis is clearly better than A's, so the choice
/// does not flicker from one step to the next for nearly equal separations.
const REFERENCE_TOLERANCE: f32 = 0.1 * LINEAR_SLOP;

/// Feature values of points created by clipping against side 0 or 1, rather than taken from an
/// incident vertex. They cannot clash with vertex indices.
const CLIPPED_FEATURE: u8 = u8::MAX - 1;

/// A point of the incident edge, and the feature that produced it.
#[derive(Debug, Clone, Copy)]
struct ClipPoint {
    position: Vec2,
    feature: u8,
}

pub(super) fn collide_polygons(a: &Polygon, transform_a: &Transform, b: &Polygon, transform_b: &Transform) -> Manifold {
    let (edge_a, separation_a) = find_max_separation(a, transform_a, b, transform_b);
    if separation_a > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }
    let (edge_b, separation_b) = find_max_separation(b, transform_b, a, transform_a);
    if separation_b > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }

    let flip = separation_b > separation_a + REFERENCE_TOLERANCE;
    let (reference, reference_transform, reference_edge, incident, incident_transform) =
        if flip { (b, transform_b, edge_b, a, transform_a) } else { (a, transform_a, edge_a, b, transform_b) };

    let reference_vertices = reference.vertices();
    let start = reference_transform.transform_point(reference_vertices[reference_edge]);
    let end = reference_transform.transform_point(reference_vertices[(reference_edge + 1) % reference_vertices.len()]);
    let tangent = (end - start).normalize();
    let normal = vec2(tangent.y, -tangent.x);

    let incident_edge = find_incident_edge(normal, incident, incident_transform);
    let Some(clipped) = clip_segment(incident_edge, -tangent, -tangent.dot(start), 0) else {
        return Manifold::default();
    };
    let Some(clipped) = clip_segment(clipped, tangent, tangent.dot(end), 1) else {
        return Manifold::default();
    };

    let front_offset = normal.dot(start);
    let mut manifold = Manifold::new(if flip { -normal } else { normal });
    for point in clipped {
        let separation = normal.dot(point.position) - front_offset;
        if separation <= SPECULATIVE_DISTANCE {
            // Halfway between the incident point and the reference face.
            let position = point.position - 0.5 * separation * normal;
            let feature_id =
                u32::from(flip) << 16 | u32::from(vertex_feature(reference_edge)) << 8 | u32::from(point.feature);
            manifold.push(ManifoldPoint::new(position, separation, feature_id));
        }
    }
    manifold
}

/// Finds the edge of `polygon` along whose normal `other` is the farthest away.
/// Returns the edge index and that separation (negative when overlapping).
fn find_max_separation(
    polygon: &Polygon,
    transform: &Transform,
    other: &Polygon,
    other_transform: &Transform,
) -> (usize, f32) {
    let mut best = (0, f32::NEG_INFINITY);
    for (i, (&vertex, &normal)) in polygon.vertices().iter().zip(polygon.normals()).enumerate() {
        // Express the edge in the other polygon's frame to avoid transforming its vertices.
        let normal = other_transform.rotation.inverse_rotate(transform.rotation.rotate(normal));
        let vertex = other_transform.inverse_transform_point(transform.transform_point(vertex));
        let separation = other
            .vertices()
            .iter()
            .map(|&other_vertex| normal.dot(other_vertex - vertex))
            .fold(f32::INFINITY, f32::min);
        if separation > best.1 {
            best = (i, separation);
        }
    }
    best
}

/// Returns, in world space, the edge of `incident` whose normal is the most opposed to
/// `reference_normal`.
fn find_incident_edge(reference_normal: Vec2, incident: &Polygon, transform: &Transform) -> [ClipPoint; 2] {
    let local_normal = transform.rotation.inverse_rotate(reference_normal);
    let mut edge = 0;
    let mut min_dot = f32::INFINITY;
    for (i, normal) in incident.normals().iter().enumerate() {
        let dot = normal.dot(local_normal);
        if dot < min_dot {
            edge = i;
            min_dot = dot;
        }
    }

    let vertices = incident.vertices();
    let next = (edge + 1) % vertices.len();
    [
        ClipPoint { position: transform.transform_point(vertices[edge]), feature: vertex_feature(edge) },
        ClipPoint { position: transform.transform_point(vertices[next]), feature: vertex_feature(next) },
    ]
}

/// Keeps the part of `segment` where `direction · p <= offset`.
/// Returns `None` when the whole segment is on the wrong side.
fn clip_segment(segment: [ClipPoint; 2], direction: Vec2, offset: f32, side: u8) -> Option<[ClipPoint; 2]> {
    let [first, second] = segment;
    let first_distance = direction.dot(first.position) - offset;
    let second_distance = direction.dot(second.position) - offset;

    match (first_distance <= 0.0, second_distance <= 0.0) {
        (true, true) => Some(segment),
        (false, false) => None,
        (first_kept, _) => {
            let t = first_distance / (first_distance - second_distance);
            let crossing =
                ClipPoint { position: first.position.lerp(second.position, t), feature: CLIPPED_FEATURE + side };
            Some(if first_kept { [first, crossing] } else { [crossing, second] })
        }
    }
}

#[allow(clippy::cast_possible_truncation, reason = "polygon indices are below MAX_POLYGON_VERTICES")]
fn vertex_feature(index: usize) -> u8 {
    debug_assert!(index < MAX_POLYGON_VERTICES);
    index as u8
}

#[cfg(test)]
mod tests {
    use std::f32::consts::FRAC_PI_4;

    use crate::math::Rot;

    use super::*;

    fn at(x: f32, y: f32) -> Transform {
        Transform::new(vec2(x, y), Rot::IDENTITY)
    }

    #[test]
    fn box_resting_on_box() {
        let ground = Polygon::rectangle(1.0, 1.0);
        let block = Polygon::rectangle(0.5, 0.5);
        let manifold = collide_polygons(&ground, &at(0.0, 0.0), &block, &at(0.0, 1.4));

        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-6));
        let [first, second] = manifold.points() else { panic!("expected two points") };
        for point in [first, second] {
            assert!((point.separation() + 0.1).abs() < 1e-5);
            assert!((point.position().y - 0.95).abs() < 1e-5);
        }
        let mut xs = [first.position().x, second.position().x];
        xs.sort_by(f32::total_cmp);
        assert!((xs[0] + 0.5).abs() < 1e-5 && (xs[1] - 0.5).abs() < 1e-5);
    }

    #[test]
    fn normal_points_from_a_to_b_whichever_is_the_reference() {
        let small = Polygon::rectangle(0.5, 0.5);
        let large = Polygon::rectangle(1.0, 1.0);
        // The small box is on top; whichever box provides the reference face, the normal must
        // point from A to B.
        let manifold = collide_polygons(&small, &at(0.0, 1.4), &large, &at(0.0, 0.0));
        assert!(manifold.normal().abs_diff_eq(-Vec2::Y, 1e-6));
        assert_eq!(manifold.points().len(), 2);
    }

    #[test]
    fn separated_boxes_do_not_touch() {
        let block = Polygon::rectangle(0.5, 0.5);
        assert!(collide_polygons(&block, &at(0.0, 0.0), &block, &at(1.5, 0.2)).is_empty());
    }

    #[test]
    fn diamond_corner_on_box_gives_one_point() {
        let ground = Polygon::rectangle(2.0, 0.5);
        let diamond = Polygon::rectangle(0.5, 0.5);
        let corner_depth = 0.02;
        let half_diagonal = 0.5 * std::f32::consts::SQRT_2;
        let transform = Transform::new(vec2(0.0, 0.5 + half_diagonal - corner_depth), Rot::from_angle(FRAC_PI_4));

        let manifold = collide_polygons(&ground, &at(0.0, 0.0), &diamond, &transform);
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-5));
        let [point] = manifold.points() else { panic!("expected one point, got {:?}", manifold.points()) };
        assert!((point.separation() + corner_depth).abs() < 1e-4);
        assert!(point.position().x.abs() < 1e-4);
    }

    #[test]
    fn feature_ids_are_stable_while_sliding() {
        let ground = Polygon::rectangle(5.0, 0.5);
        let block = Polygon::rectangle(0.5, 0.5);
        let ids = |x: f32| -> Vec<u32> {
            collide_polygons(&ground, &at(0.0, 0.0), &block, &at(x, 0.99))
                .points()
                .iter()
                .map(ManifoldPoint::feature_id)
                .collect()
        };
        assert_eq!(ids(0.0), ids(0.3));
    }
}
