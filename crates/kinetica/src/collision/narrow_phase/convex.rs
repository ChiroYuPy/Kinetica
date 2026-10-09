//! Contacts between rounded convex shapes: polygons, capsules and segments.
//!
//! Every such shape is a convex *core* (a polygon, or a segment for capsules) inflated by a
//! radius (zero for polygons and segments).
//!
//! 1. Find the edge normal (of either core) along which the cores are the most separated. If
//!    they are further apart than their radii, they do not touch.
//! 2. The shape owning that edge is the *reference*; the other one's edge facing it most
//!    directly is the *incident* edge.
//! 3. When the cores are apart and their closest features are two vertices (rounded corners),
//!    the contact is a single point between those vertices.
//! 4. Otherwise, clip the incident edge to the sides of the reference edge. The remaining end
//!    points are the contact points.

use crate::collision::{Manifold, ManifoldPoint};
use crate::constants::{LINEAR_SLOP, SPECULATIVE_DISTANCE};
use crate::math::{Transform, Vec2, vec2};
use crate::shape::{Capsule, MAX_POLYGON_VERTICES, Polygon, Segment};

/// The incident shape only becomes the reference when its axis is clearly better, so the
/// choice does not flicker from one step to the next for nearly equal separations.
const REFERENCE_TOLERANCE: f32 = 0.1 * LINEAR_SLOP;

/// Feature values of points created by clipping against side 0 or 1, rather than taken from an
/// incident vertex. They cannot clash with vertex indices.
pub(super) const CLIPPED_FEATURE: u8 = u8::MAX - 1;

/// Marks feature ids of single vertex-to-vertex points, apart from clipped points.
const VERTEX_PAIR_FEATURE: u32 = 1 << 24;

/// A convex core with a radius, in world space.
#[derive(Debug, Clone, Copy)]
pub(super) struct Rounded {
    vertices: [Vec2; MAX_POLYGON_VERTICES],
    /// Outward normal of the edge from vertex `i` to vertex `i + 1`.
    normals: [Vec2; MAX_POLYGON_VERTICES],
    count: usize,
    radius: f32,
}

impl Rounded {
    pub(super) fn polygon(polygon: &Polygon, transform: &Transform) -> Self {
        let mut shape = Self::empty(polygon.vertices().len(), 0.0);
        for (i, (&vertex, &normal)) in polygon.vertices().iter().zip(polygon.normals()).enumerate() {
            shape.vertices[i] = transform.transform_point(vertex);
            shape.normals[i] = transform.rotation.rotate(normal);
        }
        shape
    }

    pub(super) fn capsule(capsule: &Capsule, transform: &Transform) -> Self {
        let (start, end) = (transform.transform_point(capsule.start()), transform.transform_point(capsule.end()));
        Self::two_sided(start, end, capsule.radius())
    }

    pub(super) fn segment(segment: &Segment, transform: &Transform) -> Self {
        Self::two_sided(transform.transform_point(segment.start()), transform.transform_point(segment.end()), 0.0)
    }

    /// A two-vertex core: its two edges are the segment, run in each direction.
    pub(super) fn two_sided(start: Vec2, end: Vec2, radius: f32) -> Self {
        let mut shape = Self::empty(2, radius);
        let axis = (end - start).normalize();
        shape.vertices[0] = start;
        shape.vertices[1] = end;
        shape.normals[0] = vec2(axis.y, -axis.x);
        shape.normals[1] = -shape.normals[0];
        shape
    }

    fn empty(count: usize, radius: f32) -> Self {
        Self {
            vertices: [Vec2::ZERO; MAX_POLYGON_VERTICES],
            normals: [Vec2::ZERO; MAX_POLYGON_VERTICES],
            count,
            radius,
        }
    }

    pub(super) fn vertices(&self) -> &[Vec2] {
        &self.vertices[..self.count]
    }

    pub(super) fn normals(&self) -> &[Vec2] {
        &self.normals[..self.count]
    }

    pub(super) fn radius(&self) -> f32 {
        self.radius
    }

    /// Average of the vertices: a point inside the core.
    pub(super) fn center(&self) -> Vec2 {
        self.vertices().iter().sum::<Vec2>() / self.count as f32
    }

    fn edge(&self, index: usize) -> (Vec2, Vec2) {
        (self.vertices[index], self.vertices[(index + 1) % self.count])
    }
}

/// A point of an incident edge, and the feature that produced it.
#[derive(Debug, Clone, Copy)]
pub(super) struct ClipPoint {
    pub(super) position: Vec2,
    pub(super) feature: u8,
}

pub(super) fn collide_rounded(a: &Rounded, b: &Rounded) -> Manifold {
    if a.count == 2 && b.count == 2 {
        return collide_segment_cores(a, b);
    }
    let radius = a.radius + b.radius;
    let (edge_a, separation_a) = find_max_separation(a, b);
    if separation_a - radius > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }
    let (edge_b, separation_b) = find_max_separation(b, a);
    if separation_b - radius > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }

    let flip = separation_b > separation_a + REFERENCE_TOLERANCE;
    let (reference, reference_edge, incident) = if flip { (b, edge_b, a) } else { (a, edge_a, b) };
    let normal = reference.normals[reference_edge];
    let incident_edge = find_incident_edge(normal, incident);
    let (start, end) = reference.edge(reference_edge);
    let (incident_start, incident_end) = incident.edge(incident_edge);

    // Cores apart: two rounded corners may be closest, which clipping would get wrong.
    if separation_a.max(separation_b) > 0.1 * LINEAR_SLOP {
        let closest = closest_points_between_segments(start, end, incident_start, incident_end);
        // Fractions are clamped, so an end is exactly 0 or 1.
        #[allow(clippy::float_cmp, reason = "clamped values are exact")]
        let at_vertex = |fraction: f32| fraction == 0.0 || fraction == 1.0;
        if at_vertex(closest.fraction_a) && at_vertex(closest.fraction_b) {
            let offset = closest.point_b - closest.point_a;
            let distance = offset.length();
            if distance - radius > SPECULATIVE_DISTANCE {
                return Manifold::default();
            }
            let normal = offset / distance;
            let surface_reference = closest.point_a + reference.radius * normal;
            let surface_incident = closest.point_b - incident.radius * normal;
            let vertex_a =
                if closest.fraction_a == 0.0 { reference_edge } else { (reference_edge + 1) % reference.count };
            let vertex_b = if closest.fraction_b == 0.0 { incident_edge } else { (incident_edge + 1) % incident.count };
            let feature_id = VERTEX_PAIR_FEATURE
                | u32::from(flip) << 16
                | u32::from(vertex_feature(vertex_a)) << 8
                | u32::from(vertex_feature(vertex_b));
            let mut manifold = Manifold::new(if flip { -normal } else { normal });
            manifold.push(ManifoldPoint::new(
                0.5 * (surface_reference + surface_incident),
                distance - radius,
                feature_id,
            ));
            return manifold;
        }
    }

    let tangent = (end - start).normalize();
    let incident_points = [
        ClipPoint { position: incident_start, feature: vertex_feature(incident_edge) },
        ClipPoint { position: incident_end, feature: vertex_feature((incident_edge + 1) % incident.count) },
    ];
    let Some(clipped) = clip_segment(incident_points, -tangent, -tangent.dot(start), 0) else {
        return Manifold::default();
    };
    let Some(clipped) = clip_segment(clipped, tangent, tangent.dot(end), 1) else {
        return Manifold::default();
    };

    let mut manifold = Manifold::new(if flip { -normal } else { normal });
    for point in clipped {
        let core_separation = normal.dot(point.position - start);
        let separation = core_separation - radius;
        if separation <= SPECULATIVE_DISTANCE {
            // Halfway between the two surfaces.
            let position = point.position + 0.5 * (reference.radius - incident.radius - core_separation) * normal;
            let feature_id =
                u32::from(flip) << 16 | u32::from(vertex_feature(reference_edge)) << 8 | u32::from(point.feature);
            manifold.push(ManifoldPoint::new(position, separation, feature_id));
        }
    }
    manifold
}

/// A lower bound of the distance between two rounded shapes, negative when they overlap.
///
/// The largest separation of the cores along any edge normal, minus the radii: exact when the
/// shapes overlap or when the closest features are an edge and a vertex, an underestimate when
/// they are two vertices.
pub(super) fn separation_lower_bound(a: &Rounded, b: &Rounded) -> f32 {
    if a.count == 2 && b.count == 2 {
        let closest = closest_points_between_segments(a.vertices[0], a.vertices[1], b.vertices[0], b.vertices[1]);
        return closest.point_a.distance(closest.point_b) - a.radius - b.radius;
    }
    find_max_separation(a, b).1.max(find_max_separation(b, a).1) - a.radius - b.radius
}

/// Below this sine of the angle between two segments, they count as parallel and touch along
/// their overlap rather than at one point.
const PARALLEL_TOLERANCE: f32 = 0.1;

/// Contact between two shapes whose cores are segments (capsules and segments).
///
/// Edge normals miss the direction along the segments, so the separating axis test does not
/// apply: the closest points of the two segments give the contact instead.
fn collide_segment_cores(a: &Rounded, b: &Rounded) -> Manifold {
    let radius = a.radius + b.radius;
    let (start_a, end_a) = (a.vertices[0], a.vertices[1]);
    let (start_b, end_b) = (b.vertices[0], b.vertices[1]);
    let closest = closest_points_between_segments(start_a, end_a, start_b, end_b);
    let offset = closest.point_b - closest.point_a;
    let distance = offset.length();
    if distance - radius > SPECULATIVE_DISTANCE {
        return Manifold::default();
    }

    let axis_a = (end_a - start_a).normalize();
    let axis_b = (end_b - start_b).normalize();
    // Crossing cores have no closest direction: push B out along A's side it is centered on.
    let mut face_normal = axis_a.perp();
    if face_normal.dot(0.5 * (start_b + end_b) - start_a) < 0.0 {
        face_normal = -face_normal;
    }
    let normal = if distance > LINEAR_SLOP * 0.01 { offset / distance } else { face_normal };

    // Side by side: two points over the length they share.
    if axis_a.perp_dot(axis_b).abs() < PARALLEL_TOLERANCE {
        let points = [ClipPoint { position: start_b, feature: 0 }, ClipPoint { position: end_b, feature: 1 }];
        let clipped = clip_segment(points, -axis_a, -axis_a.dot(start_a), 0)
            .and_then(|points| clip_segment(points, axis_a, axis_a.dot(end_a), 1));
        if let Some(clipped) = clipped {
            let mut manifold = Manifold::new(face_normal);
            for point in clipped {
                let core_separation = face_normal.dot(point.position - start_a);
                let separation = core_separation - radius;
                if separation <= SPECULATIVE_DISTANCE {
                    let position = point.position + 0.5 * (a.radius - b.radius - core_separation) * face_normal;
                    manifold.push(ManifoldPoint::new(position, separation, u32::from(point.feature)));
                }
            }
            if !manifold.is_empty() {
                return manifold;
            }
        }
    }

    let surface_a = closest.point_a + a.radius * normal;
    let surface_b = closest.point_b - b.radius * normal;
    let mut manifold = Manifold::new(normal);
    manifold.push(ManifoldPoint::new(0.5 * (surface_a + surface_b), distance - radius, VERTEX_PAIR_FEATURE));
    manifold
}

/// Finds the edge of `shape` along whose normal the core of `other` is the farthest away.
/// Returns the edge index and that separation (negative when the cores overlap).
fn find_max_separation(shape: &Rounded, other: &Rounded) -> (usize, f32) {
    let mut best = (0, f32::NEG_INFINITY);
    for (i, (&vertex, &normal)) in shape.vertices().iter().zip(shape.normals()).enumerate() {
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

/// Returns the edge of `incident` whose normal is the most opposed to `reference_normal`.
pub(super) fn find_incident_edge(reference_normal: Vec2, incident: &Rounded) -> usize {
    let mut edge = 0;
    let mut min_dot = f32::INFINITY;
    for (i, normal) in incident.normals().iter().enumerate() {
        let dot = normal.dot(reference_normal);
        if dot < min_dot {
            edge = i;
            min_dot = dot;
        }
    }
    edge
}

/// The incident edge `edge` of `shape` as two clip points.
pub(super) fn incident_points(shape: &Rounded, edge: usize) -> [ClipPoint; 2] {
    let (start, end) = shape.edge(edge);
    [
        ClipPoint { position: start, feature: vertex_feature(edge) },
        ClipPoint { position: end, feature: vertex_feature((edge + 1) % shape.count) },
    ]
}

/// Keeps the part of `segment` where `direction · p <= offset`.
/// Returns `None` when the whole segment is on the wrong side.
pub(super) fn clip_segment(segment: [ClipPoint; 2], direction: Vec2, offset: f32, side: u8) -> Option<[ClipPoint; 2]> {
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

/// The closest points of two segments, and where they are along each one (0 at the start, 1 at
/// the end).
struct ClosestPoints {
    point_a: Vec2,
    point_b: Vec2,
    fraction_a: f32,
    fraction_b: f32,
}

/// Closest points between the segments `a` and `b` (Ericson, *Real-Time Collision Detection*,
/// 5.1.9). Both segments must have a non-zero length.
fn closest_points_between_segments(start_a: Vec2, end_a: Vec2, start_b: Vec2, end_b: Vec2) -> ClosestPoints {
    let (axis_a, axis_b) = (end_a - start_a, end_b - start_b);
    let offset = start_a - start_b;
    let (length_a, length_b) = (axis_a.length_squared(), axis_b.length_squared());
    let (along_a, along_b) = (axis_a.dot(offset), axis_b.dot(offset));
    let cross = axis_a.dot(axis_b);
    let denominator = length_a * length_b - cross * cross;

    // Closest point of the infinite lines, or any point when they are parallel.
    let mut fraction_a =
        if denominator > 0.0 { ((cross * along_b - along_a * length_b) / denominator).clamp(0.0, 1.0) } else { 0.0 };
    let mut fraction_b = (cross * fraction_a + along_b) / length_b;
    // Bring each fraction back onto its segment, adjusting the other one accordingly.
    if fraction_b < 0.0 {
        fraction_b = 0.0;
        fraction_a = (-along_a / length_a).clamp(0.0, 1.0);
    } else if fraction_b > 1.0 {
        fraction_b = 1.0;
        fraction_a = ((cross - along_a) / length_a).clamp(0.0, 1.0);
    }
    ClosestPoints {
        point_a: start_a + fraction_a * axis_a,
        point_b: start_b + fraction_b * axis_b,
        fraction_a,
        fraction_b,
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

    fn collide_polygons(a: &Polygon, transform_a: &Transform, b: &Polygon, transform_b: &Transform) -> Manifold {
        collide_rounded(&Rounded::polygon(a, transform_a), &Rounded::polygon(b, transform_b))
    }

    fn at(x: f32, y: f32) -> Transform {
        Transform::new(vec2(x, y), Rot::IDENTITY)
    }

    #[test]
    fn box_resting_on_box() {
        let ground = Polygon::rectangle(2.0, 2.0);
        let block = Polygon::rectangle(1.0, 1.0);
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
        let small = Polygon::rectangle(1.0, 1.0);
        let large = Polygon::rectangle(2.0, 2.0);
        // The small box is on top; whichever box provides the reference face, the normal must
        // point from A to B.
        let manifold = collide_polygons(&small, &at(0.0, 1.4), &large, &at(0.0, 0.0));
        assert!(manifold.normal().abs_diff_eq(-Vec2::Y, 1e-6));
        assert_eq!(manifold.points().len(), 2);
    }

    #[test]
    fn separated_boxes_do_not_touch() {
        let block = Polygon::rectangle(1.0, 1.0);
        assert!(collide_polygons(&block, &at(0.0, 0.0), &block, &at(1.5, 0.2)).is_empty());
    }

    #[test]
    fn diamond_corner_on_box_gives_one_point() {
        let ground = Polygon::rectangle(4.0, 1.0);
        let diamond = Polygon::rectangle(1.0, 1.0);
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
        let ground = Polygon::rectangle(10.0, 1.0);
        let block = Polygon::rectangle(1.0, 1.0);
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

#[cfg(test)]
mod rounded_tests {
    use crate::math::Rot;

    use super::*;

    fn at(x: f32, y: f32) -> Transform {
        Transform::new(vec2(x, y), Rot::IDENTITY)
    }

    fn ground() -> Rounded {
        Rounded::polygon(&Polygon::rectangle(10.0, 1.0), &at(0.0, -0.5))
    }

    #[test]
    fn lying_capsule_rests_on_two_points() {
        let capsule = Rounded::capsule(&Capsule::new(vec2(-1.0, 0.0), vec2(1.0, 0.0), 0.25), &at(0.0, 0.24));
        let manifold = collide_rounded(&ground(), &capsule);
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-6));
        let [first, second] = manifold.points() else { panic!("expected two points, got {manifold:?}") };
        for point in [first, second] {
            assert!((point.separation() + 0.01).abs() < 1e-5);
            assert!((point.position().x.abs() - 1.0).abs() < 1e-5);
            assert!((point.position().y + 0.005).abs() < 1e-5, "halfway between surfaces: {}", point.position());
        }
    }

    #[test]
    fn standing_capsule_rests_on_one_point() {
        let capsule = Rounded::capsule(&Capsule::vertical(1.5, 0.25), &at(0.3, 0.75));
        let manifold = collide_rounded(&ground(), &capsule);
        let [point] = manifold.points() else { panic!("expected one point, got {manifold:?}") };
        assert!(point.separation().abs() < 1e-5);
        assert!(point.position().abs_diff_eq(vec2(0.3, 0.0), 1e-5));
    }

    #[test]
    fn rounded_corners_touch_along_the_diagonal() {
        // Two capsules end to end along a diagonal, their round ends 1 cm apart.
        let direction = vec2(1.0, 1.0).normalize();
        let a = Rounded::two_sided(-direction, Vec2::ZERO, 0.5);
        let start_b = (1.0 + 0.01) * direction;
        let b = Rounded::two_sided(start_b, start_b + direction, 0.5);
        let manifold = collide_rounded(&a, &b);
        assert!(manifold.normal().abs_diff_eq(direction, 1e-5), "normal {}", manifold.normal());
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() - 0.01).abs() < 1e-5);
    }

    #[test]
    fn segment_and_box() {
        let segment = Rounded::two_sided(vec2(-2.0, 0.0), vec2(2.0, 0.0), 0.0);
        let block = Rounded::polygon(&Polygon::rectangle(1.0, 1.0), &at(0.5, 0.49));
        let manifold = collide_rounded(&segment, &block);
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-6));
        assert_eq!(manifold.points().len(), 2);
        assert!(manifold.points().iter().all(|point| (point.separation() + 0.01).abs() < 1e-5));
    }

    #[test]
    fn capsules_side_by_side_touch_along_their_overlap() {
        let a = Rounded::two_sided(vec2(0.0, 0.0), vec2(2.0, 0.0), 0.25);
        let b = Rounded::two_sided(vec2(1.0, 0.49), vec2(3.0, 0.49), 0.25);
        let manifold = collide_rounded(&a, &b);
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-6));
        let [first, second] = manifold.points() else { panic!("expected two points, got {manifold:?}") };
        assert!((first.position().x - 1.0).abs() < 1e-5 && (second.position().x - 2.0).abs() < 1e-5);
        assert!((first.separation() + 0.01).abs() < 1e-5);
    }

    #[test]
    fn crossed_capsules_touch_at_one_point() {
        let a = Rounded::two_sided(vec2(-1.0, 0.0), vec2(1.0, 0.0), 0.25);
        let b = Rounded::two_sided(vec2(0.0, 0.4), vec2(0.0, 2.0), 0.25);
        let manifold = collide_rounded(&a, &b);
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-6));
        let [point] = manifold.points() else { panic!("expected one point") };
        assert!((point.separation() + 0.1).abs() < 1e-5);
    }

    #[test]
    fn closest_points_of_crossing_and_parallel_segments() {
        let crossing =
            closest_points_between_segments(vec2(-1.0, 0.0), vec2(1.0, 0.0), vec2(0.0, -1.0), vec2(0.0, 1.0));
        assert!(crossing.point_a.abs_diff_eq(Vec2::ZERO, 1e-6) && crossing.point_b.abs_diff_eq(Vec2::ZERO, 1e-6));
        let parallel = closest_points_between_segments(vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(3.0, 1.0), vec2(4.0, 1.0));
        assert_eq!((parallel.fraction_a, parallel.fraction_b), (1.0, 0.0));
    }
}
