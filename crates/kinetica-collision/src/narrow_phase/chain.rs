//! Contacts with chain segments: one-sided, and smooth across the joints between segments.
//!
//! A shape sliding along a chain touches the end of one segment and the start of the next at
//! the same time. Against the next segment alone, the contact could come from its starting
//! corner and push sideways, making the shape catch on a perfectly flat seam. Each segment
//! therefore only accepts contact normals that its neighbors make possible: at a flat or
//! hollow joint only its own face normal, at a bump the normals between its face and the
//! neighbor's face.

use super::circle::collide_rounded_circle;
use super::convex::{Rounded, clip_segment, collide_rounded, find_incident_edge, incident_points};
use crate::constants::SPECULATIVE_DISTANCE;
use crate::manifold::{Manifold, ManifoldPoint};
use crate::math::{Transform, Vec2};
use crate::shape::{ChainSegment, Shape};

/// Tolerance on the direction of normals, as the sine of a small angle.
const NORMAL_TOLERANCE: f32 = 0.005;

/// Marks feature ids of points forced onto the segment face.
const FACE_FEATURE: u32 = 2 << 24;

/// Contact between a chain segment (A) and any non-chain shape (B).
pub(super) fn collide_chain_segment(
    chain: &ChainSegment,
    transform: &Transform,
    other: &Shape,
    other_transform: &Transform,
) -> Manifold {
    let segment = chain.segment();
    let start = transform.transform_point(segment.start());
    let end = transform.transform_point(segment.end());
    let core = Rounded::two_sided(start, end, 0.0);
    let geometry = Geometry::new(
        start,
        end,
        chain.previous().map(|point| transform.transform_point(point)),
        chain.next().map(|point| transform.transform_point(point)),
    );

    let (manifold, other_center, other_shape) = if let Shape::Circle(circle) = other {
        let center = other_transform.transform_point(circle.center());
        (collide_rounded_circle(&core, circle, other_transform), center, None)
    } else {
        let Some(rounded) = super::rounded(other, other_transform) else { return Manifold::default() };
        (collide_rounded(&core, &rounded), rounded.center(), Some(rounded))
    };
    // One-sided: shapes behind the chain go through it.
    if manifold.is_empty() || geometry.normal.dot(other_center - start) < 0.0 {
        return Manifold::default();
    }
    if geometry.accepts(manifold.normal()) {
        return manifold;
    }
    match other_shape {
        // A circle only gets a refused normal near an end, where the neighbor touches it.
        None => Manifold::default(),
        Some(rounded) => face_contact(&geometry, &rounded),
    }
}

/// The segment in world space, with what its neighbors allow.
struct Geometry {
    start: Vec2,
    end: Vec2,
    tangent: Vec2,
    /// Front side: on the left of the direction from `start` to `end`.
    normal: Vec2,
    /// How far contact normals may lean towards the start, and towards the end, measured along
    /// the tangent.
    start_lean: f32,
    end_lean: f32,
}

impl Geometry {
    fn new(start: Vec2, end: Vec2, previous: Option<Vec2>, next: Option<Vec2>) -> Self {
        let tangent = (end - start).normalize();
        let normal = tangent.perp();
        // A free end accepts any normal on the front side. A neighbor turning away (a bump)
        // lets normals lean as far as its own face normal; a flat or hollow joint, not at all.
        let start_lean = previous.map_or(1.0, |previous| {
            let neighbor = (start - previous).normalize();
            if neighbor.perp_dot(tangent) < 0.0 { -neighbor.perp().dot(tangent) } else { 0.0 }
        });
        let end_lean = next.map_or(1.0, |next| {
            let neighbor = (next - end).normalize();
            if tangent.perp_dot(neighbor) < 0.0 { neighbor.perp().dot(tangent) } else { 0.0 }
        });
        Self { start, end, tangent, normal, start_lean, end_lean }
    }

    /// Whether a contact normal (from the segment to the other shape) is possible here.
    fn accepts(&self, normal: Vec2) -> bool {
        let lean = normal.dot(self.tangent);
        normal.dot(self.normal) >= 0.0
            && -lean <= self.start_lean + NORMAL_TOLERANCE
            && lean <= self.end_lean + NORMAL_TOLERANCE
    }
}

/// Contact with the segment face itself: the other shape's edge facing the segment, clipped to
/// the segment ends.
fn face_contact(geometry: &Geometry, other: &Rounded) -> Manifold {
    let Geometry { start, end, tangent, normal, .. } = *geometry;
    let incident = incident_points(other, find_incident_edge(normal, other));
    let Some(clipped) = clip_segment(incident, -tangent, -tangent.dot(start), 0) else {
        return Manifold::default();
    };
    let Some(clipped) = clip_segment(clipped, tangent, tangent.dot(end), 1) else {
        return Manifold::default();
    };

    let mut manifold = Manifold::new(normal);
    for point in clipped {
        let core_separation = normal.dot(point.position - start);
        let separation = core_separation - other.radius();
        if separation <= SPECULATIVE_DISTANCE {
            // Halfway between the segment and the other surface.
            let position = point.position - f32::midpoint(other.radius(), core_separation) * normal;
            manifold.push(ManifoldPoint::new(position, separation, FACE_FEATURE | u32::from(point.feature)));
        }
    }
    manifold
}

#[cfg(test)]
mod tests {
    use crate::math::{Rot, vec2};
    use crate::shape::{Circle, Polygon, Segment};

    use super::*;

    fn at(x: f32, y: f32) -> Transform {
        Transform::new(vec2(x, y), Rot::IDENTITY)
    }

    /// The segment from (0, 0) to (1, 0) of a flat chain extending on both sides.
    fn flat_link() -> ChainSegment {
        ChainSegment::new(Segment::new(vec2(0.0, 0.0), vec2(1.0, 0.0)), Some(vec2(-1.0, 0.0)), Some(vec2(2.0, 0.0)))
    }

    #[test]
    fn box_across_a_flat_seam_gets_the_face_normal() {
        // The box overlaps the seam at x = 1 and sinks 1 cm: against the second segment alone,
        // the box's side face would be the shallowest axis.
        let next = ChainSegment::new(Segment::new(vec2(1.0, 0.0), vec2(2.0, 0.0)), Some(vec2(0.0, 0.0)), None);
        let block = Shape::from(Polygon::rectangle(1.0, 1.0));
        let manifold = collide_chain_segment(&next, &Transform::IDENTITY, &block, &at(0.505, 0.49));
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-5), "normal {}", manifold.normal());
        // The part of the box above this segment: from x = 1 to 1.005.
        let [first, second] = manifold.points() else { panic!("expected two points, got {manifold:?}") };
        assert!((first.position().x - 1.0).abs() < 1e-4 && (second.position().x - 1.005).abs() < 1e-4);
    }

    #[test]
    fn circle_over_a_flat_seam_touches_one_segment_only() {
        let circle = Shape::from(Circle::new(0.5));
        // Just past the end of the segment: the next one handles it.
        assert!(collide_chain_segment(&flat_link(), &Transform::IDENTITY, &circle, &at(1.05, 0.49)).is_empty());
        let manifold = collide_chain_segment(&flat_link(), &Transform::IDENTITY, &circle, &at(0.95, 0.49));
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-5));
    }

    #[test]
    fn shapes_behind_the_chain_pass_through() {
        let circle = Shape::from(Circle::new(0.5));
        assert!(collide_chain_segment(&flat_link(), &Transform::IDENTITY, &circle, &at(0.5, -0.4)).is_empty());
    }

    #[test]
    fn bump_corner_accepts_leaning_normals() {
        // A hill top at (1, 0): the next segment goes down.
        let up = ChainSegment::new(Segment::new(vec2(0.0, -1.0), vec2(1.0, 0.0)), None, Some(vec2(2.0, -1.0)));
        let circle = Shape::from(Circle::new(0.5));
        // Right above the top: the normal is vertical, between both faces.
        let manifold = collide_chain_segment(&up, &Transform::IDENTITY, &circle, &at(1.0, 0.49));
        assert!(manifold.normal().abs_diff_eq(Vec2::Y, 1e-5), "normal {}", manifold.normal());
    }

    #[test]
    fn free_ends_behave_like_a_segment() {
        let alone = ChainSegment::new(Segment::new(vec2(0.0, 0.0), vec2(1.0, 0.0)), None, None);
        let circle = Shape::from(Circle::new(0.5));
        let manifold = collide_chain_segment(&alone, &Transform::IDENTITY, &circle, &at(1.3, 0.3));
        let expected = vec2(0.3, 0.3).normalize();
        assert!(manifold.normal().abs_diff_eq(expected, 1e-5));
    }
}
