//! Narrow phase: exact contact computation between two shapes.

mod circle;
mod polygon;

use super::Manifold;
use crate::constants::LINEAR_SLOP;
use crate::math::{Transform, Vec2};
use crate::shape::Shape;

/// Computes the contact between two placed shapes.
///
/// Points are reported up to a small distance before the shapes actually touch, with a positive
/// [`separation`](super::ManifoldPoint::separation). The normal points from `shape_a` to
/// `shape_b`.
#[must_use]
pub fn collide(shape_a: &Shape, transform_a: &Transform, shape_b: &Shape, transform_b: &Transform) -> Manifold {
    match (shape_a, shape_b) {
        (Shape::Circle(a), Shape::Circle(b)) => circle::collide_circles(a, transform_a, b, transform_b),
        (Shape::Polygon(a), Shape::Circle(b)) => circle::collide_polygon_circle(a, transform_a, b, transform_b),
        (Shape::Circle(a), Shape::Polygon(b)) => {
            circle::collide_polygon_circle(b, transform_b, a, transform_a).flipped()
        }
        (Shape::Polygon(a), Shape::Polygon(b)) => polygon::collide_polygons(a, transform_a, b, transform_b),
    }
}

/// A lower bound of the distance between two placed shapes, negative when they overlap.
///
/// Exact when a circle is involved; see [`polygon::separation_lower_bound`] otherwise. It is
/// negative exactly when the shapes overlap.
pub(crate) fn separation_lower_bound(
    shape_a: &Shape,
    transform_a: &Transform,
    shape_b: &Shape,
    transform_b: &Transform,
) -> f32 {
    match (shape_a, shape_b) {
        (Shape::Circle(a), Shape::Circle(b)) => {
            transform_a.transform_point(a.center()).distance(transform_b.transform_point(b.center()))
                - a.radius()
                - b.radius()
        }
        (Shape::Polygon(a), Shape::Circle(b)) => circle::polygon_circle_separation(a, transform_a, b, transform_b),
        (Shape::Circle(a), Shape::Polygon(b)) => circle::polygon_circle_separation(b, transform_b, a, transform_a),
        (Shape::Polygon(a), Shape::Polygon(b)) => polygon::separation_lower_bound(a, transform_a, b, transform_b),
    }
}

/// Shapes closer than this count as touching for a time of impact.
const TIME_OF_IMPACT_TOLERANCE: f32 = 0.25 * LINEAR_SLOP;
const MAX_TIME_OF_IMPACT_ITERATIONS: usize = 30;

/// Fraction of `translation` at which `moving`, sliding without rotating from `start`, first
/// touches `target`; `None` if it does not within `max_fraction`. Shapes already overlapping
/// touch at `0`.
///
/// Uses conservative advancement: the shape can safely move by the distance lower bound, since
/// a translation of length `d` cannot close a gap by more than `d`.
pub(crate) fn time_of_impact(
    moving: &Shape,
    start: &Transform,
    translation: Vec2,
    target: &Shape,
    target_transform: &Transform,
    max_fraction: f32,
) -> Option<f32> {
    let length = translation.length();
    let mut fraction = 0.0;
    for _ in 0..MAX_TIME_OF_IMPACT_ITERATIONS {
        let placed = Transform::new(start.position + fraction * translation, start.rotation);
        let gap = separation_lower_bound(target, target_transform, moving, &placed);
        if gap <= TIME_OF_IMPACT_TOLERANCE {
            return Some(fraction);
        }
        if length == 0.0 {
            return None;
        }
        fraction += gap / length;
        if fraction > max_fraction {
            return None;
        }
    }
    // Still approaching after many steps: the current fraction is safe, just early.
    Some(fraction)
}
