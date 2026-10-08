//! Narrow phase: exact contact computation between two shapes.

mod chain;
mod circle;
mod convex;

use super::Manifold;
use crate::constants::LINEAR_SLOP;
use crate::math::{Transform, Vec2};
use crate::shape::Shape;
use convex::Rounded;

/// Computes the contact between two placed shapes.
///
/// Points are reported up to a small distance before the shapes actually touch, with a positive
/// [`separation`](super::ManifoldPoint::separation). The normal points from `shape_a` to
/// `shape_b`.
#[must_use]
pub fn collide(shape_a: &Shape, transform_a: &Transform, shape_b: &Shape, transform_b: &Transform) -> Manifold {
    match (shape_a, shape_b) {
        (Shape::ChainSegment(chain), _) => chain::collide_chain_segment(chain, transform_a, shape_b, transform_b),
        (_, Shape::ChainSegment(chain)) => {
            chain::collide_chain_segment(chain, transform_b, shape_a, transform_a).flipped()
        }
        (Shape::Circle(a), Shape::Circle(b)) => circle::collide_circles(a, transform_a, b, transform_b),
        (_, Shape::Circle(b)) => rounded(shape_a, transform_a)
            .map_or_else(Manifold::default, |a| circle::collide_rounded_circle(&a, b, transform_b)),
        (Shape::Circle(a), _) => rounded(shape_b, transform_b)
            .map_or_else(Manifold::default, |b| circle::collide_rounded_circle(&b, a, transform_a).flipped()),
        _ => match (rounded(shape_a, transform_a), rounded(shape_b, transform_b)) {
            (Some(a), Some(b)) => convex::collide_rounded(&a, &b),
            _ => Manifold::default(),
        },
    }
}

/// A lower bound of the distance between two placed shapes, negative when they overlap.
///
/// Exact when a circle is involved; see [`convex::separation_lower_bound`] otherwise. It is
/// negative exactly when the shapes overlap. Chain segments count as two-sided here.
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
        (_, Shape::Circle(b)) => rounded(shape_a, transform_a)
            .map_or(f32::INFINITY, |a| circle::rounded_circle_separation(&a, b, transform_b)),
        (Shape::Circle(a), _) => rounded(shape_b, transform_b)
            .map_or(f32::INFINITY, |b| circle::rounded_circle_separation(&b, a, transform_a)),
        _ => match (rounded(shape_a, transform_a), rounded(shape_b, transform_b)) {
            (Some(a), Some(b)) => convex::separation_lower_bound(&a, &b),
            _ => f32::INFINITY,
        },
    }
}

/// Every shape but the circle, as a rounded convex shape in world space.
fn rounded(shape: &Shape, transform: &Transform) -> Option<Rounded> {
    match shape {
        Shape::Circle(_) => None,
        Shape::Capsule(capsule) => Some(Rounded::capsule(capsule, transform)),
        Shape::Polygon(polygon) => Some(Rounded::polygon(polygon, transform)),
        Shape::Segment(segment) => Some(Rounded::segment(segment, transform)),
        Shape::ChainSegment(chain) => Some(Rounded::segment(chain.segment(), transform)),
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
