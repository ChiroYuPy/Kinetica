//! Narrow phase: exact contact computation between two shapes.

mod circle;
mod polygon;

use super::Manifold;
use crate::math::Transform;
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
