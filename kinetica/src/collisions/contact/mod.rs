mod circle_vs_circle;
mod circle_vs_rect;
mod rect_vs_rect;

use crate::constraints::ContactManifold;
use crate::core::{RigidBody, Shape};

pub fn test_contact(a: &RigidBody, b: &RigidBody) -> Option<ContactManifold> {
    match (&a.shape, &b.shape) {
        (Shape::Circle(radius1), Shape::Circle(radius2)) => {
            circle_vs_circle::test(a, b, *radius1, *radius2)
        }
        (Shape::Circle(radius), Shape::Rectangle(size)) => {
            let mut m = circle_vs_rect::test(a, b, *radius, *size)?;
            m.normal = -m.normal;
            Some(m)
        }
        (Shape::Rectangle(size), Shape::Circle(radius)) => {
            circle_vs_rect::test(b, a, *radius, *size)
        }
        (Shape::Rectangle(size1), Shape::Rectangle(size2)) => {
            rect_vs_rect::test(a, b, *size1, *size2)
        }
    }
}
