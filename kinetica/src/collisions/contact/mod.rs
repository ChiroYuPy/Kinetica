mod circle_vs_circle;

use crate::constraints::ContactManifold;
use crate::core::{RigidBody, Shape};

pub fn test_contact(a: &RigidBody, b: &RigidBody) -> Option<ContactManifold> {
    match (&a.shape, &b.shape) {
        (Shape::Circle(r1), Shape::Circle(r2)) => circle_vs_circle::circle_vs_circle(a, b, *r1, *r2),
        _ => None,
    }
}
