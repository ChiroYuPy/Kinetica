//! Symmetric 2×2 matrices, the effective masses of two-dimensional constraints.

use crate::math::{Vec2, vec2};

/// A symmetric 2×2 matrix.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SymmetricMatrix {
    pub(crate) xx: f32,
    pub(crate) xy: f32,
    pub(crate) yy: f32,
}

impl SymmetricMatrix {
    /// Solves `self * x = b`; returns zero when the matrix is singular.
    pub(crate) fn solve(&self, b: Vec2) -> Vec2 {
        let determinant = self.xx * self.yy - self.xy * self.xy;
        if determinant == 0.0 {
            return Vec2::ZERO;
        }
        let inverse = 1.0 / determinant;
        vec2(inverse * (self.yy * b.x - self.xy * b.y), inverse * (self.xx * b.y - self.xy * b.x))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solve_inverts_multiplication() {
        let matrix = SymmetricMatrix { xx: 3.0, xy: 1.0, yy: 2.0 };
        let x = vec2(0.5, -1.5);
        let b = vec2(matrix.xx * x.x + matrix.xy * x.y, matrix.xy * x.x + matrix.yy * x.y);
        assert!(matrix.solve(b).abs_diff_eq(x, 1e-5));
        assert_eq!(SymmetricMatrix { xx: 1.0, xy: 1.0, yy: 1.0 }.solve(b), Vec2::ZERO);
    }
}
