use crate::math::Vec2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AABB {
    pub min: Vec2,
    pub max: Vec2,
}

impl AABB {
    pub fn new(center: Vec2, half_extents: Vec2) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    pub fn center(&self) -> Vec2 {
        (self.min + self.max) / 2.0
    }

    pub fn half_extents(&self) -> Vec2 {
        (self.max - self.min) / 2.0
    }
}

pub fn intersects(a: &AABB, b: &AABB) -> bool {
    a.min.x <= b.max.x && a.max.x >= b.min.x
        && a.min.y <= b.max.y && a.max.y >= b.min.y
}

pub fn merge(a: &AABB, b: &AABB) -> AABB {
    AABB {
        min: a.min.min(b.min),
        max: a.max.max(b.max),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_intersects() {
        let a = AABB::new(Vec2::ZERO, Vec2::new(5.0, 5.0));
        let b = AABB::new(Vec2::new(12.0, 0.0), Vec2::new(5.0, 5.0));
        let c = AABB::new(Vec2::new(4.0, 0.0), Vec2::new(5.0, 5.0));

        assert!(!intersects(&a, &b));
        assert!(intersects(&a, &c));
    }
}
