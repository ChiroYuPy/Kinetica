use glam::Vec2;

/// An axis-aligned bounding box.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Aabb {
    /// Lower corner.
    pub min: Vec2,
    /// Upper corner.
    pub max: Vec2,
}

impl Aabb {
    /// Creates a box from its lower and upper corners.
    #[must_use]
    pub fn new(min: Vec2, max: Vec2) -> Self {
        debug_assert!(min.cmple(max).all(), "Aabb min must not exceed max");
        Self { min, max }
    }

    /// Returns the smallest box containing every point, or `None` if there are no points.
    #[must_use]
    pub fn from_points(points: impl IntoIterator<Item = Vec2>) -> Option<Self> {
        let mut points = points.into_iter();
        let first = points.next()?;
        Some(points.fold(Self::new(first, first), |aabb, p| Self::new(aabb.min.min(p), aabb.max.max(p))))
    }

    /// Center of the box.
    #[must_use]
    pub fn center(&self) -> Vec2 {
        0.5 * (self.min + self.max)
    }

    /// Half of the box size along each axis.
    #[must_use]
    pub fn half_extents(&self) -> Vec2 {
        0.5 * (self.max - self.min)
    }

    /// Perimeter of the box, used as its cost when building bounding volume hierarchies.
    #[must_use]
    pub fn perimeter(&self) -> f32 {
        let size = self.max - self.min;
        2.0 * (size.x + size.y)
    }

    /// Returns the box grown by `margin` on every side.
    #[must_use]
    pub fn expanded(&self, margin: f32) -> Self {
        Self::new(self.min - margin, self.max + margin)
    }

    /// Smallest box containing both boxes.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        Self::new(self.min.min(other.min), self.max.max(other.max))
    }

    /// Returns `true` if the two boxes overlap or touch.
    #[must_use]
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.cmple(other.max).all() && other.min.cmple(self.max).all()
    }

    /// Returns the fraction of `translation` at which the segment from `origin` to
    /// `origin + max_fraction * translation` enters the box, or `None` if it misses it.
    /// A segment starting inside the box enters at `0`.
    #[must_use]
    pub fn ray_entry(&self, origin: Vec2, translation: Vec2, max_fraction: f32) -> Option<f32> {
        let mut entry = 0.0_f32;
        let mut exit = max_fraction;
        // Slab method: intersect the parameter ranges spent between each pair of parallel sides.
        for axis in 0..2 {
            let (start, direction) = (origin[axis], translation[axis]);
            let (low, high) = (self.min[axis], self.max[axis]);
            if direction == 0.0 {
                if start < low || start > high {
                    return None;
                }
            } else {
                let (near, far) = ((low - start) / direction, (high - start) / direction);
                entry = entry.max(near.min(far));
                exit = exit.min(near.max(far));
                if entry > exit {
                    return None;
                }
            }
        }
        Some(entry)
    }

    /// Returns `true` if `other` lies entirely inside this box.
    #[must_use]
    pub fn contains(&self, other: &Self) -> bool {
        self.min.cmple(other.min).all() && other.max.cmple(self.max).all()
    }

    /// Returns `true` if the point is inside the box or on its boundary.
    #[must_use]
    pub fn contains_point(&self, point: Vec2) -> bool {
        self.min.cmple(point).all() && point.cmple(self.max).all()
    }
}

#[cfg(test)]
mod tests {
    use glam::vec2;

    use super::*;

    #[test]
    fn from_points_bounds_every_point() {
        let points = [vec2(1.0, -2.0), vec2(-3.0, 0.5), vec2(2.0, 4.0)];
        let aabb = Aabb::from_points(points).unwrap();
        assert_eq!(aabb, Aabb::new(vec2(-3.0, -2.0), vec2(2.0, 4.0)));
        assert!(Aabb::from_points([]).is_none());
    }

    #[test]
    fn ray_entry_finds_where_segments_enter() {
        let aabb = Aabb::new(Vec2::ZERO, Vec2::ONE);
        assert_eq!(aabb.ray_entry(vec2(-1.0, 0.5), vec2(4.0, 0.0), 1.0), Some(0.25));
        assert_eq!(aabb.ray_entry(vec2(0.5, 0.5), vec2(4.0, 0.0), 1.0), Some(0.0), "starts inside");
        assert_eq!(aabb.ray_entry(vec2(-1.0, 0.5), vec2(4.0, 0.0), 0.2), None, "too short");
        assert_eq!(aabb.ray_entry(vec2(-1.0, 2.0), vec2(4.0, 0.0), 1.0), None, "passes above");
        assert_eq!(aabb.ray_entry(vec2(2.0, 0.5), vec2(4.0, 0.0), 1.0), None, "points away");
    }

    #[test]
    fn overlap_includes_touching_boxes() {
        let a = Aabb::new(Vec2::ZERO, Vec2::ONE);
        let touching = Aabb::new(vec2(1.0, 0.0), vec2(2.0, 1.0));
        let apart = Aabb::new(vec2(1.5, 0.0), vec2(2.0, 1.0));
        assert!(a.overlaps(&touching));
        assert!(!a.overlaps(&apart));
    }
}
