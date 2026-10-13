use super::MassProperties;
use crate::math::{Aabb, Transform, Vec2};

/// A line segment without thickness, colliding on both sides.
///
/// It has no area, hence no mass: use it for fixed geometry such as walls and floors.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Segment {
    start: Vec2,
    end: Vec2,
}

impl Segment {
    /// Creates the segment from `start` to `end`, in the body's local frame.
    ///
    /// # Panics
    ///
    /// Panics if the points are not finite or are equal.
    #[must_use]
    pub fn new(start: Vec2, end: Vec2) -> Self {
        assert!(start.is_finite() && end.is_finite(), "segment points must be finite");
        assert!(start != end, "a segment needs two different points");
        Self { start, end }
    }

    /// First end, in the local frame.
    #[must_use]
    pub fn start(&self) -> Vec2 {
        self.start
    }

    /// Second end, in the local frame.
    #[must_use]
    pub fn end(&self) -> Vec2 {
        self.end
    }

    /// Unit normal on the left of the direction from `start` to `end`.
    #[must_use]
    pub fn normal(&self) -> Vec2 {
        (self.end - self.start).perp().normalize()
    }

    pub(crate) fn mass_properties(&self) -> MassProperties {
        MassProperties { mass: 0.0, center: 0.5 * (self.start + self.end), inertia: 0.0 }
    }

    /// Ray cast in the local frame, hitting either side; returns the fraction and the normal
    /// facing the ray.
    pub(crate) fn cast_ray(&self, origin: Vec2, translation: Vec2, max_fraction: f32) -> Option<(f32, Vec2)> {
        let normal = self.normal();
        let speed = normal.dot(translation);
        if speed == 0.0 {
            return None;
        }
        let fraction = normal.dot(self.start - origin) / speed;
        let axis = self.end - self.start;
        let along = (origin + fraction * translation - self.start).dot(axis);
        if !(0.0..=max_fraction).contains(&fraction) || !(0.0..=axis.length_squared()).contains(&along) {
            return None;
        }
        Some((fraction, if speed > 0.0 { -normal } else { normal }))
    }

    pub(crate) fn compute_aabb(&self, transform: &Transform) -> Aabb {
        let (start, end) = (transform.transform_point(self.start), transform.transform_point(self.end));
        Aabb::new(start.min(end), start.max(end))
    }
}

/// One segment of a chain, as created by `World::add_chain` in `kinetica`.
///
/// Chain segments are one-sided: they only collide with shapes on the side of their
/// [`normal`](Segment::normal), so a body that ends up behind the chain can get back out. They
/// know the neighboring points of the chain, which lets shapes slide across the joints between
/// segments without catching on them.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ChainSegment {
    segment: Segment,
    previous: Option<Vec2>,
    next: Option<Vec2>,
}

impl ChainSegment {
    /// Creates a chain segment with the points before its start and after its end, if any.
    #[must_use]
    pub fn new(segment: Segment, previous: Option<Vec2>, next: Option<Vec2>) -> Self {
        Self { segment, previous, next }
    }

    /// The segment itself.
    #[must_use]
    pub fn segment(&self) -> &Segment {
        &self.segment
    }

    /// The chain point before the start of the segment, if any.
    #[must_use]
    pub fn previous(&self) -> Option<Vec2> {
        self.previous
    }

    /// The chain point after the end of the segment, if any.
    #[must_use]
    pub fn next(&self) -> Option<Vec2> {
        self.next
    }
}

/// The point of the segment from `start` to `end` closest to `point`.
pub(crate) fn closest_point_on_segment(start: Vec2, end: Vec2, point: Vec2) -> Vec2 {
    let axis = end - start;
    let t = ((point - start).dot(axis) / axis.length_squared()).clamp(0.0, 1.0);
    start + t * axis
}
