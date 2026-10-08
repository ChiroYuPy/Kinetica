use std::f32::consts::PI;

use super::MassProperties;
use super::circle::cast_ray_at_disc;
use super::segment::closest_point_on_segment;
use crate::constants::LINEAR_SLOP;
use crate::math::{Aabb, Transform, Vec2};

/// A solid capsule: every point within `radius` of the segment from `start` to `end`.
///
/// Its rounded ends slide smoothly over edges and steps, which makes it the usual shape for
/// characters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Capsule {
    start: Vec2,
    end: Vec2,
    radius: f32,
}

impl Capsule {
    /// Creates a capsule around the segment from `start` to `end`, in the body's local frame.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not strictly positive and finite, or if the two points are not
    /// finite or are closer than 5 mm (use a circle instead).
    #[must_use]
    pub fn new(start: Vec2, end: Vec2, radius: f32) -> Self {
        assert!(radius.is_finite() && radius > 0.0, "capsule radius must be positive, got {radius}");
        assert!(start.is_finite() && end.is_finite(), "capsule points must be finite");
        assert!(start.distance(end) > LINEAR_SLOP, "capsule points are too close, use a circle");
        Self { start, end, radius }
    }

    /// A vertical capsule centered on the body origin, `2 * (half_length + radius)` tall.
    ///
    /// # Panics
    ///
    /// See [`Capsule::new`].
    #[must_use]
    pub fn vertical(half_length: f32, radius: f32) -> Self {
        Self::new(Vec2::new(0.0, -half_length), Vec2::new(0.0, half_length), radius)
    }

    /// First end of the inner segment, in the local frame.
    #[must_use]
    pub fn start(&self) -> Vec2 {
        self.start
    }

    /// Second end of the inner segment, in the local frame.
    #[must_use]
    pub fn end(&self) -> Vec2 {
        self.end
    }

    /// Radius around the inner segment.
    #[must_use]
    pub fn radius(&self) -> f32 {
        self.radius
    }

    pub(crate) fn mass_properties(&self, density: f32) -> MassProperties {
        let radius = self.radius;
        let length = self.start.distance(self.end);
        let disc_mass = density * PI * radius * radius;
        let box_mass = density * 2.0 * radius * length;
        // Each half disc sits at an end of the box, its own centroid 4r/3π further out: the
        // parallel axis theorem is applied from that centroid to the capsule center.
        let half_disc_centroid = 4.0 * radius / (3.0 * PI);
        let half_length = 0.5 * length;
        let disc_inertia =
            disc_mass * (0.5 * radius * radius + half_length * half_length + 2.0 * half_length * half_disc_centroid);
        let box_inertia = box_mass * (4.0 * radius * radius + length * length) / 12.0;
        MassProperties {
            mass: disc_mass + box_mass,
            center: 0.5 * (self.start + self.end),
            inertia: disc_inertia + box_inertia,
        }
    }

    pub(crate) fn contains_point(&self, local_point: Vec2) -> bool {
        closest_point_on_segment(self.start, self.end, local_point).distance(local_point) <= self.radius
    }

    /// Ray cast in the local frame; returns the fraction and the local normal.
    pub(crate) fn cast_ray(&self, origin: Vec2, translation: Vec2, max_fraction: f32) -> Option<(f32, Vec2)> {
        if self.contains_point(origin) {
            return None;
        }
        let axis = self.end - self.start;
        let length_squared = axis.length_squared();
        let side_normal = Vec2::new(axis.y, -axis.x).normalize();
        let mut best: Option<(f32, Vec2)> = None;
        let mut consider = |hit: Option<(f32, Vec2)>| {
            if let Some(hit) = hit
                && best.is_none_or(|best| hit.0 < best.0)
            {
                best = Some(hit);
            }
        };

        // The two flat sides.
        for normal in [side_normal, -side_normal] {
            let speed = normal.dot(translation);
            if speed >= 0.0 {
                continue;
            }
            let plane_point = self.start + self.radius * normal;
            let fraction = normal.dot(plane_point - origin) / speed;
            let along = (origin + fraction * translation - self.start).dot(axis);
            if (0.0..=max_fraction).contains(&fraction) && (0.0..=length_squared).contains(&along) {
                consider(Some((fraction, normal)));
            }
        }
        // The two round ends.
        for center in [self.start, self.end] {
            consider(cast_ray_at_disc(center, self.radius, origin, translation, max_fraction));
        }
        best
    }

    pub(crate) fn compute_aabb(&self, transform: &Transform) -> Aabb {
        let (start, end) = (transform.transform_point(self.start), transform.transform_point(self.end));
        Aabb::new(start.min(end), start.max(end)).expanded(self.radius)
    }
}
