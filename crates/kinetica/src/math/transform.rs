use glam::Vec2;

use super::Rot;

/// A rigid transform: a rotation followed by a translation.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Transform {
    /// Translation, in world space.
    pub position: Vec2,
    /// Rotation around the origin of the local frame.
    pub rotation: Rot,
}

impl Transform {
    /// The transform that leaves every point unchanged.
    pub const IDENTITY: Self = Self { position: Vec2::ZERO, rotation: Rot::IDENTITY };

    /// Creates a transform from a position and a rotation.
    #[must_use]
    pub fn new(position: Vec2, rotation: Rot) -> Self {
        Self { position, rotation }
    }

    /// Maps a point from local space to world space.
    #[must_use]
    pub fn transform_point(&self, local_point: Vec2) -> Vec2 {
        self.rotation.rotate(local_point) + self.position
    }

    /// Maps a point from world space to local space.
    #[must_use]
    pub fn inverse_transform_point(&self, world_point: Vec2) -> Vec2 {
        self.rotation.inverse_rotate(world_point - self.position)
    }
}

#[cfg(test)]
mod tests {
    use glam::vec2;

    use super::*;

    #[test]
    fn inverse_transform_undoes_transform() {
        let transform = Transform::new(vec2(1.0, 2.0), Rot::from_angle(1.2));
        let point = vec2(-0.5, 4.0);
        let round_trip = transform.inverse_transform_point(transform.transform_point(point));
        assert!(round_trip.abs_diff_eq(point, 1e-5));
    }
}
