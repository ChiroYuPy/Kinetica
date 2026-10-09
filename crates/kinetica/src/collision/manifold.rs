use crate::math::Vec2;

/// Where and how deeply two shapes touch.
///
/// The normal points from the first shape (A) towards the second (B). Two convex shapes touch
/// along at most a segment, so two points are enough to describe the contact.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Manifold {
    normal: Vec2,
    points: [ManifoldPoint; 2],
    point_count: usize,
}

impl Manifold {
    pub(crate) fn new(normal: Vec2) -> Self {
        Self { normal, ..Self::default() }
    }

    pub(crate) fn push(&mut self, point: ManifoldPoint) {
        self.points[self.point_count] = point;
        self.point_count += 1;
    }

    /// Copies the solver impulses of points that persist from `previous`, matched by feature, so
    /// the solver can start from last step's solution (warm starting).
    pub(crate) fn inherit_impulses(&mut self, previous: &Self) {
        for point in &mut self.points[..self.point_count] {
            if let Some(old) = previous.points().iter().find(|old| old.feature_id == point.feature_id) {
                point.normal_impulse = old.normal_impulse;
                point.tangent_impulse = old.tangent_impulse;
            }
        }
    }

    pub(crate) fn points_mut(&mut self) -> &mut [ManifoldPoint] {
        &mut self.points[..self.point_count]
    }

    /// Returns the same contact seen from the other shape.
    pub(crate) fn flipped(self) -> Self {
        Self { normal: -self.normal, ..self }
    }

    /// Unit normal, in world space, pointing from shape A to shape B.
    #[must_use]
    pub fn normal(&self) -> Vec2 {
        self.normal
    }

    /// Contact points; empty when the shapes do not touch.
    #[must_use]
    pub fn points(&self) -> &[ManifoldPoint] {
        &self.points[..self.point_count]
    }

    /// Returns `true` when the shapes do not touch.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.point_count == 0
    }
}

/// One point of a [`Manifold`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ManifoldPoint {
    position: Vec2,
    separation: f32,
    /// Identifies the pair of features (edges, vertices) that produced this point, so it can be
    /// matched with the same point at the next step.
    feature_id: u32,
    pub(crate) normal_impulse: f32,
    pub(crate) tangent_impulse: f32,
}

impl ManifoldPoint {
    pub(crate) fn new(position: Vec2, separation: f32, feature_id: u32) -> Self {
        Self { position, separation, feature_id, normal_impulse: 0.0, tangent_impulse: 0.0 }
    }

    /// World position, halfway between the two surfaces.
    #[must_use]
    pub fn position(&self) -> Vec2 {
        self.position
    }

    /// Distance between the surfaces along the normal: negative when they overlap, positive
    /// when they are about to touch.
    #[must_use]
    pub fn separation(&self) -> f32 {
        self.separation
    }

    /// Impulse the solver applied along the normal during the last substep, in N·s. Divide by
    /// the substep duration to get a force; useful to scale impact sounds or damage.
    #[must_use]
    pub fn normal_impulse(&self) -> f32 {
        self.normal_impulse
    }

    /// Friction impulse the solver applied during the last substep, in N·s.
    #[must_use]
    pub fn tangent_impulse(&self) -> f32 {
        self.tangent_impulse
    }

    #[cfg(test)]
    pub(crate) fn feature_id(&self) -> u32 {
        self.feature_id
    }
}
