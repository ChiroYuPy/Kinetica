//! Spatial queries: what is at a point, in an area, along a ray, or in the way of a moving shape.
//!
//! All queries take a [`QueryFilter`] to skip some colliders, for example the body casting the
//! ray. Results come in a deterministic order.

use crate::body::{BodyHandle, BodyType};
use crate::collider::{Collider, ColliderHandle};
use crate::collision::{ManifoldPoint, collide, separation_lower_bound, time_of_impact};
use crate::math::{Aabb, Transform, Vec2};
use crate::shape::Shape;
use crate::world::World;

/// Selects which colliders a query considers. The default accepts every collider except
/// sensors.
///
/// ```
/// use kinetica::prelude::*;
///
/// # let mut world = World::new();
/// # let player = world.add_body(BodyDesc::dynamic());
/// let filter = QueryFilter::new().exclude_body(player).exclude_dynamic();
/// let ground_below = world.cast_ray(vec2(0.0, 0.0), vec2(0.0, -2.0), &filter);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools, reason = "independent on/off switches")]
pub struct QueryFilter {
    /// Only accepts colliders belonging to one of these layers, see
    /// [`CollisionLayers::memberships`](crate::CollisionLayers::memberships).
    pub layers: u32,
    /// Skips every collider of this body.
    pub excluded_body: Option<BodyHandle>,
    /// Skips colliders of fixed bodies.
    pub skip_fixed: bool,
    /// Skips colliders of kinematic bodies.
    pub skip_kinematic: bool,
    /// Skips colliders of dynamic bodies.
    pub skip_dynamic: bool,
    /// Accepts sensors too.
    pub include_sensors: bool,
}

impl Default for QueryFilter {
    fn default() -> Self {
        Self {
            layers: u32::MAX,
            excluded_body: None,
            skip_fixed: false,
            skip_kinematic: false,
            skip_dynamic: false,
            include_sensors: false,
        }
    }
}

impl QueryFilter {
    /// A filter accepting every collider except sensors.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Only accepts colliders belonging to one of the `layers`.
    #[must_use]
    pub fn layers(mut self, layers: u32) -> Self {
        self.layers = layers;
        self
    }

    /// Accepts sensors too.
    #[must_use]
    pub fn include_sensors(mut self) -> Self {
        self.include_sensors = true;
        self
    }

    /// Skips every collider of `body`.
    #[must_use]
    pub fn exclude_body(mut self, body: BodyHandle) -> Self {
        self.excluded_body = Some(body);
        self
    }

    /// Skips colliders of fixed bodies.
    #[must_use]
    pub fn exclude_fixed(mut self) -> Self {
        self.skip_fixed = true;
        self
    }

    /// Skips colliders of kinematic bodies.
    #[must_use]
    pub fn exclude_kinematic(mut self) -> Self {
        self.skip_kinematic = true;
        self
    }

    /// Skips colliders of dynamic bodies.
    #[must_use]
    pub fn exclude_dynamic(mut self) -> Self {
        self.skip_dynamic = true;
        self
    }

    fn accepts(&self, world: &World, collider: &Collider) -> bool {
        if self.excluded_body == Some(collider.body())
            || collider.layers().memberships & self.layers == 0
            || (collider.is_sensor() && !self.include_sensors)
        {
            return false;
        }
        match world.body(collider.body()).map(crate::Body::body_type) {
            Some(BodyType::Fixed) => !self.skip_fixed,
            Some(BodyType::Kinematic) => !self.skip_kinematic,
            Some(BodyType::Dynamic) => !self.skip_dynamic,
            None => false,
        }
    }
}

/// Where a ray hit a collider.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RayHit {
    /// The collider hit.
    pub collider: ColliderHandle,
    /// Its body.
    pub body: BodyHandle,
    /// Hit point, in world space.
    pub point: Vec2,
    /// Surface normal at the hit point, pointing out of the collider.
    pub normal: Vec2,
    /// Fraction of the ray translation travelled before the hit, between 0 and 1.
    pub fraction: f32,
}

/// Where a moving shape first touches a collider.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapeHit {
    /// The collider hit.
    pub collider: ColliderHandle,
    /// Its body.
    pub body: BodyHandle,
    /// Contact point, in world space.
    pub point: Vec2,
    /// Contact normal, pointing out of the collider towards the moving shape.
    pub normal: Vec2,
    /// Fraction of the translation travelled before touching, between 0 and 1. Zero when the
    /// shape starts overlapping the collider.
    pub fraction: f32,
}

impl World {
    /// Casts a ray from `origin` to `origin + translation` and returns the closest hit.
    ///
    /// Colliders containing `origin` are not hit.
    #[must_use]
    pub fn cast_ray(&self, origin: Vec2, translation: Vec2, filter: &QueryFilter) -> Option<RayHit> {
        let mut closest: Option<RayHit> = None;
        self.broad_phase().ray_cast(origin, translation, 1.0, |handle, max_fraction| {
            let Some(hit) = self.ray_hit(handle, origin, translation, max_fraction, filter) else {
                return max_fraction;
            };
            // Equal fractions: keep the smaller handle, whatever the tree visiting order.
            let better = closest.is_none_or(|best| (hit.fraction, hit.collider) < (best.fraction, best.collider));
            if better {
                closest = Some(hit);
            }
            hit.fraction
        });
        closest
    }

    /// Casts a ray from `origin` to `origin + translation` and returns every hit, nearest first.
    #[must_use]
    pub fn cast_ray_all(&self, origin: Vec2, translation: Vec2, filter: &QueryFilter) -> Vec<RayHit> {
        let mut hits = Vec::new();
        self.broad_phase().ray_cast(origin, translation, 1.0, |handle, max_fraction| {
            hits.extend(self.ray_hit(handle, origin, translation, max_fraction, filter));
            max_fraction
        });
        hits.sort_by(|a, b| a.fraction.total_cmp(&b.fraction).then(a.collider.cmp(&b.collider)));
        hits
    }

    /// Moves `shape` from `transform` along `translation`, without rotating it, and returns the
    /// first collider it touches.
    #[must_use]
    pub fn cast_shape(
        &self,
        shape: &Shape,
        transform: &Transform,
        translation: Vec2,
        filter: &QueryFilter,
    ) -> Option<ShapeHit> {
        let end = Transform::new(transform.position + translation, transform.rotation);
        let swept = shape.compute_aabb(transform).union(&shape.compute_aabb(&end));

        let mut closest: Option<(f32, ColliderHandle)> = None;
        for handle in self.colliders_in_aabb(&swept, filter) {
            let (collider, body) = self.collider_and_body_transform(handle)?;
            let max_fraction = closest.map_or(1.0, |(fraction, _)| fraction);
            if let Some(fraction) = time_of_impact(shape, transform, translation, collider.shape(), body, max_fraction)
                && closest.is_none_or(|best| (fraction, handle) < best)
            {
                closest = Some((fraction, handle));
            }
        }

        let (fraction, handle) = closest?;
        let (collider, body_transform) = self.collider_and_body_transform(handle)?;
        let placed = Transform::new(transform.position + fraction * translation, transform.rotation);
        let manifold = collide(collider.shape(), body_transform, shape, &placed);
        let points = manifold.points();
        let (point, normal) = if points.is_empty() {
            // Not expected at the time of impact; fall back on the motion direction.
            (placed.position, -translation.normalize_or_zero())
        } else {
            let point = points.iter().map(ManifoldPoint::position).sum::<Vec2>() / points.len() as f32;
            (point, manifold.normal())
        };
        Some(ShapeHit { collider: handle, body: collider.body(), point, normal, fraction })
    }

    /// Colliders containing the world point `point`.
    #[must_use]
    pub fn colliders_at_point(&self, point: Vec2, filter: &QueryFilter) -> Vec<ColliderHandle> {
        self.collect_candidates(&Aabb::new(point, point), filter, |collider, body_transform| {
            collider.shape().contains_point(body_transform, point)
        })
    }

    /// Colliders whose bounding box overlaps `aabb`.
    ///
    /// This is a cheap, approximate test: a collider may be listed while its actual shape does
    /// not reach into the box. Use [`colliders_overlapping`](Self::colliders_overlapping) for
    /// an exact test.
    #[must_use]
    pub fn colliders_in_aabb(&self, aabb: &Aabb, filter: &QueryFilter) -> Vec<ColliderHandle> {
        self.collect_candidates(aabb, filter, |collider, _| collider.aabb().overlaps(aabb))
    }

    /// Colliders overlapping `shape` placed at `transform`. Merely touching does not count.
    #[must_use]
    pub fn colliders_overlapping(
        &self,
        shape: &Shape,
        transform: &Transform,
        filter: &QueryFilter,
    ) -> Vec<ColliderHandle> {
        self.collect_candidates(&shape.compute_aabb(transform), filter, |collider, body_transform| {
            separation_lower_bound(collider.shape(), body_transform, shape, transform) < 0.0
        })
    }

    /// Runs `keep` on every accepted collider whose fat box overlaps `aabb`, sorted by handle.
    fn collect_candidates(
        &self,
        aabb: &Aabb,
        filter: &QueryFilter,
        mut keep: impl FnMut(&Collider, &Transform) -> bool,
    ) -> Vec<ColliderHandle> {
        let mut found = Vec::new();
        self.broad_phase().query(aabb, |handle| {
            if let Some((collider, body_transform)) = self.collider_and_body_transform(handle)
                && filter.accepts(self, collider)
                && keep(collider, body_transform)
            {
                found.push(handle);
            }
        });
        found.sort_unstable();
        found
    }

    fn ray_hit(
        &self,
        handle: ColliderHandle,
        origin: Vec2,
        translation: Vec2,
        max_fraction: f32,
        filter: &QueryFilter,
    ) -> Option<RayHit> {
        let (collider, body_transform) = self.collider_and_body_transform(handle)?;
        if !filter.accepts(self, collider) {
            return None;
        }
        let intersection = collider.shape().cast_ray(body_transform, origin, translation, max_fraction)?;
        Some(RayHit {
            collider: handle,
            body: collider.body(),
            point: origin + intersection.fraction * translation,
            normal: intersection.normal,
            fraction: intersection.fraction,
        })
    }

    fn collider_and_body_transform(&self, handle: ColliderHandle) -> Option<(&Collider, &Transform)> {
        let collider = self.collider(handle)?;
        let body = self.body(collider.body())?;
        Some((collider, body.transform()))
    }
}
