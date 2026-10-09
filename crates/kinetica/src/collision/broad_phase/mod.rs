//! Broad phase: quickly finds pairs of colliders whose bounding boxes overlap.
//!
//! Each collider is stored in a [`DynamicTree`] with a *fat* bounding box, enlarged by
//! [`AABB_MARGIN`]. A collider is only re-inserted when it leaves its fat box, and only those
//! colliders are used to look for new pairs.

mod dynamic_tree;

use dynamic_tree::DynamicTree;
pub(crate) use dynamic_tree::ProxyId;

use crate::collider::ColliderHandle;
use crate::constants::AABB_MARGIN;
use crate::math::{Aabb, Vec2};

/// A pair of colliders whose fat boxes overlap, with the smaller handle first.
pub(crate) type CandidatePair = (ColliderHandle, ColliderHandle);

#[derive(Debug, Clone, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct BroadPhase {
    tree: DynamicTree<ColliderHandle>,
    /// Proxies created or re-inserted since the last search for pairs.
    moved: Vec<ProxyId>,
}

impl BroadPhase {
    pub(crate) fn create_proxy(&mut self, aabb: Aabb, collider: ColliderHandle) -> ProxyId {
        let proxy = self.tree.create_proxy(aabb.expanded(AABB_MARGIN), collider);
        self.moved.push(proxy);
        proxy
    }

    pub(crate) fn destroy_proxy(&mut self, proxy: ProxyId) {
        self.tree.destroy_proxy(proxy);
        self.moved.retain(|&moved| moved != proxy);
    }

    /// Updates the proxy for a collider's new tight box.
    pub(crate) fn update_proxy(&mut self, proxy: ProxyId, aabb: Aabb) {
        if !self.tree.aabb(proxy).contains(&aabb) {
            self.tree.move_proxy(proxy, aabb.expanded(AABB_MARGIN));
            self.moved.push(proxy);
        }
    }

    /// Calls `visit` for every collider whose fat box overlaps `aabb`.
    pub(crate) fn query(&self, aabb: &Aabb, mut visit: impl FnMut(ColliderHandle)) {
        self.tree.query(aabb, |_, collider| visit(collider));
    }

    /// Visits colliders whose fat box the segment crosses; see [`DynamicTree::ray_cast`].
    pub(crate) fn ray_cast(
        &self,
        origin: Vec2,
        translation: Vec2,
        max_fraction: f32,
        visit: impl FnMut(ColliderHandle, f32) -> f32,
    ) {
        self.tree.ray_cast(origin, translation, max_fraction, visit);
    }

    /// Makes the next search for pairs look at this proxy again, even if it did not move.
    pub(crate) fn touch_proxy(&mut self, proxy: ProxyId) {
        self.moved.push(proxy);
    }

    pub(crate) fn fat_aabb(&self, proxy: ProxyId) -> Aabb {
        self.tree.aabb(proxy)
    }

    /// Returns every overlapping pair involving a proxy that moved since the last call, sorted
    /// and without duplicates. Pairs found by earlier calls may be returned again.
    pub(crate) fn find_new_pairs(&mut self) -> Vec<CandidatePair> {
        let mut pairs = Vec::new();
        for &proxy in &self.moved {
            let collider = self.tree.data(proxy);
            self.tree.query(&self.tree.aabb(proxy), |other_proxy, other| {
                if other_proxy != proxy {
                    pairs.push(if collider < other { (collider, other) } else { (other, collider) });
                }
            });
        }
        self.moved.clear();
        pairs.sort_unstable();
        pairs.dedup();
        pairs
    }
}
