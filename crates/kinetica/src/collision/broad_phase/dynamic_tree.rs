//! Dynamic AABB tree: a self-balancing bounding volume hierarchy.
//!
//! Leaves hold user data and a bounding box; internal nodes hold the union of their two
//! children. Overlap queries skip whole subtrees whose box misses the query box, so they run in
//! roughly logarithmic time. Insertion picks the sibling that grows the tree's total perimeter
//! the least, and AVL-style rotations keep the tree roughly balanced. Balance is not strict: a
//! leaf far from all others may be paired with a tall subtree, because that is cheaper for
//! queries than pushing it deep into the tree.
//!
//! Based on the tree used in Box2D.

use crate::math::{Aabb, Vec2};

/// Identifies a leaf of the tree. It stays valid until the leaf is destroyed, even when the
/// leaf moves.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct ProxyId(NodeId);

type NodeId = u32;

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
enum NodeKind<T> {
    Leaf(T),
    Internal([NodeId; 2]),
    Free { next: Option<NodeId> },
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
struct Node<T> {
    aabb: Aabb,
    parent: Option<NodeId>,
    /// Leaves have height 0; an internal node is one higher than its tallest child.
    height: u32,
    kind: NodeKind<T>,
}

#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct DynamicTree<T> {
    nodes: Vec<Node<T>>,
    root: Option<NodeId>,
    first_free: Option<NodeId>,
}

impl<T> Default for DynamicTree<T> {
    fn default() -> Self {
        Self { nodes: Vec::new(), root: None, first_free: None }
    }
}

impl<T: Copy> DynamicTree<T> {
    pub(crate) fn create_proxy(&mut self, aabb: Aabb, data: T) -> ProxyId {
        let leaf = self.allocate(Node { aabb, parent: None, height: 0, kind: NodeKind::Leaf(data) });
        self.insert_leaf(leaf);
        ProxyId(leaf)
    }

    pub(crate) fn destroy_proxy(&mut self, proxy: ProxyId) {
        self.remove_leaf(proxy.0);
        self.free(proxy.0);
    }

    pub(crate) fn move_proxy(&mut self, proxy: ProxyId, aabb: Aabb) {
        self.remove_leaf(proxy.0);
        self.node_mut(proxy.0).aabb = aabb;
        self.insert_leaf(proxy.0);
    }

    pub(crate) fn aabb(&self, proxy: ProxyId) -> Aabb {
        self.node(proxy.0).aabb
    }

    pub(crate) fn data(&self, proxy: ProxyId) -> T {
        match self.node(proxy.0).kind {
            NodeKind::Leaf(data) => data,
            _ => unreachable!("proxy {proxy:?} is not a leaf"),
        }
    }

    /// Calls `visit` for every leaf whose box overlaps `aabb`.
    pub(crate) fn query(&self, aabb: &Aabb, mut visit: impl FnMut(ProxyId, T)) {
        let mut stack: Vec<NodeId> = self.root.into_iter().collect();
        while let Some(id) = stack.pop() {
            let node = self.node(id);
            if !node.aabb.overlaps(aabb) {
                continue;
            }
            match node.kind {
                NodeKind::Leaf(data) => visit(ProxyId(id), data),
                NodeKind::Internal(children) => stack.extend(children),
                NodeKind::Free { .. } => unreachable!("free nodes are never linked into the tree"),
            }
        }
    }

    /// Calls `visit` for every leaf whose box the segment from `origin` to
    /// `origin + max_fraction * translation` crosses. `visit` returns the new maximum fraction,
    /// which shortens the segment for the remaining leaves (return its input to keep it).
    pub(crate) fn ray_cast(
        &self,
        origin: Vec2,
        translation: Vec2,
        max_fraction: f32,
        mut visit: impl FnMut(T, f32) -> f32,
    ) {
        let mut max_fraction = max_fraction;
        let mut stack: Vec<NodeId> = self.root.into_iter().collect();
        while let Some(id) = stack.pop() {
            let node = self.node(id);
            if node.aabb.ray_entry(origin, translation, max_fraction).is_none() {
                continue;
            }
            match node.kind {
                NodeKind::Leaf(data) => max_fraction = visit(data, max_fraction),
                NodeKind::Internal(children) => stack.extend(children),
                NodeKind::Free { .. } => unreachable!("free nodes are never linked into the tree"),
            }
        }
    }

    fn insert_leaf(&mut self, leaf: NodeId) {
        let Some(root) = self.root else {
            self.root = Some(leaf);
            self.node_mut(leaf).parent = None;
            return;
        };

        let leaf_aabb = self.node(leaf).aabb;
        let sibling = self.find_best_sibling(root, &leaf_aabb);
        let old_parent = self.node(sibling).parent;
        let new_parent = self.allocate(Node {
            aabb: leaf_aabb.union(&self.node(sibling).aabb),
            parent: None,
            height: self.node(sibling).height + 1,
            kind: NodeKind::Internal([sibling, leaf]),
        });
        self.replace_child(old_parent, sibling, new_parent);
        self.node_mut(sibling).parent = Some(new_parent);
        self.node_mut(leaf).parent = Some(new_parent);

        self.rebalance_upwards(Some(new_parent));
    }

    /// Walks down from `root` towards the node that should become the new leaf's sibling.
    fn find_best_sibling(&self, root: NodeId, leaf_aabb: &Aabb) -> NodeId {
        let mut index = root;
        while let NodeKind::Internal([first, second]) = self.node(index).kind {
            let perimeter = self.node(index).aabb.perimeter();
            let combined_perimeter = self.node(index).aabb.union(leaf_aabb).perimeter();

            // Cost of pairing the leaf with this whole subtree under a new parent.
            let pair_here_cost = 2.0 * combined_perimeter;
            // Every ancestor of a deeper sibling grows as much as this node does.
            let inherited_cost = 2.0 * (combined_perimeter - perimeter);
            let descend_cost = |child: NodeId| {
                let child = self.node(child);
                let enlarged = child.aabb.union(leaf_aabb).perimeter();
                let growth = match child.kind {
                    NodeKind::Leaf(_) => enlarged,
                    _ => enlarged - child.aabb.perimeter(),
                };
                growth + inherited_cost
            };

            let (first_cost, second_cost) = (descend_cost(first), descend_cost(second));
            if pair_here_cost < first_cost && pair_here_cost < second_cost {
                break;
            }
            index = if first_cost < second_cost { first } else { second };
        }
        index
    }

    fn remove_leaf(&mut self, leaf: NodeId) {
        let Some(parent) = self.node(leaf).parent else {
            self.root = None;
            return;
        };
        let [first, second] = self.children(parent);
        let sibling = if first == leaf { second } else { first };
        let grandparent = self.node(parent).parent;

        // The sibling takes the place of the parent, which disappears.
        self.replace_child(grandparent, parent, sibling);
        self.free(parent);
        self.node_mut(leaf).parent = None;

        self.rebalance_upwards(grandparent);
    }

    /// Restores balance, bounding boxes and heights from `start` up to the root.
    fn rebalance_upwards(&mut self, start: Option<NodeId>) {
        let mut current = start;
        while let Some(id) = current {
            let id = self.balance(id);
            self.refit(id);
            current = self.node(id).parent;
        }
    }

    /// If one child of `node` is more than one level taller than the other, rotates it up.
    /// Returns the node now at the top of this subtree.
    fn balance(&mut self, node: NodeId) -> NodeId {
        let NodeKind::Internal([first, second]) = self.node(node).kind else {
            return node;
        };
        let (first_height, second_height) = (self.node(first).height, self.node(second).height);
        if first_height > second_height + 1 {
            self.rotate_up(node, 0)
        } else if second_height > first_height + 1 {
            self.rotate_up(node, 1)
        } else {
            node
        }
    }

    /// Rotates the child in `slot` above `node`. The child's taller subtree stays with it; its
    /// shorter subtree is handed to `node`.
    fn rotate_up(&mut self, node: NodeId, slot: usize) -> NodeId {
        let up = self.children(node)[slot];
        // `up` is at least two levels tall, so it is an internal node.
        let [first, second] = self.children(up);
        let (taller, shorter) =
            if self.node(first).height > self.node(second).height { (first, second) } else { (second, first) };

        let parent = self.node(node).parent;
        self.replace_child(parent, node, up);
        self.node_mut(up).kind = NodeKind::Internal([node, taller]);
        self.node_mut(node).parent = Some(up);
        self.set_child(node, slot, shorter);

        self.refit(node);
        self.refit(up);
        up
    }

    fn refit(&mut self, node: NodeId) {
        let [first, second] = self.children(node);
        let (first, second) = (self.node(first), self.node(second));
        let aabb = first.aabb.union(&second.aabb);
        let height = 1 + first.height.max(second.height);
        let node = self.node_mut(node);
        node.aabb = aabb;
        node.height = height;
    }

    fn children(&self, node: NodeId) -> [NodeId; 2] {
        match self.node(node).kind {
            NodeKind::Internal(children) => children,
            _ => unreachable!("node {node} is not an internal node"),
        }
    }

    fn set_child(&mut self, node: NodeId, slot: usize, child: NodeId) {
        let NodeKind::Internal(children) = &mut self.node_mut(node).kind else {
            unreachable!("node {node} is not an internal node");
        };
        children[slot] = child;
        self.node_mut(child).parent = Some(node);
    }

    /// Puts `new` where `old` was under `parent`, or at the root when there is no parent.
    fn replace_child(&mut self, parent: Option<NodeId>, old: NodeId, new: NodeId) {
        if let Some(parent) = parent {
            let slot = usize::from(self.children(parent)[1] == old);
            self.set_child(parent, slot, new);
        } else {
            self.root = Some(new);
            self.node_mut(new).parent = None;
        }
    }

    fn allocate(&mut self, node: Node<T>) -> NodeId {
        if let Some(id) = self.first_free {
            let NodeKind::Free { next } = self.node(id).kind else {
                unreachable!("the free list only links free nodes");
            };
            self.first_free = next;
            *self.node_mut(id) = node;
            id
        } else {
            let id = NodeId::try_from(self.nodes.len()).expect("tree holds more than u32::MAX nodes");
            self.nodes.push(node);
            id
        }
    }

    fn free(&mut self, id: NodeId) {
        let next = self.first_free;
        let node = self.node_mut(id);
        node.kind = NodeKind::Free { next };
        node.parent = None;
        self.first_free = Some(id);
    }

    fn node(&self, id: NodeId) -> &Node<T> {
        &self.nodes[id as usize]
    }

    fn node_mut(&mut self, id: NodeId) -> &mut Node<T> {
        &mut self.nodes[id as usize]
    }
}

#[cfg(test)]
mod tests {
    use glam::vec2;

    use super::*;

    impl<T: Copy> DynamicTree<T> {
        fn height(&self) -> u32 {
            self.root.map_or(0, |root| self.node(root).height)
        }

        /// Checks parent links, heights and bounding boxes; returns the leaf count.
        fn validate(&self) -> usize {
            let Some(root) = self.root else { return 0 };
            assert_eq!(self.node(root).parent, None);
            self.validate_subtree(root)
        }

        fn validate_subtree(&self, id: NodeId) -> usize {
            let node = self.node(id);
            match node.kind {
                NodeKind::Leaf(_) => {
                    assert_eq!(node.height, 0);
                    1
                }
                NodeKind::Internal([first, second]) => {
                    for child in [first, second] {
                        assert_eq!(self.node(child).parent, Some(id), "broken parent link");
                        assert!(node.aabb.contains(&self.node(child).aabb), "parent box must contain children");
                    }
                    assert_eq!(node.height, 1 + self.node(first).height.max(self.node(second).height));
                    self.validate_subtree(first) + self.validate_subtree(second)
                }
                NodeKind::Free { .. } => panic!("free node {id} is linked into the tree"),
            }
        }
    }

    fn unit_box_at(x: f32, y: f32) -> Aabb {
        Aabb::new(vec2(x, y), vec2(x + 1.0, y + 1.0))
    }

    fn query_all(tree: &DynamicTree<usize>, aabb: &Aabb) -> Vec<usize> {
        let mut found = Vec::new();
        tree.query(aabb, |_, data| found.push(data));
        found.sort_unstable();
        found
    }

    #[test]
    fn query_matches_brute_force() {
        let mut tree = DynamicTree::default();
        let boxes: Vec<_> = (0..200).map(|i| unit_box_at((i % 20) as f32 * 1.5, (i / 20) as f32 * 1.5)).collect();
        for (i, aabb) in boxes.iter().enumerate() {
            tree.create_proxy(*aabb, i);
        }
        assert_eq!(tree.validate(), boxes.len());

        let query = Aabb::new(vec2(3.2, 2.0), vec2(9.0, 6.1));
        let expected: Vec<_> = (0..boxes.len()).filter(|&i| boxes[i].overlaps(&query)).collect();
        assert_eq!(query_all(&tree, &query), expected);
    }

    #[test]
    fn ray_cast_visits_crossed_leaves() {
        let mut tree = DynamicTree::default();
        for i in 0..10 {
            tree.create_proxy(unit_box_at(i as f32 * 2.0, 0.0), i);
        }
        let mut visited = Vec::new();
        tree.ray_cast(vec2(-1.0, 0.5), vec2(10.0, 0.0), 1.0, |data, max_fraction| {
            visited.push(data);
            max_fraction
        });
        visited.sort_unstable();
        // The segment ends at x = 9: boxes starting at 0, 2, 4, 6 and 8.
        assert_eq!(visited, [0, 1, 2, 3, 4]);
    }

    #[test]
    fn sorted_insertions_stay_balanced() {
        let mut tree = DynamicTree::default();
        let count = 1024;
        for i in 0..count {
            tree.create_proxy(unit_box_at(i as f32 * 2.0, 0.0), i);
        }
        tree.validate();
        // A perfectly balanced tree of 1024 leaves has height 10.
        assert!(tree.height() <= 20, "height {} is too large", tree.height());
    }

    #[test]
    fn moved_and_destroyed_proxies_are_found_where_they_are() {
        let mut tree = DynamicTree::default();
        let proxies: Vec<_> = (0..50).map(|i| tree.create_proxy(unit_box_at(i as f32 * 2.0, 0.0), i)).collect();

        tree.move_proxy(proxies[3], unit_box_at(0.0, 50.0));
        for &proxy in &proxies[10..20] {
            tree.destroy_proxy(proxy);
        }
        assert_eq!(tree.validate(), 40);

        assert_eq!(query_all(&tree, &unit_box_at(0.0, 50.0)), [3]);
        assert_eq!(query_all(&tree, &unit_box_at(6.0, 0.0)), Vec::<usize>::new());
        assert_eq!(query_all(&tree, &unit_box_at(30.0, 0.0)), Vec::<usize>::new());

        // Freed nodes are reused and the tree stays consistent.
        tree.create_proxy(unit_box_at(30.0, 0.0), 99);
        assert_eq!(tree.validate(), 41);
        assert_eq!(query_all(&tree, &unit_box_at(30.0, 0.0)), [99]);
    }

    #[test]
    fn removing_every_proxy_empties_the_tree() {
        let mut tree = DynamicTree::default();
        let proxies: Vec<_> = (0..10).map(|i| tree.create_proxy(unit_box_at(i as f32, 0.0), i)).collect();
        for proxy in proxies {
            tree.destroy_proxy(proxy);
        }
        assert_eq!(tree.validate(), 0);
        assert_eq!(query_all(&tree, &unit_box_at(0.0, 0.0)), Vec::<usize>::new());
    }
}
