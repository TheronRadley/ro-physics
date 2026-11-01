//! AABB tree implementation for broad-phase collision detection.
//!
//! This module provides a dynamic AABB tree (a self-balancing binary tree of
//! fat AABBs) used to accelerate broad-phase pair generation and spatial
//! queries.
//!
//! # Implementation notes
//!
//! * Only leaves store user data; internal nodes exist purely to bound their
//!   subtree.
//! * Leaf AABBs are "fattened" by a margin so that slow movement does not
//!   force a reinsertion every frame (see [`AabbTree::update`]).
//! * Tree height is kept balanced with AVL rotations after every insert and
//!   remove, and all traversals are iterative, so deep trees cannot overflow
//!   the stack.
//! * Removed nodes are recycled through a free list instead of being dropped,
//!   which keeps node indices stable and avoids unbounded growth.

use std::fmt;

use crate::geometry::Aabb;
use crate::math::Vec2;

/// Sentinel used by [`TreeNode`] to mean "no index".
const NULL: usize = usize::MAX;

/// A node in the AABB tree
#[derive(Debug, Clone, Copy)]
struct TreeNode {
    /// Fattened AABB of this node (bounds the whole subtree for internal nodes)
    aabb: Aabb,
    /// User data (index of the owning collider entry)
    user_data: usize,
    /// Parent index, or [`NULL`]
    parent: usize,
    /// Left child index, or [`NULL`] for leaves
    left: usize,
    /// Right child index, or [`NULL`] for leaves
    right: usize,
    /// Height of the subtree (0 for leaves)
    height: u32,
}

impl TreeNode {
    /// Create a new leaf node
    #[inline]
    fn new_leaf(aabb: Aabb, user_data: usize) -> Self {
        Self {
            aabb,
            user_data,
            parent: NULL,
            left: NULL,
            right: NULL,
            height: 0,
        }
    }

    /// Check if this is a leaf node
    #[inline]
    fn is_leaf(&self) -> bool {
        self.left == NULL
    }
}

/// A dynamic AABB tree for broad-phase collision detection.
///
/// The AABB tree is a binary tree where each node contains an AABB that
/// bounds all the AABBs in its subtree. This allows for efficient spatial
/// queries and collision pair detection.
#[derive(Debug)]
pub struct AabbTree {
    /// Nodes in the tree (free nodes are parked in `free_list`)
    nodes: Vec<TreeNode>,
    /// Root node index, or [`NULL`]
    root: usize,
    /// Indices of nodes that can be reused
    free_list: Vec<usize>,
    /// Initial capacity hint (the tree grows on demand)
    capacity: usize,
    /// Margin added to AABBs on insert/update
    margin: f32,
}

impl AabbTree {
    /// Create a new AABB tree with the given initial capacity
    #[inline]
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        Self::with_margin(capacity, 0.0)
    }

    /// Create a new AABB tree that fattens inserted AABBs by `margin`
    #[inline]
    #[must_use]
    pub fn with_margin(capacity: usize, margin: f32) -> Self {
        Self {
            nodes: Vec::with_capacity(capacity),
            root: NULL,
            free_list: Vec::new(),
            capacity,
            margin: margin.max(0.0),
        }
    }

    /// Get the number of live nodes in the tree
    #[inline]
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.nodes.len() - self.free_list.len()
    }

    /// Get the height of the tree (0 for an empty tree)
    #[inline]
    #[must_use]
    pub fn height(&self) -> usize {
        if self.root == NULL {
            0
        } else {
            self.nodes[self.root].height as usize + 1
        }
    }

    /// Get the user data stored in a leaf node
    #[inline]
    #[must_use]
    pub fn user_data(&self, node: usize) -> Option<usize> {
        self.nodes.get(node).map(|node| node.user_data)
    }

    /// Insert a new AABB into the tree
    ///
    /// Returns the index of the new node. The AABB is fattened by the tree's
    /// margin before insertion.
    #[inline]
    #[must_use]
    pub fn insert(&mut self, aabb: Aabb, user_data: usize) -> usize {
        let index = self.allocate_node(fatten(aabb, self.margin), user_data);
        self.insert_leaf(index);
        index
    }

    /// Remove a node from the tree
    ///
    /// Unknown or already removed indices are ignored.
    #[inline]
    pub fn remove(&mut self, node: usize) {
        if !self.is_live(node) {
            return;
        }

        self.remove_leaf(node);
        self.free_node(node);
    }

    /// Update a node's AABB
    ///
    /// The node is only reinserted when the new AABB is no longer contained in
    /// its fattened AABB. Returns `true` when the tree was modified.
    #[inline]
    pub fn update(&mut self, node: usize, aabb: Aabb) -> bool {
        if !self.is_live(node) || !aabb.is_valid() {
            return false;
        }

        if self.nodes[node].aabb.contains_aabb(aabb) {
            return false;
        }

        self.remove_leaf(node);
        self.nodes[node].aabb = fatten(aabb, self.margin);
        self.insert_leaf(node);

        true
    }

    /// Get the AABB of a node
    #[inline]
    #[must_use]
    pub fn get_aabb(&self, node: usize) -> Option<Aabb> {
        if self.is_live(node) {
            Some(self.nodes[node].aabb)
        } else {
            None
        }
    }

    /// Query the tree for leaf nodes that overlap with the given AABB
    #[inline]
    #[must_use]
    pub fn query(&self, aabb: Aabb) -> Vec<usize> {
        let mut results = Vec::new();
        if self.root == NULL || !aabb.is_valid() {
            return results;
        }

        // Iterative traversal: no recursion, so deep trees cannot overflow.
        let mut stack = vec![self.root];

        while let Some(index) = stack.pop() {
            let node = self.nodes[index];

            if !node.aabb.overlaps(aabb) {
                continue;
            }

            if node.is_leaf() {
                results.push(index);
                continue;
            }

            stack.push(node.left);
            stack.push(node.right);
        }

        results
    }

    /// Query the tree for leaf nodes that contain the given point
    #[inline]
    #[must_use]
    pub fn query_point(&self, point: Vec2) -> Vec<usize> {
        self.query(Aabb::from_point(point))
    }

    /// Query the tree for leaf nodes that a ray intersects
    ///
    /// The returned value is the fraction along `direction` at which the ray
    /// enters the node's AABB; it is a distance in world units when
    /// `direction` is normalized. Results are sorted ascending and NaN
    /// fractions (only possible with a degenerate ray) sort last instead of
    /// panicking.
    #[inline]
    #[must_use]
    pub fn query_ray(
        &self,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
    ) -> Vec<(usize, f32)> {
        let mut results = Vec::new();
        if self.root == NULL {
            return results;
        }

        let mut stack = vec![self.root];

        while let Some(index) = stack.pop() {
            let node = self.nodes[index];

            if let Some(distance) = node.aabb.ray_cast(origin, direction, max_distance) {
                if node.is_leaf() {
                    results.push((index, distance));
                    continue;
                }

                stack.push(node.left);
                stack.push(node.right);
            }
        }

        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Greater));
        results
    }

    /// Clear all nodes from the tree
    #[inline]
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.root = NULL;
        self.free_list.clear();
    }

    /// Check the internal invariants of the tree (used by tests)
    ///
    /// Verifies parent/child links, heights and that every internal AABB bounds
    /// its children.
    #[must_use]
    pub fn validate(&self) -> bool {
        if self.root == NULL {
            return self.node_count() == 0;
        }

        let mut stack = vec![self.root];
        let mut leaves = 0usize;

        while let Some(index) = stack.pop() {
            let node = self.nodes[index];

            if node.is_leaf() {
                leaves += 1;
                if node.height != 0 {
                    return false;
                }
                continue;
            }

            let left = self.nodes[node.left];
            let right = self.nodes[node.right];

            if left.parent != index || right.parent != index {
                return false;
            }

            if node.height != 1 + left.height.max(right.height) {
                return false;
            }

            let expected = left.aabb.union(right.aabb);
            if !approx_aabb_eq(node.aabb, expected) {
                return false;
            }

            stack.push(node.left);
            stack.push(node.right);
        }

        // A well formed binary tree with `leaves` leaves has `leaves - 1` internal nodes
        self.node_count() + 1 == 2 * leaves
    }

    /// Whether a node index refers to a live node
    #[inline]
    fn is_live(&self, node: usize) -> bool {
        node < self.nodes.len() && self.nodes[node].height != u32::MAX
    }

    /// Allocate a node, reusing a free slot when possible
    #[inline]
    fn allocate_node(&mut self, aabb: Aabb, user_data: usize) -> usize {
        if let Some(index) = self.free_list.pop() {
            self.nodes[index] = TreeNode::new_leaf(aabb, user_data);
            index
        } else {
            if self.nodes.len() >= self.capacity {
                self.capacity = (self.capacity * 2).max(16);
                self.nodes.reserve(self.capacity - self.nodes.len());
            }
            self.nodes.push(TreeNode::new_leaf(aabb, user_data));
            self.nodes.len() - 1
        }
    }

    /// Park a node on the free list
    #[inline]
    fn free_node(&mut self, index: usize) {
        self.nodes[index] = TreeNode {
            aabb: Aabb::invalid(),
            user_data: 0,
            parent: NULL,
            left: NULL,
            right: NULL,
            // Marks the node as free
            height: u32::MAX,
        };
        self.free_list.push(index);
    }

    /// Insert a freshly allocated leaf into the tree
    fn insert_leaf(&mut self, leaf: usize) {
        if self.root == NULL {
            self.root = leaf;
            self.nodes[leaf].parent = NULL;
            return;
        }

        let leaf_aabb = self.nodes[leaf].aabb;

        // Descend to the leaf with the lowest insertion cost
        let mut index = self.root;
        while !self.nodes[index].is_leaf() {
            let left = self.nodes[index].left;
            let right = self.nodes[index].right;

            let combined = self.nodes[index].aabb.union(leaf_aabb);
            let inheritance_cost = 2.0 * (combined.perimeter() - self.nodes[index].aabb.perimeter());

            let cost_left = insertion_cost(&self.nodes[left], leaf_aabb) + inheritance_cost;
            let cost_right = insertion_cost(&self.nodes[right], leaf_aabb) + inheritance_cost;

            index = if cost_left < cost_right { left } else { right };
        }

        // Create a new internal node with `index` and `leaf` as children
        let sibling = index;
        let old_parent = self.nodes[sibling].parent;
        let new_parent = self.allocate_node(self.nodes[sibling].aabb.union(leaf_aabb), 0);

        self.nodes[new_parent].parent = old_parent;
        self.nodes[new_parent].left = sibling;
        self.nodes[new_parent].right = leaf;
        self.nodes[new_parent].height = self.nodes[sibling].height + 1;

        self.nodes[sibling].parent = new_parent;
        self.nodes[leaf].parent = new_parent;

        match old_parent {
            NULL => self.root = new_parent,
            parent => {
                if self.nodes[parent].left == sibling {
                    self.nodes[parent].left = new_parent;
                } else {
                    self.nodes[parent].right = new_parent;
                }
            }
        }

        self.walk_back_and_balance(new_parent);
    }

    /// Detach a leaf from the tree (the node itself stays allocated)
    fn remove_leaf(&mut self, leaf: usize) {
        debug_assert!(self.nodes[leaf].is_leaf(), "only leaves can be removed");

        if self.root == leaf {
            self.root = NULL;
            return;
        }

        let parent = self.nodes[leaf].parent;
        let grandparent = self.nodes[parent].parent;
        let sibling = if self.nodes[parent].left == leaf {
            self.nodes[parent].right
        } else {
            self.nodes[parent].left
        };

        // Replace the parent with the sibling
        self.nodes[sibling].parent = grandparent;

        match grandparent {
            NULL => self.root = sibling,
            grandparent => {
                if self.nodes[grandparent].left == parent {
                    self.nodes[grandparent].left = sibling;
                } else {
                    self.nodes[grandparent].right = sibling;
                }
            }
        }

        self.free_node(parent);

        if self.root != NULL {
            let start = if grandparent == NULL {
                self.root
            } else {
                grandparent
            };
            self.walk_back_and_balance(start);
        }
    }

    /// Recompute the AABB and height of a node from its children
    fn update_aabb_and_height(&mut self, index: usize) {
        let node = self.nodes[index];
        if node.is_leaf() {
            self.nodes[index].height = 0;
            return;
        }

        let left = self.nodes[node.left];
        let right = self.nodes[node.right];

        self.nodes[index].aabb = left.aabb.union(right.aabb);
        self.nodes[index].height = 1 + left.height.max(right.height);
    }

    /// Balance factor of a node (`right_height - left_height`)
    fn balance_factor(&self, index: usize) -> i32 {
        let node = self.nodes[index];
        if node.is_leaf() {
            return 0;
        }

        self.nodes[node.right].height as i32 - self.nodes[node.left].height as i32
    }

    /// Walk from `start` up to the root, fixing AABBs, heights and balance
    #[inline]
    fn walk_back_and_balance(&mut self, start: usize) {
        let mut index = start;

        loop {
            self.update_aabb_and_height(index);

            let balance = self.balance_factor(index);
            let current = if balance > 1 {
                self.rotate_left_heavy(index)
            } else if balance < -1 {
                self.rotate_right_heavy(index)
            } else {
                index
            };

            if current != index {
                self.update_aabb_and_height(index);
                self.update_aabb_and_height(current);
            }

            let parent = self.nodes[current].parent;
            match parent {
                NULL => {
                    self.root = current;
                    self.update_aabb_and_height(current);
                    break;
                }
                parent => {
                    if self.nodes[parent].left == index {
                        self.nodes[parent].left = current;
                    } else if self.nodes[parent].right == index {
                        self.nodes[parent].right = current;
                    }
                    index = parent;
                }
            }
        }
    }

    /// Rotate a right-heavy node (left rotation): returns the new subtree root
    fn rotate_left_heavy(&mut self, index: usize) -> usize {
        let parent = self.nodes[index].parent;
        let left = self.nodes[index].left;
        let pivot = self.nodes[index].right;
        let inner = self.nodes[pivot].left;
        let outer = self.nodes[pivot].right;

        self.nodes[pivot].left = index;
        self.nodes[pivot].right = outer;
        self.nodes[pivot].parent = parent;

        self.nodes[index].left = left;
        self.nodes[index].right = inner;
        self.nodes[index].parent = pivot;

        self.nodes[left].parent = index;
        self.nodes[inner].parent = index;
        self.nodes[outer].parent = pivot;

        pivot
    }

    /// Rotate a left-heavy node (right rotation): returns the new subtree root
    fn rotate_right_heavy(&mut self, index: usize) -> usize {
        let parent = self.nodes[index].parent;
        let right = self.nodes[index].right;
        let pivot = self.nodes[index].left;
        let inner = self.nodes[pivot].right;
        let outer = self.nodes[pivot].left;

        self.nodes[pivot].right = index;
        self.nodes[pivot].left = outer;
        self.nodes[pivot].parent = parent;

        self.nodes[index].right = right;
        self.nodes[index].left = inner;
        self.nodes[index].parent = pivot;

        self.nodes[right].parent = index;
        self.nodes[inner].parent = index;
        self.nodes[outer].parent = pivot;

        pivot
    }
}

/// Cost of inserting `leaf_aabb` below `node`
#[inline]
fn insertion_cost(node: &TreeNode, leaf_aabb: Aabb) -> f32 {
    if node.is_leaf() {
        node.aabb.union(leaf_aabb).perimeter()
    } else {
        // The child is internal: only the growth of its AABB matters
        node.aabb.union(leaf_aabb).perimeter() - node.aabb.perimeter()
    }
}

/// Expand an AABB by `margin` on every side
#[inline]
fn fatten(aabb: Aabb, margin: f32) -> Aabb {
    if margin <= 0.0 {
        return aabb;
    }

    let mut expanded = aabb;
    expanded.expand_by_margin(margin);
    expanded
}

#[inline]
fn approx_aabb_eq(a: Aabb, b: Aabb) -> bool {
    let epsilon = 1e-3;
    (a.min.x - b.min.x).abs() <= epsilon
        && (a.min.y - b.min.y).abs() <= epsilon
        && (a.max.x - b.max.x).abs() <= epsilon
        && (a.max.y - b.max.y).abs() <= epsilon
}

impl Default for AabbTree {
    fn default() -> Self {
        Self::new(1024)
    }
}

impl fmt::Display for AabbTree {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "AabbTree(nodes: {}, height: {})",
            self.node_count(),
            self.height()
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_at(x: f32, y: f32) -> Aabb {
        Aabb::from_center_half_extents(Vec2::new(x, y), Vec2::splat(0.5))
    }

    #[test]
    fn test_insert_and_query() {
        let mut tree = AabbTree::new(16);
        let a = tree.insert(box_at(0.0, 0.0), 0);
        let b = tree.insert(box_at(10.0, 0.0), 1);

        assert_eq!(tree.node_count(), 3); // two leaves + one internal node

        let hits = tree.query(box_at(0.0, 0.0));
        assert!(hits.contains(&a));
        assert!(!hits.contains(&b));

        let both = tree.query(Aabb::new(Vec2::new(-20.0, -20.0), Vec2::new(20.0, 20.0)));
        assert_eq!(both.len(), 2);
    }

    #[test]
    fn test_remove() {
        let mut tree = AabbTree::new(16);
        let a = tree.insert(box_at(0.0, 0.0), 0);
        let b = tree.insert(box_at(1.0, 0.0), 1);

        tree.remove(a);

        assert_eq!(tree.node_count(), 1);
        assert_eq!(tree.query(box_at(0.0, 0.0)), vec![b]);
        assert!(tree.validate());

        tree.remove(b);
        assert_eq!(tree.node_count(), 0);
        assert_eq!(tree.height(), 0);
    }

    #[test]
    fn test_remove_unknown_node_is_ignored() {
        let mut tree = AabbTree::new(16);
        tree.insert(box_at(0.0, 0.0), 0);

        tree.remove(4242);
        tree.remove(usize::MAX);

        assert_eq!(tree.node_count(), 1);
    }

    #[test]
    fn test_node_reuse() {
        let mut tree = AabbTree::new(16);
        let a = tree.insert(box_at(0.0, 0.0), 0);
        tree.insert(box_at(1.0, 0.0), 1);
        tree.remove(a);

        let c = tree.insert(box_at(2.0, 0.0), 2);
        // The slot of the removed node is recycled
        assert_eq!(c, a);
        assert_eq!(tree.user_data(c), Some(2));
        assert!(tree.validate());
    }

    #[test]
    fn test_update_keeps_index() {
        let mut tree = AabbTree::new(16);
        let a = tree.insert(box_at(0.0, 0.0), 7);

        // Moving far away requires a reinsertion but keeps the node index
        assert!(tree.update(a, box_at(50.0, 0.0)));
        assert_eq!(tree.user_data(a), Some(7));
        assert_eq!(tree.query(box_at(50.0, 0.0)), vec![a]);
        assert!(tree.query(box_at(0.0, 0.0)).is_empty());
        assert!(tree.validate());
    }

    #[test]
    fn test_update_inside_margin_is_free() {
        let mut tree = AabbTree::with_margin(16, 1.0);
        let a = tree.insert(box_at(0.0, 0.0), 0);

        // A small move stays inside the fattened AABB
        assert!(!tree.update(a, box_at(0.1, 0.0)));
    }

    #[test]
    fn test_tree_stays_balanced() {
        let mut tree = AabbTree::new(16);
        let mut nodes = Vec::new();

        for i in 0..200 {
            nodes.push(tree.insert(box_at(i as f32, 0.0), i));
        }

        assert!(tree.validate());
        // A balanced tree with 200 leaves has a height well below 200
        assert!(tree.height() < 24, "height was {}", tree.height());

        // Removing every other node keeps the tree valid
        for (i, node) in nodes.iter().enumerate() {
            if i % 2 == 0 {
                tree.remove(*node);
            }
        }

        assert!(tree.validate());
        // 100 leaves plus the 99 internal nodes of a full binary tree
        assert_eq!(tree.node_count(), 199);
    }

    #[test]
    fn test_query_ray_sorted() {
        let mut tree = AabbTree::new(16);
        tree.insert(box_at(10.0, 0.0), 0);
        tree.insert(box_at(5.0, 0.0), 1);

        let hits = tree.query_ray(Vec2::zero(), Vec2::unit_x(), 100.0);

        assert_eq!(hits.len(), 2);
        assert!(hits[0].1 <= hits[1].1);
    }

    #[test]
    fn test_clear() {
        let mut tree = AabbTree::new(16);
        tree.insert(box_at(0.0, 0.0), 0);
        tree.insert(box_at(1.0, 0.0), 1);

        tree.clear();

        assert_eq!(tree.node_count(), 0);
        assert!(tree.query(Aabb::new(Vec2::new(-100.0, -100.0), Vec2::new(100.0, 100.0))).is_empty());
    }
}
