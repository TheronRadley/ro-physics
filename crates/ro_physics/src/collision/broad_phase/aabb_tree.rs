//! AABB tree implementation for broad-phase collision detection.
//!
//! This module provides a dynamic AABB tree for efficient spatial queries.

use std::fmt;

use crate::constants::EPSILON;
use crate::geometry::Aabb;
use crate::math::Vec2;

/// A node in the AABB tree
#[derive(Debug, Clone, Copy)]
struct TreeNode {
    /// AABB of this node
    aabb: Aabb,
    /// User data (collider index or other identifier)
    user_data: usize,
    /// Left child index
    left: Option<usize>,
    /// Right child index
    right: Option<usize>,
    /// Parent index
    parent: Option<usize>,
    /// Height of this node in the tree
    height: usize,
}

impl TreeNode {
    /// Create a new leaf node
    #[inline]
    fn new_leaf(aabb: Aabb, user_data: usize) -> Self {
        Self {
            aabb,
            user_data,
            left: None,
            right: None,
            parent: None,
            height: 1,
        }
    }

    /// Check if this is a leaf node
    #[inline]
    #[must_use]
    fn is_leaf(&self) -> bool {
        self.left.is_none()
    }

    /// Get the combined AABB of this node and its children
    #[inline]
    #[must_use]
    fn combined_aabb(&self, nodes: &[TreeNode]) -> Aabb {
        if self.is_leaf() {
            self.aabb
        } else {
            let left_aabb = nodes[self.left.unwrap()].combined_aabb(nodes);
            let right_aabb = nodes[self.right.unwrap()].combined_aabb(nodes);
            left_aabb.union(right_aabb)
        }
    }
}

/// A dynamic AABB tree for broad-phase collision detection.
///
/// The AABB tree is a binary tree where each node contains an AABB that
/// bounds all the AABBs in its subtree. This allows for efficient spatial
/// queries and collision pair detection.
#[derive(Debug)]
pub struct AabbTree {
    /// Nodes in the tree
    nodes: Vec<TreeNode>,
    /// Root node index
    root: Option<usize>,
    /// Free list (reusable node indices)
    free_list: Vec<usize>,
    /// Capacity of the tree
    capacity: usize,
}

impl AabbTree {
    /// Create a new AABB tree with the given capacity
    #[inline]
    #[must_use]
    pub fn new(capacity: usize) -> Self {
        Self {
            nodes: Vec::with_capacity(capacity),
            root: None,
            free_list: Vec::new(),
            capacity,
        }
    }

    /// Get the number of nodes in the tree
    #[inline]
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.nodes.len() - self.free_list.len()
    }

    /// Get the height of the tree
    #[inline]
    #[must_use]
    pub fn height(&self) -> usize {
        self.root
            .map(|r| self.nodes[r].height)
            .unwrap_or(0)
    }

    /// Insert a new AABB into the tree
    ///
    /// Returns the index of the new node.
    #[inline]
    #[must_use]
    pub fn insert(&mut self, aabb: Aabb, user_data: usize) -> usize {
        // Check if we have a free node
        let node_index = if let Some(index) = self.free_list.pop() {
            index
        } else {
            // Allocate a new node
            let index = self.nodes.len();
            if index >= self.capacity {
                // Need to grow the tree
                self.grow();
            }
            self.nodes.push(TreeNode::new_leaf(aabb, user_data));
            index
        };

        // Insert into the tree
        self.insert_node(node_index);

        node_index
    }

    /// Remove a node from the tree
    #[inline]
    pub fn remove(&mut self, node_index: usize) {
        if node_index >= self.nodes.len() {
            return;
        }

        // Remove from the tree
        self.remove_node(node_index);

        // Add to free list
        self.free_list.push(node_index);
    }

    /// Update a node's AABB
    #[inline]
    pub fn update(&mut self, node_index: usize, aabb: Aabb) {
        if node_index >= self.nodes.len() {
            return;
        }

        let old_aabb = self.nodes[node_index].aabb;
        self.nodes[node_index].aabb = aabb;

        // If the AABB grew, we may need to reinsert
        if aabb != old_aabb {
            // For now, just reinsert (this is simplified)
            self.remove_node(node_index);
            self.insert_node(node_index);
        }
    }

    /// Get the AABB of a node
    #[inline]
    #[must_use]
    pub fn get_aabb(&self, node_index: usize) -> Option<Aabb> {
        self.nodes
            .get(node_index)
            .map(|node| node.combined_aabb(&self.nodes))
    }

    /// Query the tree for nodes that overlap with the given AABB
    #[inline]
    #[must_use]
    pub fn query(&self, aabb: Aabb) -> Vec<usize> {
        let mut results = Vec::new();
        if let Some(root) = self.root {
            self.query_node(root, aabb, &mut results);
        }
        results
    }

    /// Query the tree for nodes that contain the given point
    #[inline]
    #[must_use]
    pub fn query_point(&self, point: Vec2) -> Vec<usize> {
        let point_aabb = Aabb::from_point(point);
        self.query(point_aabb)
    }

    /// Query the tree for nodes that a ray intersects
    #[inline]
    #[must_use]
    pub fn query_ray(
        &self,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
    ) -> Vec<(usize, f32)> {
        let mut results = Vec::new();
        if let Some(root) = self.root {
            self.query_ray_node(
                root,
                origin,
                direction,
                max_distance,
                &mut results,
            );
        }
        
        // Sort by distance
        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        results
    }

    /// Clear all nodes from the tree
    #[inline]
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.root = None;
        self.free_list.clear();
    }

    /// Insert a node into the tree
    fn insert_node(&mut self, node_index: usize) {
        if self.root.is_none() {
            self.root = Some(node_index);
            return;
        }

        // Find the best place to insert
        let mut current = self.root.unwrap();
        let node_aabb = self.nodes[node_index].aabb;

        loop {
            let current_aabb = self.nodes[current].combined_aabb(&self.nodes);
            let combined_aabb = current_aabb.union(node_aabb);

            // Check if we can insert here
            if self.nodes[current].is_leaf() {
                // Create a new parent node
                let old_node = self.nodes[current].clone();
                
                // Convert current node to left child
                self.nodes[current].left = None;
                self.nodes[current].right = None;
                self.nodes[current].aabb = combined_aabb;
                self.nodes[current].height = 2;

                // Allocate new nodes for the old node and the new node
                let left_index = self.allocate_node(old_node);
                let right_index = node_index;

                self.nodes[current].left = Some(left_index);
                self.nodes[current].right = Some(right_index);

                self.nodes[left_index].parent = Some(current);
                self.nodes[right_index].parent = Some(current);

                // Update heights
                self.update_heights(current);

                // Rebalance if needed
                self.rebalance(current);

                break;
            } else {
                // Go to the child with the smallest area increase
                let left = self.nodes[current].left.unwrap();
                let right = self.nodes[current].right.unwrap();

                let left_aabb = self.nodes[left].combined_aabb(&self.nodes);
                let right_aabb = self.nodes[right].combined_aabb(&self.nodes);

                let left_combined = left_aabb.union(node_aabb);
                let right_combined = right_aabb.union(node_aabb);

                let left_area = left_combined.area();
                let right_area = right_combined.area();

                if left_area < right_area {
                    current = left;
                } else {
                    current = right;
                }
            }
        }
    }

    /// Remove a node from the tree
    fn remove_node(&mut self, node_index: usize) {
        if node_index >= self.nodes.len() {
            return;
        }

        let node = &self.nodes[node_index];

        if node.parent.is_none() {
            // This is the root node
            self.root = None;
            return;
        }

        let parent = node.parent.unwrap();
        let sibling = if self.nodes[parent].left == Some(node_index) {
            self.nodes[parent].right
        } else {
            self.nodes[parent].left
        };

        if let Some(sibling_index) = sibling {
            // Replace parent with sibling
            let sibling_node = self.nodes[sibling_index].clone();
            
            // Copy sibling to parent
            self.nodes[parent] = sibling_node;
            self.nodes[parent].parent = node.parent;

            // Update children's parent pointers
            if let Some(left) = self.nodes[parent].left {
                self.nodes[left].parent = Some(parent);
            }
            if let Some(right) = self.nodes[parent].right {
                self.nodes[right].parent = Some(parent);
            }

            // Update root if needed
            if node.parent == self.root {
                self.root = Some(parent);
            }

            // Update heights and rebalance
            self.update_heights(parent);
            self.rebalance(parent);
        } else {
            // Just remove from parent
            if self.nodes[parent].left == Some(node_index) {
                self.nodes[parent].left = None;
            } else {
                self.nodes[parent].right = None;
            }

            // Update parent's AABB
            self.update_aabb(parent);
            self.update_heights(parent);
            self.rebalance(parent);
        }
    }

    /// Allocate a new node
    fn allocate_node(&mut self, node: TreeNode) -> usize {
        if let Some(index) = self.free_list.pop() {
            self.nodes[index] = node;
            index
        } else {
            let index = self.nodes.len();
            self.nodes.push(node);
            index
        }
    }

    /// Update the AABB of a node based on its children
    fn update_aabb(&mut self, node_index: usize) {
        if node_index >= self.nodes.len() {
            return;
        }

        let node = &mut self.nodes[node_index];
        if node.is_leaf() {
            return;
        }

        let left = node.left.unwrap();
        let right = node.right.unwrap();

        node.aabb = self.nodes[left]
            .combined_aabb(&self.nodes)
            .union(self.nodes[right].combined_aabb(&self.nodes));
    }

    /// Update heights from a node up to the root
    fn update_heights(&mut self, node_index: usize) {
        let mut current = node_index;

        loop {
            let node = &mut self.nodes[current];
            if node.is_leaf() {
                node.height = 1;
            } else {
                let left = node.left.unwrap();
                let right = node.right.unwrap();
                node.height = 1 + self.nodes[left].height.max(self.nodes[right].height);
            }

            if let Some(parent) = node.parent {
                current = parent;
            } else {
                break;
            }
        }
    }

    /// Rebalance the tree from a node up to the root
    fn rebalance(&mut self, node_index: usize) {
        let mut current = node_index;

        loop {
            if !self.needs_rebalance(current) {
                break;
            }

            // Perform rotation
            let parent = self.nodes[current].parent.unwrap();
            let grandparent = self.nodes[parent].parent;

            let is_left_child = self.nodes[parent].left == Some(current);

            if is_left_child {
                self.rotate_right(parent);
            } else {
                self.rotate_left(parent);
            }

            // Update heights
            self.update_heights(if let Some(gp) = grandparent {
                gp
            } else {
                parent
            });

            if let Some(gp) = grandparent {
                current = gp;
            } else {
                break;
            }
        }
    }

    /// Check if a node needs rebalancing
    fn needs_rebalance(&self, node_index: usize) -> bool {
        if node_index >= self.nodes.len() {
            return false;
        }

        let node = &self.nodes[node_index];
        if node.is_leaf() {
            return false;
        }

        let left = node.left.unwrap();
        let right = node.right.unwrap();

        let left_height = self.nodes[left].height;
        let right_height = self.nodes[right].height;

        left_height > right_height + 1 || right_height > left_height + 1
    }

    /// Perform a left rotation
    fn rotate_left(&mut self, node_index: usize) {
        let node = &self.nodes[node_index];
        let right = node.right.unwrap();

        let right_left = self.nodes[right].left;

        // Set right as the new parent
        self.nodes[right].left = Some(node_index);
        self.nodes[right].parent = node.parent;

        // Set node as the left child of right
        self.nodes[node_index].right = right_left;
        self.nodes[node_index].parent = Some(right);

        if let Some(right_left_index) = right_left {
            self.nodes[right_left_index].parent = Some(node_index);
        }

        // Update parent's child pointer
        if let Some(parent) = node.parent {
            if self.nodes[parent].left == Some(node_index) {
                self.nodes[parent].left = Some(right);
            } else {
                self.nodes[parent].right = Some(right);
            }
        } else {
            self.root = Some(right);
        }

        // Update AABBs and heights
        self.update_aabb(node_index);
        self.update_aabb(right);
    }

    /// Perform a right rotation
    fn rotate_right(&mut self, node_index: usize) {
        let node = &self.nodes[node_index];
        let left = node.left.unwrap();

        let left_right = self.nodes[left].right;

        // Set left as the new parent
        self.nodes[left].right = Some(node_index);
        self.nodes[left].parent = node.parent;

        // Set node as the right child of left
        self.nodes[node_index].left = left_right;
        self.nodes[node_index].parent = Some(left);

        if let Some(left_right_index) = left_right {
            self.nodes[left_right_index].parent = Some(node_index);
        }

        // Update parent's child pointer
        if let Some(parent) = node.parent {
            if self.nodes[parent].left == Some(node_index) {
                self.nodes[parent].left = Some(left);
            } else {
                self.nodes[parent].right = Some(left);
            }
        } else {
            self.root = Some(left);
        }

        // Update AABBs and heights
        self.update_aabb(node_index);
        self.update_aabb(left);
    }

    /// Query a node for overlaps with an AABB
    fn query_node(&self, node_index: usize, aabb: Aabb, results: &mut Vec<usize>) {
        if node_index >= self.nodes.len() {
            return;
        }

        let node = &self.nodes[node_index];

        // Check if this node's AABB overlaps with the query AABB
        if !node.combined_aabb(&self.nodes).overlaps(aabb) {
            return;
        }

        // If this is a leaf, add it to results
        if node.is_leaf() {
            results.push(node_index);
            return;
        }

        // Query children
        if let Some(left) = node.left {
            self.query_node(left, aabb, results);
        }
        if let Some(right) = node.right {
            self.query_node(right, aabb, results);
        }
    }

    /// Query a node for ray intersections
    fn query_ray_node(
        &self,
        node_index: usize,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
        results: &mut Vec<(usize, f32)>,
    ) {
        if node_index >= self.nodes.len() {
            return;
        }

        let node = &self.nodes[node_index];

        // Check if the ray intersects this node's AABB
        if let Some(distance) = node.combined_aabb(&self.nodes).ray_cast(origin, direction, max_distance) {
            // If this is a leaf, add it to results
            if node.is_leaf() {
                results.push((node_index, distance));
                return;
            }

            // Query children
            if let Some(left) = node.left {
                self.query_ray_node(left, origin, direction, max_distance, results);
            }
            if let Some(right) = node.right {
                self.query_ray_node(right, origin, direction, max_distance, results);
            }
        }
    }

    /// Grow the tree capacity
    fn grow(&mut self) {
        let new_capacity = (self.capacity * 2).max(16);
        self.nodes.reserve(new_capacity - self.capacity);
        self.capacity = new_capacity;
    }
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

    #[test]
    fn test_construction() {
        let tree = AabbTree::new(100);
        assert_eq!(tree.node_count(), 0);
        assert_eq!(tree.height(), 0);
    }

    #[test]
    fn test_insert() {
        let mut tree = AabbTree::new(100);

        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let node1 = tree.insert(aabb1, 0);

        assert_eq!(tree.node_count(), 1);
        assert_eq!(tree.root, Some(node1));

        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        let node2 = tree.insert(aabb2, 1);

        assert_eq!(tree.node_count(), 2);
    }

    #[test]
    fn test_remove() {
        let mut tree = AabbTree::new(100);

        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let node = tree.insert(aabb, 0);

        assert_eq!(tree.node_count(), 1);

        tree.remove(node);
        assert_eq!(tree.node_count(), 0);
    }

    #[test]
    fn test_update() {
        let mut tree = AabbTree::new(100);

        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let node = tree.insert(aabb1, 0);

        let aabb2 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(20.0, 20.0));
        tree.update(node, aabb2);

        let retrieved_aabb = tree.get_aabb(node);
        assert!(retrieved_aabb.is_some());
    }

    #[test]
    fn test_query() {
        let mut tree = AabbTree::new(100);

        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        tree.insert(aabb1, 0);

        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        tree.insert(aabb2, 1);

        let query_aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(12.0, 12.0));
        let results = tree.query(query_aabb);

        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_query_point() {
        let mut tree = AabbTree::new(100);

        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        tree.insert(aabb, 0);

        let point_inside = Vec2::new(5.0, 5.0);
        let results_inside = tree.query_point(point_inside);
        assert_eq!(results_inside.len(), 1);

        let point_outside = Vec2::new(15.0, 15.0);
        let results_outside = tree.query_point(point_outside);
        assert_eq!(results_outside.len(), 0);
    }

    #[test]
    fn test_query_ray() {
        let mut tree = AabbTree::new(100);

        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        tree.insert(aabb, 0);

        let origin = Vec2::new(-5.0, 5.0);
        let direction = Vec2::new(1.0, 0.0);
        let results = tree.query_ray(origin, direction, 100.0);

        assert_eq!(results.len(), 1);
        assert!(approx_eq(results[0].1, 5.0, 1e-5));
    }

    #[test]
    fn test_clear() {
        let mut tree = AabbTree::new(100);

        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        tree.insert(aabb, 0);

        assert_eq!(tree.node_count(), 1);

        tree.clear();
        assert_eq!(tree.node_count(), 0);
        assert!(tree.root.is_none());
    }

    #[test]
    fn test_default() {
        let tree = AabbTree::default();
        assert_eq!(tree.capacity, 1024);
    }

    #[test]
    fn test_display() {
        let tree = AabbTree::new(100);
        let tree_str = format!("{}", tree);
        assert!(tree_str.contains("AabbTree"));
    }

    // Helper function for tests
    #[allow(dead_code)]
    fn approx_eq(a: f32, b: f32, epsilon: f32 = 1e-6) -> bool {
        (a - b).abs() <= epsilon
    }
}
