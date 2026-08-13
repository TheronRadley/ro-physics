//! Broad-phase collision detection.
//!
//! This module provides broad-phase collision detection using
//! a dynamic AABB tree for efficient spatial queries.

mod aabb_tree;
mod config;

pub use aabb_tree::AabbTree;
pub use config::BroadPhaseConfig;

use crate::bodies::BodyHandle;
use crate::collision::{Collider, ColliderHandle};
use crate::geometry::Aabb;
use crate::math::Vec2;

/// Broad-phase collision detection system.
///
/// The broad-phase is responsible for efficiently finding pairs of
/// colliders that might be overlapping, reducing the number of
/// narrow-phase collision checks needed.
#[derive(Debug)]
pub struct BroadPhase {
    /// Configuration
    config: BroadPhaseConfig,
    /// AABB tree for spatial indexing
    tree: AabbTree,
    /// Map from collider handles to tree nodes
    collider_to_node: std::collections::HashMap<ColliderHandle, usize>,
    /// List of active colliders
    colliders: Vec<ColliderHandle>,
}

impl BroadPhase {
    /// Create a new broad-phase detector
    #[inline]
    #[must_use]
    pub fn new(config: BroadPhaseConfig) -> Self {
        Self {
            config,
            tree: AabbTree::new(config.aabb_tree_capacity),
            collider_to_node: std::collections::HashMap::new(),
            colliders: Vec::new(),
        }
    }

    /// Create a new broad-phase detector with default configuration
    #[inline]
    #[must_use]
    pub fn with_capacity(capacity: usize) -> Self {
        Self::new(BroadPhaseConfig::new(capacity))
    }

    /// Get the configuration
    #[inline]
    #[must_use]
    pub const fn config(&self) -> &BroadPhaseConfig {
        &self.config
    }

    /// Add a collider to the broad-phase
    #[inline]
    pub fn add_collider(
        &mut self,
        handle: ColliderHandle,
        aabb: Aabb,
        user_data: usize,
    ) {
        if self.collider_to_node.contains_key(&handle) {
            return;
        }

        let node = self.tree.insert(aabb, user_data);
        self.collider_to_node.insert(handle, node);
        self.colliders.push(handle);
    }

    /// Remove a collider from the broad-phase
    #[inline]
    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        if let Some(node) = self.collider_to_node.remove(&handle) {
            self.tree.remove(node);
            self.colliders.retain(|&h| h != handle);
        }
    }

    /// Update a collider's AABB
    #[inline]
    pub fn update_collider(
        &mut self,
        handle: ColliderHandle,
        aabb: Aabb,
    ) {
        if let Some(node) = self.collider_to_node.get(&handle) {
            self.tree.update(*node, aabb);
        }
    }

    /// Find all collider pairs that might be overlapping
    #[inline]
    #[must_use]
    pub fn find_collision_pairs(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let mut pairs = Vec::new();

        // For each collider, query the tree for potential overlaps
        for &handle in &self.colliders {
            if let Some(node) = self.collider_to_node.get(&handle) {
                if let Some(aabb) = self.tree.get_aabb(*node) {
                    // Query for overlapping AABBs
                    let overlapping = self.tree.query(aabb);
                    
                    for &other_node in &overlapping {
                        if let Some(other_handle) = self.get_handle_from_node(other_node) {
                            if handle < other_handle {
                                // Only add each pair once
                                pairs.push((handle, other_handle));
                            }
                        }
                    }
                }
            }
        }

        pairs
    }

    /// Find all colliders that overlap with a given AABB
    #[inline]
    #[must_use]
    pub fn query_aabb(&self, aabb: Aabb) -> Vec<ColliderHandle> {
        let nodes = self.tree.query(aabb);
        nodes
            .into_iter()
            .filter_map(|node| self.get_handle_from_node(node))
            .collect()
    }

    /// Find all colliders that contain a given point
    #[inline]
    #[must_use]
    pub fn query_point(&self, point: Vec2) -> Vec<ColliderHandle> {
        let nodes = self.tree.query_point(point);
        nodes
            .into_iter()
            .filter_map(|node| self.get_handle_from_node(node))
            .collect()
    }

    /// Find all colliders that a ray intersects
    #[inline]
    #[must_use]
    pub fn query_ray(
        &self,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
    ) -> Vec<(ColliderHandle, f32)> {
        let results = self.tree.query_ray(origin, direction, max_distance);
        results
            .into_iter()
            .filter_map(|(node, distance)| {
                self.get_handle_from_node(node)
                    .map(|handle| (handle, distance))
            })
            .collect()
    }

    /// Get the AABB of a collider
    #[inline]
    #[must_use]
    pub fn get_aabb(&self, handle: ColliderHandle) -> Option<Aabb> {
        self.collider_to_node
            .get(&handle)
            .and_then(|&node| self.tree.get_aabb(node))
    }

    /// Get statistics about the broad-phase
    #[inline]
    #[must_use]
    pub fn stats(&self) -> BroadPhaseStats {
        BroadPhaseStats {
            collider_count: self.colliders.len(),
            tree_node_count: self.tree.node_count(),
            tree_height: self.tree.height(),
        }
    }

    /// Clear all colliders
    #[inline]
    pub fn clear(&mut self) {
        self.tree.clear();
        self.collider_to_node.clear();
        self.colliders.clear();
    }

    /// Get a handle from a tree node
    fn get_handle_from_node(&self, node: usize) -> Option<ColliderHandle> {
        for (&handle, &n) in &self.collider_to_node {
            if n == node {
                return Some(handle);
            }
        }
        None
    }
}

impl Default for BroadPhase {
    fn default() -> Self {
        Self::new(BroadPhaseConfig::default())
    }
}

/// Statistics about the broad-phase
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BroadPhaseStats {
    /// Number of colliders in the broad-phase
    pub collider_count: usize,
    /// Number of nodes in the AABB tree
    pub tree_node_count: usize,
    /// Height of the AABB tree
    pub tree_height: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::collision::ColliderHandle;

    #[test]
    fn test_construction() {
        let bp = BroadPhase::new(BroadPhaseConfig::new(100));
        assert_eq!(bp.config().aabb_tree_capacity, 100);

        let bp2 = BroadPhase::with_capacity(200);
        assert_eq!(bp2.config().aabb_tree_capacity, 200);
    }

    #[test]
    fn test_add_remove_collider() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle1 = ColliderHandle::new(1, 0);
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));

        bp.add_collider(handle1, aabb1, 0);
        assert_eq!(bp.colliders.len(), 1);
        assert!(bp.collider_to_node.contains_key(&handle1));

        bp.remove_collider(handle1);
        assert_eq!(bp.colliders.len(), 0);
        assert!(!bp.collider_to_node.contains_key(&handle1));
    }

    #[test]
    fn test_update_collider() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle = ColliderHandle::new(1, 0);
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));

        bp.add_collider(handle, aabb1, 0);
        assert_eq!(bp.get_aabb(handle), Some(aabb1));

        bp.update_collider(handle, aabb2);
        assert_eq!(bp.get_aabb(handle), Some(aabb2));
    }

    #[test]
    fn test_find_collision_pairs() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle1 = ColliderHandle::new(1, 0);
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));

        let handle2 = ColliderHandle::new(2, 0);
        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));

        let handle3 = ColliderHandle::new(3, 0);
        let aabb3 = Aabb::new(Vec2::new(20.0, 20.0), Vec2::new(30.0, 30.0));

        bp.add_collider(handle1, aabb1, 0);
        bp.add_collider(handle2, aabb2, 0);
        bp.add_collider(handle3, aabb3, 0);

        let pairs = bp.find_collision_pairs();
        
        // handle1 and handle2 should overlap
        assert!(pairs.contains(&(handle1, handle2)) || pairs.contains(&(handle2, handle1)));
        
        // handle1 and handle3 should not overlap
        assert!(!pairs.contains(&(handle1, handle3)) && !pairs.contains(&(handle3, handle1)));
        
        // handle2 and handle3 should not overlap
        assert!(!pairs.contains(&(handle2, handle3)) && !pairs.contains(&(handle3, handle2)));
    }

    #[test]
    fn test_query_aabb() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle1 = ColliderHandle::new(1, 0);
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));

        let handle2 = ColliderHandle::new(2, 0);
        let aabb2 = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));

        bp.add_collider(handle1, aabb1, 0);
        bp.add_collider(handle2, aabb2, 0);

        let query_aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(12.0, 12.0));
        let results = bp.query_aabb(query_aabb);

        assert!(results.contains(&handle1));
        assert!(results.contains(&handle2));
    }

    #[test]
    fn test_query_point() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle1 = ColliderHandle::new(1, 0);
        let aabb1 = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));

        bp.add_collider(handle1, aabb1, 0);

        let point_inside = Vec2::new(5.0, 5.0);
        let results_inside = bp.query_point(point_inside);
        assert!(results_inside.contains(&handle1));

        let point_outside = Vec2::new(15.0, 15.0);
        let results_outside = bp.query_point(point_outside);
        assert!(!results_outside.contains(&handle1));
    }

    #[test]
    fn test_stats() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let stats = bp.stats();
        assert_eq!(stats.collider_count, 0);

        let handle = ColliderHandle::new(1, 0);
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));

        bp.add_collider(handle, aabb, 0);

        let stats = bp.stats();
        assert_eq!(stats.collider_count, 1);
    }

    #[test]
    fn test_clear() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle = ColliderHandle::new(1, 0);
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));

        bp.add_collider(handle, aabb, 0);
        assert_eq!(bp.colliders.len(), 1);

        bp.clear();
        assert_eq!(bp.colliders.len(), 0);
        assert!(bp.collider_to_node.is_empty());
    }

    #[test]
    fn test_default() {
        let bp = BroadPhase::default();
        assert_eq!(bp.config().aabb_tree_capacity, 1024);
    }
}
