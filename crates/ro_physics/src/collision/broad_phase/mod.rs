//! Broad-phase collision detection.
//!
//! This module provides broad-phase collision detection using a dynamic AABB
//! tree for efficient spatial queries.
//!
//! The broad-phase is responsible for efficiently finding pairs of colliders
//! that might be overlapping, reducing the number of narrow-phase collision
//! checks needed. Without it, collision detection is O(n²) in the number of
//! colliders; with the tree, each insert, remove and query is O(log n).
//!
//! # Bookkeeping
//!
//! Every collider added to the broad-phase occupies a *slot*. Slots are
//! recycled when colliders are removed, and the tree node belonging to a slot
//! is tracked explicitly, so no operation ever has to scan the tree to find
//! the node that owns a collider.

mod aabb_tree;
mod config;

pub use aabb_tree::AabbTree;
pub use config::BroadPhaseConfig;

use std::collections::HashMap;

use crate::collision::ColliderHandle;
use crate::geometry::Aabb;
use crate::math::Vec2;

/// Sentinel used for "this slot has no tree node"
const NO_NODE: usize = usize::MAX;

/// Broad-phase collision detection system.
#[derive(Debug)]
pub struct BroadPhase {
    /// Configuration
    config: BroadPhaseConfig,
    /// AABB tree for spatial indexing (user data of a node is its slot)
    tree: AabbTree,
    /// Map from collider handles to slots
    collider_to_slot: HashMap<ColliderHandle, usize>,
    /// Collider handle stored in each slot (null when the slot is free)
    handles: Vec<ColliderHandle>,
    /// Tight (unfattened) AABB stored in each slot
    aabbs: Vec<Aabb>,
    /// Tree node of each slot, or [`NO_NODE`]
    nodes: Vec<usize>,
    /// Caller supplied user data of each slot
    user_data: Vec<usize>,
    /// Slots that are free and can be reused
    free_slots: Vec<usize>,
    /// Handles of the colliders currently in the broad-phase
    colliders: Vec<ColliderHandle>,
}

impl BroadPhase {
    /// Create a new broad-phase detector
    #[inline]
    #[must_use]
    pub fn new(config: BroadPhaseConfig) -> Self {
        Self {
            tree: AabbTree::with_margin(config.aabb_tree_capacity, config.aabb_margin),
            config,
            collider_to_slot: HashMap::new(),
            handles: Vec::new(),
            aabbs: Vec::new(),
            nodes: Vec::new(),
            user_data: Vec::new(),
            free_slots: Vec::new(),
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
    ///
    /// Adding the same handle twice is a no-op. Invalid (non-finite or
    /// inverted) AABBs are rejected.
    #[inline]
    pub fn add_collider(&mut self, handle: ColliderHandle, aabb: Aabb, user_data: usize) {
        if handle.is_null() || !aabb.is_valid() || self.collider_to_slot.contains_key(&handle) {
            return;
        }

        let slot = match self.free_slots.pop() {
            Some(slot) => {
                self.handles[slot] = handle;
                self.aabbs[slot] = aabb;
                self.user_data[slot] = user_data;
                slot
            }
            None => {
                self.handles.push(handle);
                self.aabbs.push(aabb);
                self.nodes.push(NO_NODE);
                self.user_data.push(user_data);
                self.handles.len() - 1
            }
        };

        self.nodes[slot] = self.tree.insert(aabb, slot);
        self.collider_to_slot.insert(handle, slot);
        self.colliders.push(handle);
    }

    /// Remove a collider from the broad-phase
    #[inline]
    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        let Some(slot) = self.collider_to_slot.remove(&handle) else {
            return;
        };

        let node = self.nodes[slot];
        if node != NO_NODE {
            self.tree.remove(node);
        }

        self.nodes[slot] = NO_NODE;
        self.handles[slot] = ColliderHandle::null();
        self.aabbs[slot] = Aabb::invalid();
        self.user_data[slot] = 0;
        self.free_slots.push(slot);
        self.colliders.retain(|&existing| existing != handle);
    }

    /// Update a collider's AABB
    ///
    /// The tree is only touched when the new AABB is no longer contained in
    /// the fattened AABB stored in the tree, so slow moving colliders are
    /// cheap to update.
    #[inline]
    pub fn update_collider(&mut self, handle: ColliderHandle, aabb: Aabb) {
        if !aabb.is_valid() {
            return;
        }

        let Some(&slot) = self.collider_to_slot.get(&handle) else {
            return;
        };

        self.aabbs[slot] = aabb;

        let node = self.nodes[slot];
        if node == NO_NODE {
            self.nodes[slot] = self.tree.insert(aabb, slot);
        } else if self.config.incremental {
            self.tree.update(node, aabb);
        } else {
            self.tree.remove(node);
            self.nodes[slot] = self.tree.insert(aabb, slot);
        }
    }

    /// Find all collider pairs that might be overlapping
    ///
    /// Each unordered pair is reported exactly once, ordered by slot index.
    /// Pairs are filtered by a real overlap test on the (unfattened) AABBs, so
    /// the narrow-phase never sees pairs that are merely close.
    #[inline]
    #[must_use]
    pub fn find_collision_pairs(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let mut pairs = Vec::new();

        for &handle in &self.colliders {
            let Some(&slot) = self.collider_to_slot.get(&handle) else {
                continue;
            };

            let node = self.nodes[slot];
            if node == NO_NODE {
                continue;
            }

            let Some(tree_aabb) = self.tree.get_aabb(node) else {
                continue;
            };
            let aabb = self.aabbs[slot];

            for other_node in self.tree.query(tree_aabb) {
                let Some(other_slot) = self.tree.user_data(other_node) else {
                    continue;
                };

                // Each pair is visited twice; keep only one direction
                if other_slot <= slot {
                    continue;
                }

                let other = self.handles[other_slot];
                if other.is_null() {
                    continue;
                }

                if !aabb.overlaps(self.aabbs[other_slot]) {
                    continue;
                }

                pairs.push((handle, other));
            }
        }

        pairs
    }

    /// Find all colliders that overlap with a given AABB
    #[inline]
    #[must_use]
    pub fn query_aabb(&self, aabb: Aabb) -> Vec<ColliderHandle> {
        self.tree
            .query(aabb)
            .into_iter()
            .filter_map(|node| self.handle_for_node(node))
            .collect()
    }

    /// Find all colliders that contain a given point
    #[inline]
    #[must_use]
    pub fn query_point(&self, point: Vec2) -> Vec<ColliderHandle> {
        if !point.is_finite() {
            return Vec::new();
        }

        self.tree
            .query_point(point)
            .into_iter()
            .filter_map(|node| self.handle_for_node(node))
            .collect()
    }

    /// Find all colliders that a ray intersects
    ///
    /// Returns the handles together with the fraction along `direction` at
    /// which the ray enters the collider's AABB. Degenerate or non-finite
    /// directions yield no results instead of panicking.
    #[inline]
    #[must_use]
    pub fn query_ray(
        &self,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
    ) -> Vec<(ColliderHandle, f32)> {
        if !origin.is_finite()
            || !direction.is_finite()
            || direction.length_squared() <= 0.0
            || !max_distance.is_finite()
            || max_distance < 0.0
        {
            return Vec::new();
        }

        self.tree
            .query_ray(origin, direction, max_distance)
            .into_iter()
            .filter_map(|(node, distance)| {
                self.handle_for_node(node).map(|handle| (handle, distance))
            })
            .collect()
    }

    /// Get the (unfattened) AABB of a collider
    #[inline]
    #[must_use]
    pub fn get_aabb(&self, handle: ColliderHandle) -> Option<Aabb> {
        let slot = *self.collider_to_slot.get(&handle)?;
        let aabb = self.aabbs[slot];
        aabb.is_valid().then_some(aabb)
    }

    /// Get the user data that was supplied when a collider was added
    #[inline]
    #[must_use]
    pub fn user_data(&self, handle: ColliderHandle) -> Option<usize> {
        let slot = *self.collider_to_slot.get(&handle)?;
        Some(self.user_data[slot])
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
        self.collider_to_slot.clear();
        self.handles.clear();
        self.aabbs.clear();
        self.nodes.clear();
        self.user_data.clear();
        self.free_slots.clear();
        self.colliders.clear();
    }

    /// Number of colliders currently tracked by the broad-phase
    #[inline]
    #[must_use]
    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    /// Check the internal consistency of the tree (mainly useful in tests)
    #[inline]
    #[must_use]
    pub fn validate(&self) -> bool {
        self.tree.validate()
    }

    /// Resolve the collider handle stored in a tree node
    #[inline]
    fn handle_for_node(&self, node: usize) -> Option<ColliderHandle> {
        let slot = self.tree.user_data(node)?;
        let handle = *self.handles.get(slot)?;
        (!handle.is_null()).then_some(handle)
    }
}

impl Default for BroadPhase {
    fn default() -> Self {
        Self::new(BroadPhaseConfig::default())
    }
}

/// Statistics about the broad-phase
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

    fn box_aabb(min_x: f32, min_y: f32, size: f32) -> Aabb {
        Aabb::new(
            Vec2::new(min_x, min_y),
            Vec2::new(min_x + size, min_y + size),
        )
    }

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
        let aabb1 = box_aabb(0.0, 0.0, 10.0);

        bp.add_collider(handle1, aabb1, 0);
        assert_eq!(bp.collider_count(), 1);
        assert!(bp.collider_to_slot.contains_key(&handle1));

        bp.remove_collider(handle1);
        assert_eq!(bp.collider_count(), 0);
        assert!(!bp.collider_to_slot.contains_key(&handle1));
        assert!(bp.validate());
    }

    #[test]
    fn test_add_invalid_aabb_is_ignored() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));
        let handle = ColliderHandle::new(1, 0);

        bp.add_collider(handle, Aabb::invalid(), 0);
        bp.add_collider(ColliderHandle::null(), box_aabb(0.0, 0.0, 1.0), 0);

        assert_eq!(bp.collider_count(), 0);
    }

    #[test]
    fn test_duplicate_add_is_ignored() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));
        let handle = ColliderHandle::new(1, 0);

        bp.add_collider(handle, box_aabb(0.0, 0.0, 10.0), 0);
        bp.add_collider(handle, box_aabb(0.0, 0.0, 10.0), 1);

        assert_eq!(bp.collider_count(), 1);
        assert_eq!(bp.user_data(handle), Some(0));
    }

    #[test]
    fn test_update_collider() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle = ColliderHandle::new(1, 0);
        let aabb1 = box_aabb(0.0, 0.0, 10.0);
        let aabb2 = box_aabb(5.0, 5.0, 10.0);

        bp.add_collider(handle, aabb1, 0);
        assert_eq!(bp.get_aabb(handle), Some(aabb1));

        bp.update_collider(handle, aabb2);
        assert_eq!(bp.get_aabb(handle), Some(aabb2));
        assert!(bp.validate());
    }

    #[test]
    fn test_update_far_away_keeps_node_valid() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle = ColliderHandle::new(1, 0);
        bp.add_collider(handle, box_aabb(0.0, 0.0, 10.0), 0);
        bp.update_collider(handle, box_aabb(1000.0, 0.0, 10.0));

        assert!(bp.validate());
        assert!(bp.query_aabb(box_aabb(1000.0, 0.0, 10.0)).contains(&handle));
        assert!(!bp.query_aabb(box_aabb(0.0, 0.0, 10.0)).contains(&handle));
    }

    #[test]
    fn test_find_collision_pairs() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle1 = ColliderHandle::new(1, 0);
        let aabb1 = box_aabb(0.0, 0.0, 10.0);

        let handle2 = ColliderHandle::new(2, 0);
        let aabb2 = box_aabb(5.0, 5.0, 10.0);

        let handle3 = ColliderHandle::new(3, 0);
        let aabb3 = box_aabb(20.0, 20.0, 10.0);

        bp.add_collider(handle1, aabb1, 0);
        bp.add_collider(handle2, aabb2, 0);
        bp.add_collider(handle3, aabb3, 0);

        let pairs = bp.find_collision_pairs();

        // handle1 and handle2 overlap
        assert!(pairs.contains(&(handle1, handle2)) || pairs.contains(&(handle2, handle1)));

        // handle1 and handle3 do not overlap
        assert!(!pairs.contains(&(handle1, handle3)) && !pairs.contains(&(handle3, handle1)));

        // handle2 and handle3 do not overlap
        assert!(!pairs.contains(&(handle2, handle3)) && !pairs.contains(&(handle3, handle2)));
    }

    #[test]
    fn test_pairs_are_reported_once() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        for index in 0..20u64 {
            let x = index as f32 * 0.5;
            bp.add_collider(ColliderHandle::new(index + 1, 0), box_aabb(x, 0.0, 10.0), 0);
        }

        let pairs = bp.find_collision_pairs();

        let mut seen = Vec::new();
        for pair in &pairs {
            assert!(!seen.contains(pair), "duplicate pair {pair:?}");
            seen.push(*pair);
        }

        // Every collider overlaps its neighbours, so there must be pairs, and
        // far apart colliders must never be paired.
        assert!(!pairs.is_empty());
        assert!(pairs.len() < 20 * 20);
    }

    #[test]
    fn test_slot_reuse_after_removal() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let first = ColliderHandle::new(1, 0);
        bp.add_collider(first, box_aabb(0.0, 0.0, 1.0), 0);
        bp.remove_collider(first);

        let second = ColliderHandle::new(2, 0);
        bp.add_collider(second, box_aabb(50.0, 50.0, 1.0), 7);

        assert_eq!(bp.collider_count(), 1);
        assert_eq!(bp.user_data(second), Some(7));
        assert_eq!(bp.query_point(Vec2::new(50.5, 50.5)), vec![second]);
        assert!(bp.validate());

        // The stale handle no longer resolves
        assert_eq!(bp.get_aabb(first), None);
        assert!(!bp.query_point(Vec2::new(0.5, 0.5)).contains(&first));
    }

    #[test]
    fn test_query_aabb() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle1 = ColliderHandle::new(1, 0);
        let aabb1 = box_aabb(0.0, 0.0, 10.0);

        let handle2 = ColliderHandle::new(2, 0);
        let aabb2 = box_aabb(5.0, 5.0, 10.0);

        bp.add_collider(handle1, aabb1, 0);
        bp.add_collider(handle2, aabb2, 0);

        let results = bp.query_aabb(box_aabb(0.0, 0.0, 12.0));

        assert!(results.contains(&handle1));
        assert!(results.contains(&handle2));
    }

    #[test]
    fn test_query_point() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle1 = ColliderHandle::new(1, 0);
        bp.add_collider(handle1, box_aabb(0.0, 0.0, 10.0), 0);

        assert!(bp.query_point(Vec2::new(5.0, 5.0)).contains(&handle1));
        assert!(!bp.query_point(Vec2::new(15.0, 15.0)).contains(&handle1));
        // Non finite points never panic
        assert!(bp.query_point(Vec2::new(f32::NAN, 0.0)).is_empty());
    }

    #[test]
    fn test_query_ray() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let near = ColliderHandle::new(1, 0);
        let far = ColliderHandle::new(2, 0);
        bp.add_collider(near, box_aabb(5.0, -1.0, 2.0), 0);
        bp.add_collider(far, box_aabb(10.0, -1.0, 2.0), 0);

        let hits = bp.query_ray(Vec2::zero(), Vec2::unit_x(), 100.0);

        assert_eq!(hits.len(), 2);
        assert_eq!(hits[0].0, near);
        assert_eq!(hits[1].0, far);
        assert!(hits[0].1 < hits[1].1);

        // A degenerate ray is rejected instead of panicking
        assert!(bp.query_ray(Vec2::zero(), Vec2::zero(), 100.0).is_empty());
        assert!(bp
            .query_ray(Vec2::zero(), Vec2::new(f32::NAN, 0.0), 100.0)
            .is_empty());
    }

    #[test]
    fn test_stats() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let stats = bp.stats();
        assert_eq!(stats.collider_count, 0);
        assert_eq!(stats.tree_node_count, 0);
        assert_eq!(stats.tree_height, 0);

        let handle = ColliderHandle::new(1, 0);
        bp.add_collider(handle, box_aabb(0.0, 0.0, 10.0), 0);

        let stats = bp.stats();
        assert_eq!(stats.collider_count, 1);
        assert_eq!(stats.tree_node_count, 1);
    }

    #[test]
    fn test_tree_stays_balanced_over_many_inserts() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(16));

        for index in 0..300u64 {
            let x = index as f32;
            bp.add_collider(ColliderHandle::new(index + 1, 0), box_aabb(x, 0.0, 1.0), 0);
        }

        assert!(bp.validate());
        assert!(
            bp.stats().tree_height < 24,
            "height {}",
            bp.stats().tree_height
        );

        for index in (0..300u64).step_by(3) {
            bp.remove_collider(ColliderHandle::new(index + 1, 0));
        }

        assert!(bp.validate());
        assert_eq!(bp.collider_count(), 200);
    }

    #[test]
    fn test_clear() {
        let mut bp = BroadPhase::new(BroadPhaseConfig::new(100));

        let handle = ColliderHandle::new(1, 0);
        bp.add_collider(handle, box_aabb(0.0, 0.0, 10.0), 0);
        assert_eq!(bp.collider_count(), 1);

        bp.clear();

        assert_eq!(bp.collider_count(), 0);
        assert!(bp.collider_to_slot.is_empty());
        assert_eq!(bp.stats().tree_node_count, 0);
    }

    #[test]
    fn test_default() {
        let bp = BroadPhase::default();
        assert_eq!(bp.config().aabb_tree_capacity, 1024);
    }
}
