//! Bounding Volume Hierarchy (BVH) for spatial acceleration.
//!
//! This module provides a BVH implementation for accelerating spatial queries
//! like ray casting, point queries, and AABB queries.

use std::fmt;

use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;
use crate::geometry::Aabb;
use crate::math::Vec2;

/// BVH node for spatial partitioning
#[derive(Debug, Clone)]
struct BvhNode {
    /// Bounding box of this node
    aabb: Aabb,
    /// Left child index (None for leaf nodes)
    left: Option<usize>,
    /// Right child index (None for leaf nodes)
    right: Option<usize>,
    /// Collider index (for leaf nodes)
    collider_index: Option<usize>,
}

/// Bounding Volume Hierarchy for spatial acceleration
#[derive(Debug)]
pub struct Bvh {
    /// List of all nodes
    nodes: Vec<BvhNode>,
    /// Root node index
    root: Option<usize>,
    /// List of collider AABBs
    collider_aabbs: Vec<Aabb>,
    /// List of collider handles
    collider_handles: Vec<ColliderHandle>,
    /// List of body handles
    body_handles: Vec<BodyHandle>,
}

impl Bvh {
    /// Create a new empty BVH
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: None,
            collider_aabbs: Vec::new(),
            collider_handles: Vec::new(),
            body_handles: Vec::new(),
        }
    }

    /// Create a BVH from a list of collider AABBs
    #[inline]
    #[must_use]
    pub fn from_colliders(
        collider_aabbs: Vec<Aabb>,
        collider_handles: Vec<ColliderHandle>,
        body_handles: Vec<BodyHandle>,
    ) -> Self {
        let mut bvh = Self::new();
        bvh.build(collider_aabbs, collider_handles, body_handles);
        bvh
    }

    /// Build the BVH from collider data
    pub fn build(
        &mut self,
        collider_aabbs: Vec<Aabb>,
        collider_handles: Vec<ColliderHandle>,
        body_handles: Vec<BodyHandle>,
    ) {
        self.collider_aabbs = collider_aabbs;
        self.collider_handles = collider_handles;
        self.body_handles = body_handles;
        self.nodes.clear();
        self.root = None;

        if self.collider_aabbs.is_empty() {
            return;
        }

        // Build indices for sorting
        let mut indices: Vec<usize> = (0..self.collider_aabbs.len()).collect();

        // Sort along the longest axis
        self.sort_along_axis(&mut indices, 0);

        // Recursively build the tree
        self.root = Some(self.build_node(&indices));
    }

    /// Sort indices along a given axis
    fn sort_along_axis(&self, indices: &mut [usize], axis: usize) {
        indices.sort_by(|&a, &b| {
            let a_center = self.collider_aabbs[a].center();
            let b_center = self.collider_aabbs[b].center();
            // `total_cmp` gives a total order for floats, so NaN can never
            // panic here the way `partial_cmp(..).unwrap()` would.
            axis_value(a_center, axis).total_cmp(&axis_value(b_center, axis))
        });
    }

    /// Build a BVH node from a range of indices
    fn build_node(&mut self, indices: &[usize]) -> usize {
        if indices.len() == 1 {
            // Leaf node
            let node = BvhNode {
                aabb: self.collider_aabbs[indices[0]],
                left: None,
                right: None,
                collider_index: Some(indices[0]),
            };
            self.nodes.push(node);
            return self.nodes.len() - 1;
        }

        // Internal node
        // Choose axis with largest span
        let axis = self.choose_split_axis(indices);

        // Sort along chosen axis
        let mut sorted_indices = indices.to_vec();
        self.sort_along_axis(&mut sorted_indices, axis);

        // Split in the middle
        let mid = sorted_indices.len() / 2;
        let left_indices = &sorted_indices[..mid];
        let right_indices = &sorted_indices[mid..];

        // Calculate combined AABB
        let mut aabb = Aabb::invalid();
        for &idx in &sorted_indices {
            aabb = aabb.union(self.collider_aabbs[idx]);
        }

        // Build children
        let left_child = self.build_node(left_indices);
        let right_child = self.build_node(right_indices);

        let node = BvhNode {
            aabb,
            left: Some(left_child),
            right: Some(right_child),
            collider_index: None,
        };

        self.nodes.push(node);
        self.nodes.len() - 1
    }

    /// Choose the best axis to split along
    fn choose_split_axis(&self, indices: &[usize]) -> usize {
        let mut best_axis = 0;
        let mut best_span = f32::NEG_INFINITY;

        for axis in 0..2 {
            let mut min_val = f32::INFINITY;
            let mut max_val = f32::NEG_INFINITY;

            for &idx in indices {
                let center = self.collider_aabbs[idx].center();
                min_val = min_val.min(axis_value(center, axis));
                max_val = max_val.max(axis_value(center, axis));
            }

            let span = max_val - min_val;
            if span > best_span {
                best_span = span;
                best_axis = axis;
            }
        }

        best_axis
    }

    /// Clear the BVH
    #[inline]
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.root = None;
        self.collider_aabbs.clear();
        self.collider_handles.clear();
        self.body_handles.clear();
    }

    /// Check if the BVH is empty
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.root.is_none()
    }

    /// Get the number of nodes
    #[inline]
    #[must_use]
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get the number of leaf nodes (colliders)
    #[inline]
    #[must_use]
    pub fn collider_count(&self) -> usize {
        self.collider_aabbs.len()
    }

    /// Query for colliders containing a point using BVH acceleration
    #[inline]
    #[must_use]
    pub fn point_query(&self, point: Vec2) -> Vec<(BodyHandle, ColliderHandle)> {
        let mut results = Vec::new();

        if let Some(root_idx) = self.root {
            self.point_query_recursive(root_idx, point, &mut results);
        }

        results
    }

    /// Recursive point query
    fn point_query_recursive(
        &self,
        node_idx: usize,
        point: Vec2,
        results: &mut Vec<(BodyHandle, ColliderHandle)>,
    ) {
        let node = &self.nodes[node_idx];

        // Check if point is in node's AABB
        if !node.aabb.contains_point(point) {
            return;
        }

        // If leaf node, check the collider
        if let Some(collider_idx) = node.collider_index {
            results.push((
                self.body_handles[collider_idx],
                self.collider_handles[collider_idx],
            ));
            return;
        }

        // Recurse into children
        if let Some(left) = node.left {
            self.point_query_recursive(left, point, results);
        }
        if let Some(right) = node.right {
            self.point_query_recursive(right, point, results);
        }
    }

    /// Query for colliders overlapping an AABB using BVH acceleration
    #[inline]
    #[must_use]
    pub fn aabb_query(&self, aabb: Aabb) -> Vec<(BodyHandle, ColliderHandle)> {
        let mut results = Vec::new();

        if let Some(root_idx) = self.root {
            self.aabb_query_recursive(root_idx, aabb, &mut results);
        }

        results
    }

    /// Recursive AABB query
    fn aabb_query_recursive(
        &self,
        node_idx: usize,
        aabb: Aabb,
        results: &mut Vec<(BodyHandle, ColliderHandle)>,
    ) {
        let node = &self.nodes[node_idx];

        // Check if AABBs overlap
        if !node.aabb.overlaps(aabb) {
            return;
        }

        // If leaf node, add the collider
        if let Some(collider_idx) = node.collider_index {
            results.push((
                self.body_handles[collider_idx],
                self.collider_handles[collider_idx],
            ));
            return;
        }

        // Recurse into children
        if let Some(left) = node.left {
            self.aabb_query_recursive(left, aabb, results);
        }
        if let Some(right) = node.right {
            self.aabb_query_recursive(right, aabb, results);
        }
    }

    /// Cast a ray and return all hits using BVH acceleration
    #[inline]
    #[must_use]
    pub fn ray_cast_all(
        &self,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
    ) -> Vec<(BodyHandle, ColliderHandle, f32)> {
        let mut results = Vec::new();

        if let Some(root_idx) = self.root {
            self.ray_cast_recursive(root_idx, origin, direction, max_distance, &mut results);
        }

        // Sort by distance
        results.sort_by(|a, b| a.2.total_cmp(&b.2));
        results
    }

    /// Recursive ray cast
    fn ray_cast_recursive(
        &self,
        node_idx: usize,
        origin: Vec2,
        direction: Vec2,
        max_distance: f32,
        results: &mut Vec<(BodyHandle, ColliderHandle, f32)>,
    ) {
        let node = &self.nodes[node_idx];

        // Check ray-AABB intersection
        if let Some((t_min, t_max)) = ray_aabb_intersection(origin, direction, node.aabb) {
            if t_min > max_distance || t_max < 0.0 {
                return;
            }
        } else {
            return;
        }

        // If leaf node, add the potential hit
        if let Some(collider_idx) = node.collider_index {
            // For now, just record that this collider might be hit
            // In a full implementation, we'd do exact ray-collider intersection
            results.push((
                self.body_handles[collider_idx],
                self.collider_handles[collider_idx],
                0.0, // Placeholder distance
            ));
            return;
        }

        // Recurse into children (front to back for better performance)
        if let Some(left) = node.left {
            self.ray_cast_recursive(left, origin, direction, max_distance, results);
        }
        if let Some(right) = node.right {
            self.ray_cast_recursive(right, origin, direction, max_distance, results);
        }
    }

    /// Update a collider's AABB
    #[inline]
    pub fn update_collider(&mut self, index: usize, aabb: Aabb) {
        if index < self.collider_aabbs.len() {
            self.collider_aabbs[index] = aabb;
            // Rebuild the BVH
            self.rebuild();
        }
    }

    /// Rebuild the BVH after modifications
    #[inline]
    pub fn rebuild(&mut self) {
        if !self.collider_aabbs.is_empty() {
            let indices: Vec<usize> = (0..self.collider_aabbs.len()).collect();
            self.root = Some(self.build_node(&indices));
        } else {
            self.root = None;
        }
    }
}

impl Default for Bvh {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for Bvh {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "BVH(nodes: {}, colliders: {})",
            self.node_count(),
            self.collider_count()
        )
    }
}

/// Calculate intersection between a ray and an AABB
///
/// Returns Some((t_min, t_max)) if there's an intersection, None otherwise
///
/// Read one component of a vector (0 -> x, anything else -> y)
/// `Vec2` does not implement `Index`, so spatial code has to go through this
/// helper instead of indexing with a dynamic axis.
#[inline]
fn axis_value(v: Vec2, axis: usize) -> f32 {
    if axis == 0 {
        v.x
    } else {
        v.y
    }
}

fn ray_aabb_intersection(origin: Vec2, direction: Vec2, aabb: Aabb) -> Option<(f32, f32)> {
    use crate::constants::EPSILON;

    let inv_direction = Vec2::new(
        if direction.x.abs() > EPSILON {
            1.0 / direction.x
        } else {
            f32::INFINITY
        },
        if direction.y.abs() > EPSILON {
            1.0 / direction.y
        } else {
            f32::INFINITY
        },
    );

    let mut t_min: f32 = 0.0;
    let mut t_max = f32::INFINITY;

    // X-axis
    let t1 = (aabb.min.x - origin.x) * inv_direction.x;
    let t2 = (aabb.max.x - origin.x) * inv_direction.x;
    t_min = t_min.max(t1.min(t2));
    t_max = t_max.min(t1.max(t2));

    // Y-axis
    let t1 = (aabb.min.y - origin.y) * inv_direction.y;
    let t2 = (aabb.max.y - origin.y) * inv_direction.y;
    t_min = t_min.max(t1.min(t2));
    t_max = t_max.min(t1.max(t2));

    if t_min <= t_max && t_max >= 0.0 {
        Some((t_min, t_max))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::collision::ColliderHandle;

    #[test]
    fn test_bvh_construction() {
        let bvh = Bvh::new();
        assert!(bvh.is_empty());
        assert_eq!(bvh.node_count(), 0);
        assert_eq!(bvh.collider_count(), 0);
    }

    #[test]
    fn test_bvh_build() {
        let aabbs = vec![
            Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
            Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(-0.5, -0.5)),
            Aabb::new(Vec2::new(0.5, 0.5), Vec2::new(2.0, 2.0)),
        ];

        let handles = vec![
            ColliderHandle::new(1, 0),
            ColliderHandle::new(2, 0),
            ColliderHandle::new(3, 0),
        ];

        let body_handles = vec![
            BodyHandle::new(1, 0),
            BodyHandle::new(2, 0),
            BodyHandle::new(3, 0),
        ];

        let mut bvh = Bvh::new();
        bvh.build(aabbs, handles, body_handles);

        assert!(!bvh.is_empty());
        assert!(bvh.node_count() > 0);
        assert_eq!(bvh.collider_count(), 3);
    }

    #[test]
    fn test_bvh_point_query() {
        let aabbs = vec![
            Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
            Aabb::new(Vec2::new(2.0, 2.0), Vec2::new(3.0, 3.0)),
        ];

        let handles = vec![ColliderHandle::new(1, 0), ColliderHandle::new(2, 0)];

        let body_handles = vec![BodyHandle::new(1, 0), BodyHandle::new(2, 0)];

        let mut bvh = Bvh::new();
        bvh.build(aabbs, handles, body_handles);

        // Query point inside first AABB
        let results = bvh.point_query(Vec2::new(0.0, 0.0));
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].1, ColliderHandle::new(1, 0));

        // Query point inside second AABB
        let results = bvh.point_query(Vec2::new(2.5, 2.5));
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].1, ColliderHandle::new(2, 0));

        // Query point outside both AABBs
        let results = bvh.point_query(Vec2::new(-5.0, -5.0));
        assert_eq!(results.len(), 0);
    }

    #[test]
    fn test_bvh_aabb_query() {
        let aabbs = vec![
            Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0)),
            Aabb::new(Vec2::new(2.0, 2.0), Vec2::new(3.0, 3.0)),
        ];

        let handles = vec![ColliderHandle::new(1, 0), ColliderHandle::new(2, 0)];

        let body_handles = vec![BodyHandle::new(1, 0), BodyHandle::new(2, 0)];

        let mut bvh = Bvh::new();
        bvh.build(aabbs, handles, body_handles);

        // Query AABB that overlaps first AABB
        let query_aabb = Aabb::new(Vec2::new(-0.5, -0.5), Vec2::new(0.5, 0.5));
        let results = bvh.aabb_query(query_aabb);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].1, ColliderHandle::new(1, 0));

        // Query AABB that overlaps both
        let query_aabb = Aabb::new(Vec2::new(-2.0, -2.0), Vec2::new(4.0, 4.0));
        let results = bvh.aabb_query(query_aabb);
        assert_eq!(results.len(), 2);
    }

    #[test]
    fn test_ray_aabb_intersection() {
        // Ray pointing right
        let origin = Vec2::new(-5.0, 0.0);
        let direction = Vec2::new(1.0, 0.0);
        let aabb = Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0));

        let result = ray_aabb_intersection(origin, direction, aabb);
        assert!(result.is_some());
        let (t_min, t_max) = result.unwrap();
        assert!(t_min > 0.0);
        assert!(t_max > t_min);

        // Ray pointing left (away from AABB)
        let direction = Vec2::new(-1.0, 0.0);
        let result = ray_aabb_intersection(origin, direction, aabb);
        assert!(result.is_none());

        // Ray inside AABB
        let origin = Vec2::new(0.0, 0.0);
        let direction = Vec2::new(1.0, 0.0);
        let result = ray_aabb_intersection(origin, direction, aabb);
        assert!(result.is_some());
        let (t_min, _) = result.unwrap();
        assert!(t_min <= 0.0);
    }

    #[test]
    fn test_bvh_clear() {
        let aabbs = vec![Aabb::new(Vec2::new(-1.0, -1.0), Vec2::new(1.0, 1.0))];
        let handles = vec![ColliderHandle::new(1, 0)];
        let body_handles = vec![BodyHandle::new(1, 0)];

        let mut bvh = Bvh::new();
        bvh.build(aabbs, handles, body_handles);
        assert!(!bvh.is_empty());

        bvh.clear();
        assert!(bvh.is_empty());
        assert_eq!(bvh.node_count(), 0);
        assert_eq!(bvh.collider_count(), 0);
    }

    #[test]
    fn test_bvh_display() {
        let bvh = Bvh::new();
        let bvh_str = format!("{}", bvh);
        assert!(bvh_str.contains("BVH"));
    }
}
