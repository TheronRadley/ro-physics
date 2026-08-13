//! Point query implementation.
//!
//! This module provides point query functionality for the physics world.

use std::fmt;

use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;
use crate::math::Vec2;

/// Point query structure
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct PointQuery {
    /// The point to query
    pub point: Vec2,
}

impl PointQuery {
    /// Create a new point query
    #[inline]
    #[must_use]
    pub const fn new(point: Vec2) -> Self {
        Self { point }
    }

    /// Get the query point
    #[inline]
    #[must_use]
    pub const fn point(&self) -> Vec2 {
        self.point
    }

    /// Set the query point
    #[inline]
    pub fn set_point(&mut self, point: Vec2) -> &mut Self {
        self.point = point;
        self
    }
}

impl Default for PointQuery {
    fn default() -> Self {
        Self::new(Vec2::zero())
    }
}

impl fmt::Display for PointQuery {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PointQuery(point: {})", self.point)
    }
}

/// Result of a point query
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct PointQueryResult {
    /// Handle to the body that contains the point
    pub body_handle: BodyHandle,
    /// Handle to the collider that contains the point
    pub collider_handle: ColliderHandle,
    /// Whether the point is inside the collider
    pub is_inside: bool,
    /// Distance from point to collider surface (if outside)
    pub distance: f32,
}

impl PointQueryResult {
    /// Create a new point query result
    #[inline]
    #[must_use]
    pub const fn new(
        body_handle: BodyHandle,
        collider_handle: ColliderHandle,
        is_inside: bool,
        distance: f32,
    ) -> Self {
        Self {
            body_handle,
            collider_handle,
            is_inside,
            distance,
        }
    }

    /// Check if the point is inside any collider
    #[inline]
    #[must_use]
    pub const fn is_inside_any(&self) -> bool {
        self.is_inside
    }
}

impl Default for PointQueryResult {
    fn default() -> Self {
        Self::new(BodyHandle::null(), ColliderHandle::null(), false, 0.0)
    }
}

impl fmt::Display for PointQueryResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "PointQueryResult(body: {}, collider: {}, inside: {}, distance: {:.4})",
            self.body_handle, self.collider_handle, self.is_inside, self.distance
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::collision::ColliderHandle;

    #[test]
    fn test_point_query() {
        let query = PointQuery::new(Vec2::new(1.0, 2.0));
        assert_eq!(query.point(), Vec2::new(1.0, 2.0));
    }

    #[test]
    fn test_point_query_default() {
        let query = PointQuery::default();
        assert_eq!(query.point(), Vec2::zero());
    }

    #[test]
    fn test_point_query_set_point() {
        let mut query = PointQuery::new(Vec2::new(1.0, 2.0));
        query.set_point(Vec2::new(3.0, 4.0));
        assert_eq!(query.point(), Vec2::new(3.0, 4.0));
    }

    #[test]
    fn test_point_query_display() {
        let query = PointQuery::new(Vec2::new(1.0, 2.0));
        let query_str = format!("{}", query);
        assert!(query_str.contains("PointQuery"));
        assert!(query_str.contains("1.0"));
        assert!(query_str.contains("2.0"));
    }

    #[test]
    fn test_point_query_result() {
        let result = PointQueryResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
            true,
            0.0,
        );

        assert_eq!(result.body_handle, BodyHandle::new(1, 0));
        assert_eq!(result.collider_handle, ColliderHandle::new(1, 0));
        assert!(result.is_inside_any());
        assert!(result.is_inside);
    }

    #[test]
    fn test_point_query_result_default() {
        let result = PointQueryResult::default();
        assert!(result.body_handle.is_null());
        assert!(result.collider_handle.is_null());
        assert!(!result.is_inside_any());
    }

    #[test]
    fn test_point_query_result_display() {
        let result = PointQueryResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
            true,
            0.5,
        );

        let result_str = format!("{}", result);
        assert!(result_str.contains("PointQueryResult"));
        assert!(result_str.contains("true"));
    }
}
