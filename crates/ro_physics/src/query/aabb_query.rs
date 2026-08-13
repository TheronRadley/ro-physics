//! AABB query implementation.
//!
//! This module provides AABB query functionality for the physics world.

use std::fmt;

use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;
use crate::geometry::Aabb;

/// AABB query structure
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct AabbQuery {
    /// The AABB to query
    pub aabb: Aabb,
}

impl AabbQuery {
    /// Create a new AABB query
    #[inline]
    #[must_use]
    pub const fn new(aabb: Aabb) -> Self {
        Self { aabb }
    }

    /// Get the query AABB
    #[inline]
    #[must_use]
    pub const fn aabb(&self) -> Aabb {
        self.aabb
    }

    /// Set the query AABB
    #[inline]
    pub fn set_aabb(&mut self, aabb: Aabb) -> &mut Self {
        self.aabb = aabb;
        self
    }
}

impl Default for AabbQuery {
    fn default() -> Self {
        Self::new(Aabb::default())
    }
}

impl fmt::Display for AabbQuery {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "AabbQuery(aabb: {})", self.aabb)
    }
}

/// AABB query result
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct AabbQueryResult {
    /// Handle to the body that overlaps the AABB
    pub body_handle: BodyHandle,
    /// Handle to the collider that overlaps the AABB
    pub collider_handle: ColliderHandle,
}

impl AabbQueryResult {
    /// Create a new AABB query result
    #[inline]
    #[must_use]
    pub const fn new(body_handle: BodyHandle, collider_handle: ColliderHandle) -> Self {
        Self {
            body_handle,
            collider_handle,
        }
    }

    /// Get the body handle
    #[inline]
    #[must_use]
    pub const fn body_handle(&self) -> BodyHandle {
        self.body_handle
    }

    /// Get the collider handle
    #[inline]
    #[must_use]
    pub const fn collider_handle(&self) -> ColliderHandle {
        self.collider_handle
    }
}

impl Default for AabbQueryResult {
    fn default() -> Self {
        Self::new(BodyHandle::null(), ColliderHandle::null())
    }
}

impl fmt::Display for AabbQueryResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "AabbQueryResult(body: {}, collider: {})",
            self.body_handle, self.collider_handle
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::BodyHandle;
    use crate::collision::ColliderHandle;
    use crate::geometry::Aabb;
    use crate::math::Vec2;

    #[test]
    fn test_aabb_query() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let query = AabbQuery::new(aabb);
        assert_eq!(query.aabb(), aabb);
    }

    #[test]
    fn test_aabb_query_default() {
        let query = AabbQuery::default();
        assert_eq!(query.aabb(), Aabb::default());
    }

    #[test]
    fn test_aabb_query_set_aabb() {
        let mut query = AabbQuery::new(Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0)));
        let new_aabb = Aabb::new(Vec2::new(5.0, 5.0), Vec2::new(15.0, 15.0));
        query.set_aabb(new_aabb);
        assert_eq!(query.aabb(), new_aabb);
    }

    #[test]
    fn test_aabb_query_display() {
        let aabb = Aabb::new(Vec2::new(0.0, 0.0), Vec2::new(10.0, 10.0));
        let query = AabbQuery::new(aabb);
        let query_str = format!("{}", query);
        assert!(query_str.contains("AabbQuery"));
    }

    #[test]
    fn test_aabb_query_result() {
        let result = AabbQueryResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
        );

        assert_eq!(result.body_handle(), BodyHandle::new(1, 0));
        assert_eq!(result.collider_handle(), ColliderHandle::new(1, 0));
    }

    #[test]
    fn test_aabb_query_result_default() {
        let result = AabbQueryResult::default();
        assert!(result.body_handle().is_null());
        assert!(result.collider_handle().is_null());
    }

    #[test]
    fn test_aabb_query_result_display() {
        let result = AabbQueryResult::new(
            BodyHandle::new(1, 0),
            ColliderHandle::new(1, 0),
        );

        let result_str = format!("{}", result);
        assert!(result_str.contains("AabbQueryResult"));
    }
}
