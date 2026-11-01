//! Collision event types for the physics engine.
//!
//! This module provides event types for collision callbacks.

use std::fmt;

use crate::bodies::BodyHandle;
use crate::collision::ColliderHandle;

/// Type of collision event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub enum CollisionEventType {
    /// Collision started (bodies just touched)
    Started,
    /// Collision persisted (bodies were already touching and still are)
    Persisted,
    /// Collision ended (bodies separated)
    Ended,
}

impl CollisionEventType {
    /// Check if this type is [`CollisionEventType::Started`]
    #[inline]
    #[must_use]
    pub const fn is_start(self) -> bool {
        matches!(self, Self::Started)
    }

    /// Check if this type is [`CollisionEventType::Persisted`]
    #[inline]
    #[must_use]
    pub const fn is_persist(self) -> bool {
        matches!(self, Self::Persisted)
    }

    /// Check if this type is [`CollisionEventType::Ended`]
    #[inline]
    #[must_use]
    pub const fn is_end(self) -> bool {
        matches!(self, Self::Ended)
    }
}

impl fmt::Display for CollisionEventType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CollisionEventType::Started => write!(f, "Started"),
            CollisionEventType::Persisted => write!(f, "Persisted"),
            CollisionEventType::Ended => write!(f, "Ended"),
        }
    }
}

/// Collision event information
///
/// Contains information about a collision between two bodies.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionEvent {
    /// Type of collision event
    pub event_type: CollisionEventType,
    /// Handle to the first body
    pub body_a: BodyHandle,
    /// Handle to the second body
    pub body_b: BodyHandle,
    /// Handle to the first collider
    pub collider_a: ColliderHandle,
    /// Handle to the second collider
    pub collider_b: ColliderHandle,
    /// Whether body A is a sensor
    pub is_sensor_a: bool,
    /// Whether body B is a sensor
    pub is_sensor_b: bool,
}

impl CollisionEvent {
    /// Create a new collision event
    #[inline]
    #[must_use]
    pub fn new(
        event_type: CollisionEventType,
        body_a: BodyHandle,
        body_b: BodyHandle,
        collider_a: ColliderHandle,
        collider_b: ColliderHandle,
        is_sensor_a: bool,
        is_sensor_b: bool,
    ) -> Self {
        Self {
            event_type,
            body_a,
            body_b,
            collider_a,
            collider_b,
            is_sensor_a,
            is_sensor_b,
        }
    }

    /// Check if this is a start event
    #[inline]
    #[must_use]
    pub fn is_start(&self) -> bool {
        self.event_type == CollisionEventType::Started
    }

    /// Check if this is a persist event
    #[inline]
    #[must_use]
    pub fn is_persist(&self) -> bool {
        self.event_type == CollisionEventType::Persisted
    }

    /// Check if this is an end event
    #[inline]
    #[must_use]
    pub fn is_end(&self) -> bool {
        self.event_type == CollisionEventType::Ended
    }

    /// Check if either collider is a sensor
    #[inline]
    #[must_use]
    pub fn is_sensor(&self) -> bool {
        self.is_sensor_a || self.is_sensor_b
    }

    /// Get the other body given one body handle
    #[inline]
    #[must_use]
    pub fn other_body(&self, body: BodyHandle) -> BodyHandle {
        if body == self.body_a {
            self.body_b
        } else {
            self.body_a
        }
    }
}

impl Default for CollisionEvent {
    fn default() -> Self {
        Self {
            event_type: CollisionEventType::Started,
            body_a: BodyHandle::null(),
            body_b: BodyHandle::null(),
            collider_a: ColliderHandle::null(),
            collider_b: ColliderHandle::null(),
            is_sensor_a: false,
            is_sensor_b: false,
        }
    }
}

impl fmt::Display for CollisionEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "CollisionEvent(type: {}, body_a: {}, body_b: {})",
            self.event_type, self.body_a, self.body_b
        )
    }
}

/// Trait for collision event callbacks
pub trait CollisionEventHandler: Send + Sync {
    /// Called when a collision event occurs
    fn on_collision(&mut self, event: CollisionEvent);
}

/// Simple collision event handler that just collects events
#[derive(Debug, Default, Clone)]
pub struct SimpleCollisionEventHandler {
    events: std::sync::Arc<std::sync::Mutex<Vec<CollisionEvent>>>,
}

impl SimpleCollisionEventHandler {
    /// Create a new simple collision event handler
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            events: std::sync::Arc::new(std::sync::Mutex::new(Vec::new())),
        }
    }

    /// Get all collected events
    #[inline]
    #[must_use]
    pub fn events(&self) -> Vec<CollisionEvent> {
        self.events.lock().unwrap().clone()
    }

    /// Clear all collected events
    #[inline]
    pub fn clear(&self) {
        self.events.lock().unwrap().clear();
    }
}

impl CollisionEventHandler for SimpleCollisionEventHandler {
    fn on_collision(&mut self, event: CollisionEvent) {
        self.events.lock().unwrap().push(event);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collision_event_type() {
        assert!(CollisionEventType::Started.is_start());
        assert!(!CollisionEventType::Started.is_persist());
        assert!(!CollisionEventType::Started.is_end());

        assert!(!CollisionEventType::Persisted.is_start());
        assert!(CollisionEventType::Persisted.is_persist());
        assert!(!CollisionEventType::Persisted.is_end());

        assert!(!CollisionEventType::Ended.is_start());
        assert!(!CollisionEventType::Ended.is_persist());
        assert!(CollisionEventType::Ended.is_end());
    }

    #[test]
    fn test_collision_event() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);
        let collider_a = ColliderHandle::new(1, 0);
        let collider_b = ColliderHandle::new(2, 0);

        let event = CollisionEvent::new(
            CollisionEventType::Started,
            handle_a,
            handle_b,
            collider_a,
            collider_b,
            false,
            false,
        );

        assert!(event.is_start());
        assert!(!event.is_sensor());
        assert_eq!(event.other_body(handle_a), handle_b);
        assert_eq!(event.other_body(handle_b), handle_a);
    }

    #[test]
    fn test_sensor_event() {
        let handle_a = BodyHandle::new(1, 0);
        let handle_b = BodyHandle::new(2, 0);
        let collider_a = ColliderHandle::new(1, 0);
        let collider_b = ColliderHandle::new(2, 0);

        let event = CollisionEvent::new(
            CollisionEventType::Started,
            handle_a,
            handle_b,
            collider_a,
            collider_b,
            true,
            false,
        );

        assert!(event.is_sensor());
    }

    #[test]
    fn test_default() {
        let event = CollisionEvent::default();
        assert!(event.body_a.is_null());
        assert!(event.body_b.is_null());
    }

    #[test]
    fn test_display() {
        let event = CollisionEvent::default();
        let event_str = format!("{}", event);
        assert!(event_str.contains("CollisionEvent"));
    }

    #[test]
    fn test_simple_handler() {
        let mut handler = SimpleCollisionEventHandler::new();
        
        let event = CollisionEvent::new(
            CollisionEventType::Started,
            BodyHandle::new(1, 0),
            BodyHandle::new(2, 0),
            ColliderHandle::new(1, 0),
            ColliderHandle::new(2, 0),
            false,
            false,
        );

        handler.on_collision(event);
        
        let events = handler.events();
        assert_eq!(events.len(), 1);
        assert!(events[0].is_start());

        handler.clear();
        assert_eq!(handler.events().len(), 0);
    }
}
