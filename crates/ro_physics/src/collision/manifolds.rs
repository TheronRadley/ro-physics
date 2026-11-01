//! Contact manifold implementation.
//!
//! This module provides the contact manifold type for representing
//! collision information between two bodies.
//!
//! # Conventions
//!
//! * A manifold is generated for the pair **(body A, body B)** and the normal
//!   points **from A to B**.
//! * [`Contact::penetration`] is positive while the shapes overlap and negative
//!   when they are separated (speculative contacts).
//! * Impulses are applied as `-normal * impulse` on body A and
//!   `+normal * impulse` on body B.

use std::fmt;

use crate::bodies::BodyHandle;
use crate::math::Vec2;
use crate::collision::ColliderHandle;

/// Maximum number of contact points per manifold
const MAX_CONTACTS: usize = 4;

/// A single contact point between two bodies.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Contact {
    /// World position of the contact point
    pub position: Vec2,
    /// Contact normal (from body A to body B)
    pub normal: Vec2,
    /// Penetration depth (positive while overlapping)
    pub penetration: f32,
    /// Normal impulse accumulated during the last solve
    pub normal_impulse: f32,
    /// Tangent (friction) impulse accumulated during the last solve
    pub tangent_impulse: f32,
    /// Contact identifier used to match contacts between frames
    pub contact_id: ContactId,
}

impl Contact {
    /// Create a new contact
    #[inline]
    #[must_use]
    pub fn new(
        position: Vec2,
        normal: Vec2,
        penetration: f32,
        normal_impulse: f32,
        tangent_impulse: f32,
        contact_id: ContactId,
    ) -> Self {
        Self {
            position,
            normal,
            penetration,
            normal_impulse,
            tangent_impulse,
            contact_id,
        }
    }

    /// Create a contact without accumulated impulses
    #[inline]
    #[must_use]
    pub fn without_impulses(position: Vec2, normal: Vec2, penetration: f32, contact_id: ContactId) -> Self {
        Self::new(position, normal, penetration, 0.0, 0.0, contact_id)
    }

    /// Get the position
    #[inline]
    #[must_use]
    pub const fn position(&self) -> Vec2 {
        self.position
    }

    /// Get the normal
    #[inline]
    #[must_use]
    pub const fn normal(&self) -> Vec2 {
        self.normal
    }

    /// Get the penetration
    #[inline]
    #[must_use]
    pub const fn penetration(&self) -> f32 {
        self.penetration
    }

    /// Get the normal impulse
    #[inline]
    #[must_use]
    pub const fn normal_impulse(&self) -> f32 {
        self.normal_impulse
    }

    /// Get the tangent impulse
    #[inline]
    #[must_use]
    pub const fn tangent_impulse(&self) -> f32 {
        self.tangent_impulse
    }

    /// Get the contact ID
    #[inline]
    #[must_use]
    pub const fn contact_id(&self) -> ContactId {
        self.contact_id
    }

    /// Set the accumulated normal impulse
    #[inline]
    pub fn set_normal_impulse(&mut self, impulse: f32) {
        self.normal_impulse = impulse;
    }

    /// Set the accumulated tangent impulse
    #[inline]
    pub fn set_tangent_impulse(&mut self, impulse: f32) {
        self.tangent_impulse = impulse;
    }

    /// Check if the contact is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.normal.is_finite() && self.position.is_finite() && self.penetration.is_finite()
    }

    /// Check if the contact is separating
    #[inline]
    #[must_use]
    pub fn is_separating(&self) -> bool {
        self.penetration < 0.0
    }
}

impl Default for Contact {
    fn default() -> Self {
        Self {
            position: Vec2::zero(),
            normal: Vec2::unit_x(),
            penetration: 0.0,
            normal_impulse: 0.0,
            tangent_impulse: 0.0,
            contact_id: ContactId::default(),
        }
    }
}

impl fmt::Display for Contact {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Contact(pos: {}, normal: {}, penetration: {:.4})",
            self.position, self.normal, self.penetration
        )
    }
}

/// A contact identifier for warm starting.
///
/// Stores the features (vertex or edge indices) that generated the contact on
/// each shape plus a flag recording whether the pair was flipped. Contacts
/// whose ids match between frames are considered the same contact, which lets
/// the solver seed them with last frame's impulses.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactId {
    /// Feature index on body A
    pub feature_a: u32,
    /// Feature index on body B
    pub feature_b: u32,
    /// Whether the contact is flipped
    pub flipped: bool,
}

impl ContactId {
    /// Create a new contact ID
    #[inline]
    #[must_use]
    pub const fn new(feature_a: u32, feature_b: u32) -> Self {
        Self {
            feature_a,
            feature_b,
            flipped: false,
        }
    }

    /// Create a flipped contact ID
    #[inline]
    #[must_use]
    pub fn flipped(feature_a: u32, feature_b: u32) -> Self {
        Self {
            feature_a,
            feature_b,
            flipped: true,
        }
    }

    /// Flip the contact ID
    #[inline]
    #[must_use]
    pub fn flip(self) -> Self {
        Self {
            feature_a: self.feature_b,
            feature_b: self.feature_a,
            flipped: !self.flipped,
        }
    }

    /// The identifier of this contact as seen from the other shape
    ///
    /// Two ids compare equal when they describe the same feature pair,
    /// regardless of which shape was A and which was B.
    #[inline]
    #[must_use]
    pub const fn key(self) -> (u32, u32) {
        if self.flipped {
            (self.feature_b, self.feature_a)
        } else {
            (self.feature_a, self.feature_b)
        }
    }
}

impl fmt::Display for ContactId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ContactId(a: {}, b: {}, flipped: {})",
            self.feature_a, self.feature_b, self.flipped
        )
    }
}

/// A contact manifold containing multiple contact points.
///
/// A manifold represents the collision information between two bodies,
/// including all contact points, the collision normal, and accumulated impulses.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct ContactManifold {
    /// Handle of body A
    body_a: BodyHandle,
    /// Handle of body B
    body_b: BodyHandle,
    /// Handle of collider A
    collider_a: ColliderHandle,
    /// Handle of collider B
    collider_b: ColliderHandle,
    /// Contact points
    contacts: Vec<Contact>,
    /// Normal vector (from body A to body B)
    normal: Vec2,
    /// Friction coefficient (combined from both colliders)
    friction: f32,
    /// Restitution coefficient (combined from both colliders)
    restitution: f32,
    /// Whether either collider is a sensor (no impulses are applied)
    sensor: bool,
}

impl ContactManifold {
    /// Create a new empty contact manifold without associated handles
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            body_a: BodyHandle::null(),
            body_b: BodyHandle::null(),
            collider_a: ColliderHandle::null(),
            collider_b: ColliderHandle::null(),
            contacts: Vec::with_capacity(MAX_CONTACTS),
            normal: Vec2::unit_x(),
            friction: 0.0,
            restitution: 0.0,
            sensor: false,
        }
    }

    /// Create a manifold for a specific body/collider pair
    #[inline]
    #[must_use]
    pub fn between(
        body_a: BodyHandle,
        body_b: BodyHandle,
        collider_a: ColliderHandle,
        collider_b: ColliderHandle,
    ) -> Self {
        Self {
            body_a,
            body_b,
            collider_a,
            collider_b,
            ..Self::new()
        }
    }

    /// Create a manifold with a single contact
    #[inline]
    #[must_use]
    pub fn with_contact(
        position: Vec2,
        normal: Vec2,
        penetration: f32,
        friction: f32,
        restitution: f32,
    ) -> Self {
        let mut manifold = Self::new();
        manifold.add_contact(position, normal, penetration, 0.0, 0.0, ContactId::default());
        manifold.friction = friction;
        manifold.restitution = restitution;
        manifold
    }

    /// Add a contact to the manifold
    ///
    /// Contacts beyond [`MAX_CONTACTS`] are ignored.
    #[inline]
    pub fn add_contact(
        &mut self,
        position: Vec2,
        normal: Vec2,
        penetration: f32,
        normal_impulse: f32,
        tangent_impulse: f32,
        contact_id: ContactId,
    ) {
        if self.contacts.len() >= MAX_CONTACTS {
            return;
        }

        self.contacts.push(Contact::new(
            position,
            normal,
            penetration,
            normal_impulse,
            tangent_impulse,
            contact_id,
        ));
        self.normal = normal;
    }

    /// Get the number of contact points
    #[inline]
    #[must_use]
    pub fn contact_count(&self) -> usize {
        self.contacts.len()
    }

    /// Get a contact by index
    #[inline]
    #[must_use]
    pub fn contact(&self, index: usize) -> Contact {
        if index < self.contacts.len() {
            self.contacts[index]
        } else {
            Contact::default()
        }
    }

    /// Get all contacts
    #[inline]
    #[must_use]
    pub fn contacts(&self) -> &[Contact] {
        &self.contacts
    }

    /// Get mutable access to all contacts
    #[inline]
    #[must_use]
    pub fn contacts_mut(&mut self) -> &mut [Contact] {
        &mut self.contacts
    }

    /// Get the normal vector
    #[inline]
    #[must_use]
    pub fn normal(&self) -> Vec2 {
        self.normal
    }

    /// Set the normal vector
    #[inline]
    pub fn set_normal(&mut self, normal: Vec2) {
        self.normal = normal;
    }

    /// Get the handle of body A
    #[inline]
    #[must_use]
    pub const fn body_a(&self) -> BodyHandle {
        self.body_a
    }

    /// Get the handle of body B
    #[inline]
    #[must_use]
    pub const fn body_b(&self) -> BodyHandle {
        self.body_b
    }

    /// Get the handle of collider A
    #[inline]
    #[must_use]
    pub const fn collider_a(&self) -> ColliderHandle {
        self.collider_a
    }

    /// Get the handle of collider B
    #[inline]
    #[must_use]
    pub const fn collider_b(&self) -> ColliderHandle {
        self.collider_b
    }

    /// Set the body and collider handles of the pair
    #[inline]
    pub fn set_pair(
        &mut self,
        body_a: BodyHandle,
        body_b: BodyHandle,
        collider_a: ColliderHandle,
        collider_b: ColliderHandle,
    ) {
        self.body_a = body_a;
        self.body_b = body_b;
        self.collider_a = collider_a;
        self.collider_b = collider_b;
    }

    /// Get the friction coefficient
    #[inline]
    #[must_use]
    pub fn friction(&self) -> f32 {
        self.friction
    }

    /// Set the friction coefficient
    #[inline]
    pub fn set_friction(&mut self, friction: f32) {
        self.friction = friction.clamp(0.0, 1.0);
    }

    /// Get the restitution coefficient
    #[inline]
    #[must_use]
    pub fn restitution(&self) -> f32 {
        self.restitution
    }

    /// Set the restitution coefficient
    #[inline]
    pub fn set_restitution(&mut self, restitution: f32) {
        self.restitution = restitution.clamp(0.0, 1.0);
    }

    /// Whether either collider of the pair is a sensor
    ///
    /// Sensors generate collision events but never receive impulses.
    #[inline]
    #[must_use]
    pub fn is_sensor(&self) -> bool {
        self.sensor
    }

    /// Mark the pair as involving a sensor
    #[inline]
    pub fn set_sensor(&mut self, sensor: bool) {
        self.sensor = sensor;
    }

    /// Check if the manifold is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        !self.contacts.is_empty() && self.normal.is_finite()
    }

    /// Flip the manifold (swap bodies A and B)
    #[inline]
    pub fn flip(&mut self) {
        for contact in &mut self.contacts {
            contact.normal = -contact.normal;
            contact.contact_id = contact.contact_id.flip();
        }
        self.normal = -self.normal;

        std::mem::swap(&mut self.body_a, &mut self.body_b);
        std::mem::swap(&mut self.collider_a, &mut self.collider_b);
    }

    /// Clear the manifold
    #[inline]
    pub fn clear(&mut self) {
        self.contacts.clear();
    }

    /// Calculate the average contact point
    #[inline]
    #[must_use]
    pub fn average_contact_point(&self) -> Vec2 {
        if self.contacts.is_empty() {
            return Vec2::zero();
        }

        let mut sum = Vec2::zero();
        for contact in &self.contacts {
            sum += contact.position;
        }
        sum / self.contacts.len() as f32
    }

    /// Calculate the average penetration
    #[inline]
    #[must_use]
    pub fn average_penetration(&self) -> f32 {
        if self.contacts.is_empty() {
            return 0.0;
        }

        let mut sum = 0.0;
        for contact in &self.contacts {
            sum += contact.penetration;
        }
        sum / self.contacts.len() as f32
    }

    /// Merge contacts that are very close together
    #[inline]
    pub fn merge_contacts(&mut self, tolerance: f32) {
        if self.contacts.len() <= 1 {
            return;
        }

        let count = self.contacts.len();
        let mut merged: Vec<Contact> = Vec::with_capacity(MAX_CONTACTS);
        let mut used = vec![false; count];

        for (i, contact) in self.contacts.iter().copied().enumerate().take(count) {
            if used[i] {
                continue;
            }

            let mut merged_contact = contact;
            used[i] = true;

            for (j, contact_j) in self
                .contacts
                .iter()
                .copied()
                .enumerate()
                .skip(i + 1)
                .take(count.saturating_sub(i + 1))
            {
                if used[j] {
                    continue;
                }
                let delta = contact_j.position - merged_contact.position;

                if delta.length_squared() <= tolerance * tolerance {
                    merged_contact.position = (merged_contact.position + contact_j.position) / 2.0;
                    merged_contact.penetration = merged_contact
                        .penetration
                        .max(contact_j.penetration);
                    // Keep the largest accumulated impulse so warm starting
                    // does not lose energy when contacts collapse.
                    merged_contact.normal_impulse += contact_j.normal_impulse;
                    merged_contact.tangent_impulse += contact_j.tangent_impulse;
                    used[j] = true;
                }
            }

            merged.push(merged_contact);
        }

        self.contacts = merged;
    }

    /// Copy warm-start impulses from the manifold of the previous step
    ///
    /// Contacts are matched by their [`ContactId`]; unmatched contacts start
    /// with zero impulse. This is what makes resting stacks converge quickly
    /// instead of re-solving from scratch every frame.
    pub fn warm_start_from(&mut self, previous: &Self) {
        for contact in &mut self.contacts {
            let key = contact.contact_id.key();
            if let Some(match_) = previous
                .contacts
                .iter()
                .find(|old| old.contact_id.key() == key)
            {
                contact.normal_impulse = match_.normal_impulse;
                contact.tangent_impulse = match_.tangent_impulse;
            } else {
                contact.normal_impulse = 0.0;
                contact.tangent_impulse = 0.0;
            }
        }
    }
}

impl Default for ContactManifold {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for ContactManifold {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ContactManifold(contacts: {}, normal: {})",
            self.contacts.len(),
            self.normal
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_manifold() {
        let manifold = ContactManifold::new();
        assert_eq!(manifold.contact_count(), 0);
        assert!(!manifold.is_valid());
    }

    #[test]
    fn test_add_contact() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(0.0, 1.0),
            Vec2::unit_y(),
            0.25,
            0.0,
            0.0,
            ContactId::new(1, 2),
        );

        assert_eq!(manifold.contact_count(), 1);
        assert!(manifold.is_valid());
        assert_eq!(manifold.contact(0).penetration(), 0.25);
        assert_eq!(manifold.normal(), Vec2::unit_y());
    }

    #[test]
    fn test_contact_limit() {
        let mut manifold = ContactManifold::new();
        for i in 0..(MAX_CONTACTS + 2) {
            manifold.add_contact(
                Vec2::new(i as f32, 0.0),
                Vec2::unit_y(),
                0.1,
                0.0,
                0.0,
                ContactId::new(i as u32, 0),
            );
        }

        assert_eq!(manifold.contact_count(), MAX_CONTACTS);
    }

    #[test]
    fn test_warm_start_matching() {
        let mut previous = ContactManifold::new();
        previous.add_contact(
            Vec2::zero(),
            Vec2::unit_y(),
            0.1,
            3.0,
            1.5,
            ContactId::new(0, 1),
        );
        previous.add_contact(
            Vec2::new(1.0, 0.0),
            Vec2::unit_y(),
            0.1,
            4.0,
            0.5,
            ContactId::new(7, 3),
        );

        let mut current = ContactManifold::new();
        // Matches the first previous contact
        current.add_contact(Vec2::zero(), Vec2::unit_y(), 0.1, 0.0, 0.0, ContactId::new(0, 1));
        // New contact without a previous counterpart
        current.add_contact(Vec2::new(5.0, 0.0), Vec2::unit_y(), 0.1, 0.0, 0.0, ContactId::new(9, 9));

        current.warm_start_from(&previous);

        assert_eq!(current.contact(0).normal_impulse(), 3.0);
        assert_eq!(current.contact(0).tangent_impulse(), 1.5);
        assert_eq!(current.contact(1).normal_impulse(), 0.0);
    }

    #[test]
    fn test_warm_start_matches_flipped_contacts() {
        let mut previous = ContactManifold::new();
        previous.add_contact(
            Vec2::zero(),
            Vec2::unit_y(),
            0.1,
            2.0,
            0.0,
            ContactId::new(4, 2),
        );

        let mut current = ContactManifold::new();
        current.add_contact(
            Vec2::zero(),
            Vec2::unit_y(),
            0.1,
            0.0,
            0.0,
            ContactId::flipped(2, 4),
        );

        current.warm_start_from(&previous);
        assert_eq!(current.contact(0).normal_impulse(), 2.0);
    }

    #[test]
    fn test_flip_swaps_bodies() {
        let a = BodyHandle::new(1, 0);
        let b = BodyHandle::new(2, 0);
        let ca = ColliderHandle::new(3, 0);
        let cb = ColliderHandle::new(4, 0);

        let mut manifold = ContactManifold::between(a, b, ca, cb);
        manifold.add_contact(
            Vec2::zero(),
            Vec2::unit_y(),
            0.1,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );

        manifold.flip();

        assert_eq!(manifold.body_a(), b);
        assert_eq!(manifold.body_b(), a);
        assert_eq!(manifold.collider_a(), cb);
        assert_eq!(manifold.collider_b(), ca);
        assert_eq!(manifold.normal(), Vec2::new(0.0, -1.0));
        assert_eq!(manifold.contact(0).normal(), Vec2::new(0.0, -1.0));
    }

    #[test]
    fn test_merge_contacts() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(Vec2::zero(), Vec2::unit_y(), 0.1, 1.0, 0.0, ContactId::new(0, 0));
        manifold.add_contact(Vec2::new(0.001, 0.0), Vec2::unit_y(), 0.3, 2.0, 0.0, ContactId::new(1, 0));
        manifold.add_contact(Vec2::new(10.0, 0.0), Vec2::unit_y(), 0.2, 0.0, 0.0, ContactId::new(2, 0));

        manifold.merge_contacts(0.01);

        assert_eq!(manifold.contact_count(), 2);
        assert!(manifold.contact(0).penetration() >= 0.3);
        assert_eq!(manifold.contact(0).normal_impulse(), 3.0);
    }
}
