//! Contact manifold implementation.
//!
//! This module provides the contact manifold type for representing
//! collision information between two bodies.

use std::fmt;

use crate::constants::EPSILON;
use crate::math::Vec2;

/// Maximum number of contact points per manifold
const MAX_CONTACTS: usize = 4;

/// A single contact point between two bodies.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Contact {
    /// World position of the contact point
    pub position: Vec2,
    /// Normal vector (from body A to body B)
    pub normal: Vec2,
    /// Penetration depth (positive if overlapping)
    pub penetration: f32,
    /// Normal impulse accumulated
    pub normal_impulse: f32,
    /// Tangent impulse accumulated
    pub tangent_impulse: f32,
    /// Contact identifier for warm starting
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

    /// Check if the contact is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.penetration >= 0.0
            && self.normal.is_finite()
            && self.position.is_finite()
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
/// Used to match contacts between frames for stable simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
}

impl Default for ContactId {
    fn default() -> Self {
        Self {
            feature_a: 0,
            feature_b: 0,
            flipped: false,
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
    /// Contact points
    contacts: Vec<Contact>,
    /// Normal vector (from body A to body B)
    normal: Vec2,
    /// Number of contact points
    contact_count: usize,
    /// Friction coefficient
    friction: f32,
    /// Restitution coefficient
    restitution: f32,
    /// Whether the manifold is valid
    valid: bool,
}

impl ContactManifold {
    /// Create a new empty contact manifold
    #[inline]
    #[must_use]
    pub fn new() -> Self {
        Self {
            contacts: Vec::with_capacity(MAX_CONTACTS),
            normal: Vec2::unit_x(),
            contact_count: 0,
            friction: 0.0,
            restitution: 0.0,
            valid: false,
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
        self.contact_count = self.contacts.len();
        self.normal = normal;
        self.valid = true;
    }

    /// Get the number of contact points
    #[inline]
    #[must_use]
    pub fn contact_count(&self) -> usize {
        self.contact_count
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

    /// Check if the manifold is valid
    #[inline]
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.valid && self.contact_count > 0
    }

    /// Flip the manifold (swap bodies A and B)
    #[inline]
    pub fn flip(&mut self) {
        for contact in &mut self.contacts {
            contact.normal = -contact.normal;
            contact.contact_id = contact.contact_id.flip();
        }
        self.normal = -self.normal;
    }

    /// Clear the manifold
    #[inline]
    pub fn clear(&mut self) {
        self.contacts.clear();
        self.contact_count = 0;
        self.valid = false;
    }

    /// Calculate the average contact point
    #[inline]
    #[must_use]
    pub fn average_contact_point(&self) -> Vec2 {
        if self.contact_count == 0 {
            return Vec2::zero();
        }

        let mut sum = Vec2::zero();
        for contact in &self.contacts {
            sum += contact.position;
        }
        sum / self.contact_count as f32
    }

    /// Calculate the average penetration
    #[inline]
    #[must_use]
    pub fn average_penetration(&self) -> f32 {
        if self.contact_count == 0 {
            return 0.0;
        }

        let mut sum = 0.0;
        for contact in &self.contacts {
            sum += contact.penetration;
        }
        sum / self.contact_count as f32
    }

    /// Merge contacts that are very close together
    #[inline]
    pub fn merge_contacts(&mut self, tolerance: f32) {
        if self.contact_count <= 1 {
            return;
        }

        let mut merged = Vec::with_capacity(MAX_CONTACTS);
        let mut used = vec![false; self.contact_count];

        for i in 0..self.contact_count {
            if used[i] {
                continue;
            }

            let contact_i = self.contacts[i];
            let mut merged_contact = contact_i;
            used[i] = true;

            for j in (i + 1)..self.contact_count {
                if used[j] {
                    continue;
                }

                let contact_j = self.contacts[j];
                let delta = contact_j.position - merged_contact.position;

                if delta.length_squared() <= tolerance * tolerance {
                    // Merge the contacts
                    merged_contact.position = (merged_contact.position + contact_j.position) / 2.0;
                    merged_contact.penetration = merged_contact
                        .penetration
                        .max(contact_j.penetration);
                    used[j] = true;
                }
            }

            merged.push(merged_contact);
        }

        self.contacts = merged;
        self.contact_count = self.contacts.len();
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
            self.contact_count, self.normal
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::utils::approx_eq;

    #[test]
    fn test_contact() {
        let contact = Contact::new(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            10.0,
            5.0,
            ContactId::new(0, 1),
        );

        assert_eq!(contact.position(), Vec2::new(1.0, 2.0));
        assert_eq!(contact.normal(), Vec2::new(0.0, 1.0));
        assert!(approx_eq(contact.penetration(), 0.5));
        assert!(approx_eq(contact.normal_impulse(), 10.0));
        assert!(approx_eq(contact.tangent_impulse(), 5.0));
    }

    #[test]
    fn test_contact_id() {
        let id1 = ContactId::new(0, 1);
        let id2 = ContactId::flipped(0, 1);
        let id3 = id1.flip();

        assert!(!id1.flipped);
        assert!(id2.flipped);
        assert_eq!(id3.feature_a, 1);
        assert_eq!(id3.feature_b, 0);
        assert!(id3.flipped);
    }

    #[test]
    fn test_manifold() {
        let mut manifold = ContactManifold::new();

        assert_eq!(manifold.contact_count(), 0);
        assert!(!manifold.is_valid());

        manifold.add_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );

        assert_eq!(manifold.contact_count(), 1);
        assert!(manifold.is_valid());

        manifold.add_contact(
            Vec2::new(1.1, 2.1),
            Vec2::new(0.0, 1.0),
            0.6,
            0.0,
            0.0,
            ContactId::new(0, 2),
        );

        assert_eq!(manifold.contact_count(), 2);
    }

    #[test]
    fn test_manifold_with_contact() {
        let manifold = ContactManifold::with_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.6,
            0.2,
        );

        assert_eq!(manifold.contact_count(), 1);
        assert!(approx_eq(manifold.friction(), 0.6));
        assert!(approx_eq(manifold.restitution(), 0.2));
    }

    #[test]
    fn test_manifold_flip() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );

        let normal_before = manifold.normal();
        manifold.flip();
        let normal_after = manifold.normal();

        assert_eq!(normal_after, -normal_before);
    }

    #[test]
    fn test_manifold_clear() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );

        assert!(manifold.is_valid());

        manifold.clear();
        assert!(!manifold.is_valid());
        assert_eq!(manifold.contact_count(), 0);
    }

    #[test]
    fn test_average_contact_point() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );
        manifold.add_contact(
            Vec2::new(3.0, 4.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 2),
        );

        let avg = manifold.average_contact_point();
        assert_eq!(avg, Vec2::new(2.0, 3.0));
    }

    #[test]
    fn test_average_penetration() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );
        manifold.add_contact(
            Vec2::new(3.0, 4.0),
            Vec2::new(0.0, 1.0),
            0.7,
            0.0,
            0.0,
            ContactId::new(0, 2),
        );

        let avg = manifold.average_penetration();
        assert!(approx_eq(avg, 0.6));
    }

    #[test]
    fn test_merge_contacts() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );
        manifold.add_contact(
            Vec2::new(1.01, 2.01),
            Vec2::new(0.0, 1.0),
            0.51,
            0.0,
            0.0,
            ContactId::new(0, 2),
        );

        assert_eq!(manifold.contact_count(), 2);

        manifold.merge_contacts(0.1);
        assert_eq!(manifold.contact_count(), 1);
    }

    #[test]
    fn test_default() {
        let manifold = ContactManifold::default();
        assert_eq!(manifold.contact_count(), 0);
        assert!(!manifold.is_valid());
    }

    #[test]
    fn test_display() {
        let mut manifold = ContactManifold::new();
        manifold.add_contact(
            Vec2::new(1.0, 2.0),
            Vec2::new(0.0, 1.0),
            0.5,
            0.0,
            0.0,
            ContactId::new(0, 1),
        );

        let manifold_str = format!("{}", manifold);
        assert!(manifold_str.contains("ContactManifold"));
    }
}
