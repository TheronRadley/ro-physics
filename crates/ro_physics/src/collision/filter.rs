//! Collision filtering for the physics engine.
//!
//! This module provides collision filtering based on categories and masks.

use std::fmt;

/// Collision filter for determining which bodies can collide.
///
/// Uses a category/mask system similar to Box2D:
/// - Each body belongs to one or more categories (bits 0-15)
/// - Each body has a mask specifying which categories it can collide with (bits 0-15)
/// - Two bodies collide if (categoryA & maskB) != 0 and (categoryB & maskA) != 0
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serialization", derive(serde::Serialize, serde::Deserialize))]
pub struct Filter {
    /// Category bits (which categories this body belongs to)
    pub category_bits: u16,
    /// Mask bits (which categories this body can collide with)
    pub mask_bits: u16,
    /// Group index (optional group-based filtering)
    pub group_index: i16,
}

impl Filter {
    /// Create a new filter with the given category and mask bits
    #[inline]
    #[must_use]
    pub const fn new(category_bits: u16, mask_bits: u16) -> Self {
        Self {
            category_bits,
            mask_bits,
            group_index: 0,
        }
    }

    /// Create a filter that collides with everything
    #[inline]
    #[must_use]
    pub const fn all() -> Self {
        Self {
            category_bits: 0xFFFF,
            mask_bits: 0xFFFF,
            group_index: 0,
        }
    }

    /// Create a filter that collides with nothing
    #[inline]
    #[must_use]
    pub const fn none() -> Self {
        Self {
            category_bits: 0,
            mask_bits: 0,
            group_index: 0,
        }
    }

    /// Create a filter for a specific category
    #[inline]
    #[must_use]
    pub const fn category(category: u16) -> Self {
        Self {
            category_bits: category,
            mask_bits: category,
            group_index: 0,
        }
    }

    /// Check if this filter can collide with another filter
    ///
    /// Two colliders collide when each one's category is accepted by the
    /// other's mask. A non-zero group index overrides the category/mask test
    /// when both colliders share it: a positive group forces the pair to
    /// collide, a negative group forces it never to collide (the standard
    /// Box2D semantics).
    #[inline]
    #[must_use]
    pub fn can_collide_with(&self, other: &Self) -> bool {
        // Group filtering takes precedence over the category/mask test
        if self.group_index != 0 && self.group_index == other.group_index {
            return self.group_index > 0;
        }

        // Check category/mask collision
        if (self.category_bits & other.mask_bits) == 0 {
            return false;
        }
        if (other.category_bits & self.mask_bits) == 0 {
            return false;
        }

        true
    }

    /// Set the category bits
    #[inline]
    pub fn set_category_bits(&mut self, bits: u16) {
        self.category_bits = bits;
    }

    /// Set the mask bits
    #[inline]
    pub fn set_mask_bits(&mut self, bits: u16) {
        self.mask_bits = bits;
    }

    /// Set the group index
    #[inline]
    pub fn set_group_index(&mut self, index: i16) {
        self.group_index = index;
    }

    /// Add a category
    #[inline]
    pub fn add_category(&mut self, category: u16) {
        self.category_bits |= category;
    }

    /// Remove a category
    #[inline]
    pub fn remove_category(&mut self, category: u16) {
        self.category_bits &= !category;
    }

    /// Check if a category is set
    #[inline]
    #[must_use]
    pub fn has_category(self, category: u16) -> bool {
        (self.category_bits & category) != 0
    }

    /// Add a mask category
    #[inline]
    pub fn add_mask(&mut self, category: u16) {
        self.mask_bits |= category;
    }

    /// Remove a mask category
    #[inline]
    pub fn remove_mask(&mut self, category: u16) {
        self.mask_bits &= !category;
    }

    /// Check if a mask category is set
    #[inline]
    #[must_use]
    pub fn has_mask(self, category: u16) -> bool {
        (self.mask_bits & category) != 0
    }
}

impl Default for Filter {
    fn default() -> Self {
        Self::all()
    }
}

impl fmt::Display for Filter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Filter(categories: 0x{:04X}, mask: 0x{:04X}, group: {})",
            self.category_bits, self.mask_bits, self.group_index
        )
    }
}

/// Common category definitions
pub mod categories {
    /// Category 1
    pub const CATEGORY_1: u16 = 0x0001;
    /// Category 2
    pub const CATEGORY_2: u16 = 0x0002;
    /// Category 3
    pub const CATEGORY_3: u16 = 0x0004;
    /// Category 4
    pub const CATEGORY_4: u16 = 0x0008;
    /// Category 5
    pub const CATEGORY_5: u16 = 0x0010;
    /// Category 6
    pub const CATEGORY_6: u16 = 0x0020;
    /// Category 7
    pub const CATEGORY_7: u16 = 0x0040;
    /// Category 8
    pub const CATEGORY_8: u16 = 0x0080;
    /// Category 9
    pub const CATEGORY_9: u16 = 0x0100;
    /// Category 10
    pub const CATEGORY_10: u16 = 0x0200;
    /// Category 11
    pub const CATEGORY_11: u16 = 0x0400;
    /// Category 12
    pub const CATEGORY_12: u16 = 0x0800;
    /// Category 13
    pub const CATEGORY_13: u16 = 0x1000;
    /// Category 14
    pub const CATEGORY_14: u16 = 0x2000;
    /// Category 15
    pub const CATEGORY_15: u16 = 0x4000;
    /// Category 16
    pub const CATEGORY_16: u16 = 0x8000;
    /// All categories
    pub const ALL: u16 = 0xFFFF;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let filter1 = Filter::new(0x0001, 0x0001);
        assert_eq!(filter1.category_bits, 0x0001);
        assert_eq!(filter1.mask_bits, 0x0001);
        assert_eq!(filter1.group_index, 0);

        let filter2 = Filter::all();
        assert_eq!(filter2.category_bits, 0xFFFF);
        assert_eq!(filter2.mask_bits, 0xFFFF);

        let filter3 = Filter::none();
        assert_eq!(filter3.category_bits, 0);
        assert_eq!(filter3.mask_bits, 0);

        let filter4 = Filter::category(0x0002);
        assert_eq!(filter4.category_bits, 0x0002);
        assert_eq!(filter4.mask_bits, 0x0002);
    }

    #[test]
    fn test_collision() {
        let filter1 = Filter::category(0x0001);
        let filter2 = Filter::category(0x0001);
        let filter3 = Filter::category(0x0002);

        assert!(filter1.can_collide_with(&filter2));
        assert!(!filter1.can_collide_with(&filter3));

        // Test with mask
        let filter4 = Filter::new(0x0001, 0x0001 | 0x0002);
        let filter5 = Filter::category(0x0002);
        
        assert!(filter4.can_collide_with(&filter5));
        assert!(filter5.can_collide_with(&filter4));
    }

    #[test]
    fn test_group_filtering() {
        let mut filter1 = Filter::all();
        let mut filter2 = Filter::all();

        // Same positive group - should collide
        filter1.set_group_index(1);
        filter2.set_group_index(1);
        assert!(filter1.can_collide_with(&filter2));

        // Different groups - should collide
        filter1.set_group_index(1);
        filter2.set_group_index(2);
        assert!(filter1.can_collide_with(&filter2));

        // Negative group - should not collide with other negative groups
        filter1.set_group_index(-1);
        filter2.set_group_index(-1);
        assert!(!filter1.can_collide_with(&filter2));

        // Negative group with positive group - should collide
        filter1.set_group_index(-1);
        filter2.set_group_index(1);
        assert!(filter1.can_collide_with(&filter2));
    }

    #[test]
    fn test_modify() {
        let mut filter = Filter::none();

        filter.add_category(0x0001);
        assert!(filter.has_category(0x0001));
        assert!(!filter.has_category(0x0002));

        filter.remove_category(0x0001);
        assert!(!filter.has_category(0x0001));

        filter.add_mask(0x0002);
        assert!(filter.has_mask(0x0002));

        filter.remove_mask(0x0002);
        assert!(!filter.has_mask(0x0002));
    }

    #[test]
    fn test_default() {
        let filter = Filter::default();
        assert_eq!(filter.category_bits, 0xFFFF);
        assert_eq!(filter.mask_bits, 0xFFFF);
    }

    #[test]
    fn test_display() {
        let filter = Filter::new(0x0001, 0x0003);
        let filter_str = format!("{}", filter);
        assert!(filter_str.contains("Filter"));
        assert!(filter_str.contains("0x0001"));
        assert!(filter_str.contains("0x0003"));
    }

    #[test]
    fn test_categories() {
        assert_eq!(categories::CATEGORY_1, 0x0001);
        assert_eq!(categories::CATEGORY_2, 0x0002);
        assert_eq!(categories::ALL, 0xFFFF);
    }
}
