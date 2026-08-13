//! Physics scenes for the demo.

mod ragdoll;
mod stress;

pub use ragdoll::RagdollScene;
pub use stress::{create_stress_scene, get_stress_stats, StressStats};
