//! Fuzz the (de)serializers: arbitrary bytes must produce a `Result`, never a panic.

#![no_main]

use libfuzzer_sys::fuzz_target;

use ro_physics::math::{Rot2, Transform, Vec2};
use ro_physics::serialization::{from_binary, from_json, from_ron};

fuzz_target!(|data: &[u8]| {
    // Every one of these is expected to fail on arbitrary bytes; what matters
    // is that it fails with an `Err`.
    let _ = from_binary::<Vec2>(data);
    let _ = from_binary::<Rot2>(data);
    let _ = from_binary::<Transform>(data);

    if let Ok(text) = std::str::from_utf8(data) {
        let _ = from_json::<Vec2>(text);
        let _ = from_ron::<Vec2>(text);
    }
});
