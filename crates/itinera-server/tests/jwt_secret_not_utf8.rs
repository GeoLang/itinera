#![cfg(unix)]

use itinera_graph::{Graph, SpeedProfile};
use itinera_server::{AppState, router};
use std::ffi::OsStr;
use std::os::unix::ffi::OsStrExt;

#[test]
#[should_panic(expected = "ITINERA_JWT_SECRET is set but not UTF-8")]
fn a_non_utf8_secret_refuses_to_build_the_router() {
    // ITINERA_JWT_SECRET is process-wide and this binary has one test
    unsafe { std::env::set_var("ITINERA_JWT_SECRET", OsStr::from_bytes(&[0xff])) };
    let _ = router(AppState::new(
        Graph::build(Vec::new(), Vec::new()),
        SpeedProfile::car(),
    ));
}
