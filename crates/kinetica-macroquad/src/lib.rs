//! # kinetica-macroquad
//!
//! Everything needed to put a [Kinetica](kinetica) world on screen with
//! [macroquad], so games only write their own logic:
//!
//! - [`WorldCamera`]: a camera in meters with the Y axis up, zoom, pan and target following;
//! - [`DebugRenderer`]: draws a world, or single colliders, with macroquad shapes;
//! - [`FixedTimestep`]: steps the world at a fixed rate whatever the frame rate;
//! - [`MouseGrab`]: drags dynamic bodies with the mouse.
//!
//! ```no_run
//! use kinetica::prelude::*;
//! use kinetica_macroquad::{DebugRenderer, FixedTimestep, WorldCamera};
//! use macroquad::prelude::{BLACK, clear_background, next_frame};
//!
//! #[macroquad::main("Falling ball")]
//! async fn main() {
//!     let mut world = World::new();
//!     let ground = world.add_body(BodyDesc::fixed());
//!     world.add_collider(ground, ColliderDesc::rectangle(10.0, 0.5));
//!     let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 5.0)));
//!     world.add_collider(ball, ColliderDesc::circle(0.5).restitution(0.6));
//!
//!     let camera = WorldCamera::new(vec2(0.0, 3.0), 10.0);
//!     let mut timestep = FixedTimestep::default();
//!     loop {
//!         for _ in 0..timestep.advance(macroquad::time::get_frame_time()) {
//!             world.step(timestep.dt());
//!         }
//!         clear_background(BLACK);
//!         camera.apply();
//!         DebugRenderer::new(&camera).draw_world(&world, &Default::default());
//!         next_frame().await;
//!     }
//! }
//! ```

mod camera;
mod grab;
mod render;
mod timestep;

pub use camera::WorldCamera;
pub use grab::MouseGrab;
pub use render::{DebugRenderer, to_macroquad_color};
pub use timestep::FixedTimestep;
