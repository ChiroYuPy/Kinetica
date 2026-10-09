//! # Kinetica
//!
//! A 2D rigid-body physics engine for games.
//!
//! Everything lives in a [`World`]: describe bodies with a [`BodyDesc`] and their shapes with a
//! [`ColliderDesc`], add them, then call [`World::step`] at a fixed rate. Adding a body returns
//! a handle; index the world with it to read or change the body.
//!
//! Sizes are full sizes (a `rectangle(2.0, 1.0)` is 2 m wide), and joint anchors are given in
//! world space.
//!
//! Units are meters, kilograms and seconds; the Y axis points up. Pick sizes close to real
//! objects (a crate is about 1 m wide) and convert to pixels only when rendering.
//!
//! ```
//! use kinetica::prelude::*;
//!
//! let mut world = World::new();
//!
//! world.add_body(BodyDesc::fixed().collider(ColliderDesc::rectangle(40.0, 1.0)));
//! let crate_box = world.add_body(
//!     BodyDesc::dynamic().position(vec2(0.0, 5.0)).collider(ColliderDesc::square(1.0).density(2.0)),
//! );
//!
//! for _ in 0..60 {
//!     world.step(1.0 / 60.0);
//! }
//! println!("The crate fell to {}", world[crate_box].position());
//! ```

#![warn(missing_docs)]

mod arena;
mod body;
mod collider;
pub mod collision;
mod constants;
mod contact;
mod continuous;
mod debug_draw;
mod island;
pub mod joint;
pub mod math;
mod profile;
mod query;
pub mod shape;
mod solver;
mod world;

pub use body::{Body, BodyDesc, BodyHandle, BodyType};
pub use collider::{ChainDesc, Collider, ColliderDesc, ColliderHandle, CollisionLayers};
pub use contact::{CollisionEvent, Contact, ContactPair, SensorEvent, SensorPair};
pub use debug_draw::{Color, DebugDraw, DebugDrawOptions};
pub use joint::{
    DistanceJoint, DistanceJointDesc, Joint, JointDesc, JointHandle, JointType, Motor, MouseJoint, MouseJointDesc,
    PrismaticJoint, PrismaticJointDesc, RevoluteJoint, RevoluteJointDesc, Spring, WeldJoint, WeldJointDesc,
};
pub use math::{Vec2, vec2};
pub use profile::StepProfile;
pub use query::{QueryFilter, RayHit, ShapeHit};
pub use world::{DEFAULT_GRAVITY, World};

/// The types needed by most programs, to import with `use kinetica::prelude::*;`.
pub mod prelude {
    pub use crate::math::{Rot, Transform, Vec2, vec2};
    pub use crate::{
        BodyDesc, BodyHandle, BodyType, ChainDesc, ColliderDesc, ColliderHandle, CollisionEvent, CollisionLayers,
        ContactPair, DistanceJoint, DistanceJointDesc, Joint, JointHandle, Motor, MouseJoint, MouseJointDesc,
        PrismaticJoint, PrismaticJointDesc, QueryFilter, RevoluteJoint, RevoluteJointDesc, SensorEvent, SensorPair,
        WeldJoint, WeldJointDesc, World,
    };
}
