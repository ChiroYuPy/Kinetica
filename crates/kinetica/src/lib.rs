//! # Kinetica
//!
//! A 2D rigid-body physics engine for games.
//!
//! Everything lives in a [`World`]: create bodies with a [`BodyDesc`], give them shapes with a
//! [`ColliderDesc`], then call [`World::step`] at a fixed rate.
//!
//! Units are meters, kilograms and seconds; the Y axis points up. Pick sizes close to real
//! objects (a crate is about 1 m wide) and convert to pixels only when rendering.
//!
//! ```
//! use kinetica::prelude::*;
//!
//! let mut world = World::new();
//!
//! let ground = world.add_body(BodyDesc::fixed());
//! world.add_collider(ground, ColliderDesc::rectangle(20.0, 0.5));
//!
//! let crate_body = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 5.0)));
//! world.add_collider(crate_body, ColliderDesc::rectangle(0.5, 0.5).density(2.0));
//!
//! world.step(1.0 / 60.0);
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
mod query;
pub mod shape;
mod solver;
mod world;

pub use body::{Body, BodyDesc, BodyHandle, BodyType};
pub use collider::{ChainDesc, Collider, ColliderDesc, ColliderHandle, CollisionLayers};
pub use contact::{CollisionEvent, Contact, ContactPair, SensorEvent, SensorPair};
pub use debug_draw::{Color, DebugDraw, DebugDrawOptions};
pub use joint::{
    DistanceJointDesc, Joint, JointDesc, JointHandle, JointKind, MouseJointDesc, PrismaticJointDesc, RevoluteJointDesc,
    WeldJointDesc,
};
pub use math::{Vec2, vec2};
pub use query::{QueryFilter, RayHit, ShapeHit};
pub use world::{DEFAULT_GRAVITY, World};

/// The types needed by most programs, to import with `use kinetica::prelude::*;`.
pub mod prelude {
    pub use crate::math::{Rot, Transform, Vec2, vec2};
    pub use crate::{
        BodyDesc, BodyHandle, BodyType, ChainDesc, ColliderDesc, ColliderHandle, CollisionEvent, CollisionLayers,
        DistanceJointDesc, JointHandle, JointKind, MouseJointDesc, PrismaticJointDesc, QueryFilter, RevoluteJointDesc,
        SensorEvent, WeldJointDesc, World,
    };
}
