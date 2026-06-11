use crate::collisions::broad_phase::PotentialPair;
use crate::collisions::contact::test_contact;
use crate::constraints::ContactManifold;
use crate::RigidBody;

pub fn test_pairs(pairs: &[PotentialPair], bodies: &[RigidBody]) -> Vec<ContactManifold> {
    pairs
        .iter()
        .filter_map(|pair| test_pair(pair, bodies))
        .collect()
}

fn test_pair(pair: &PotentialPair, bodies: &[RigidBody]) -> Option<ContactManifold> {
    let body_a = &bodies[pair.a];
    let body_b = &bodies[pair.b];

    if body_a.mass.inverse_mass == 0.0 && body_b.mass.inverse_mass == 0.0 {
        return None;
    }

    test_contact(body_a, body_b).map(|mut m| {
        m.body_a = pair.a;
        m.body_b = pair.b;
        m
    })
}
