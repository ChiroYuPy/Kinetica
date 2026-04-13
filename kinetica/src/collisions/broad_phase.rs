use crate::math::{intersects, AABB};

#[derive(Clone, Debug)]
pub struct PotentialPair {
    pub a: usize,
    pub b: usize,
}

#[derive(Clone, Copy, Debug)]
struct Endpoint {
    value: f32,
    is_min: bool,
    body_id: usize,
}

pub struct SweepAndPrune {
    endpoints_x: Vec<Endpoint>,
}

impl SweepAndPrune {
    pub fn new() -> Self {
        Self {
            endpoints_x: Vec::new(),
        }
    }

    fn update_endpoints(&mut self, aabbs: &[AABB]) {
        self.endpoints_x.clear();
        for (id, aabb) in aabbs.iter().enumerate() {
            self.endpoints_x.push(Endpoint {
                value: aabb.min.x,
                is_min: true,
                body_id: id,
            });
            self.endpoints_x.push(Endpoint {
                value: aabb.max.x,
                is_min: false,
                body_id: id,
            });
        }
    }

    pub fn find_pairs(&mut self, aabbs: &[AABB]) -> Vec<PotentialPair> {
        if aabbs.is_empty() {
            return Vec::new();
        }

        self.update_endpoints(aabbs);

        // Tri : min avant max quand les valeurs sont égales
        self.endpoints_x.sort_by(|a, b| {
            match a.value.partial_cmp(&b.value).unwrap() {
                std::cmp::Ordering::Equal => {
                    // Même valeur : les min doivent venir avant les max
                    if a.is_min == b.is_min {
                        std::cmp::Ordering::Equal
                    } else if a.is_min {
                        std::cmp::Ordering::Less
                    } else {
                        std::cmp::Ordering::Greater
                    }
                }
                ord => ord
            }
        });

        let mut pairs = Vec::new();
        let mut active = Vec::new();

        for endpoint in &self.endpoints_x {
            if endpoint.is_min {
                for &active_id in &active {
                    if active_id != endpoint.body_id {
                        let a = &aabbs[active_id];
                        let b = &aabbs[endpoint.body_id];
                        if intersects(a, b) {
                            pairs.push(PotentialPair {
                                a: active_id,
                                b: endpoint.body_id,
                            });
                        }
                    }
                }
                active.push(endpoint.body_id);
            } else {
                if let Some(pos) = active.iter().position(|&id| id == endpoint.body_id) {
                    active.remove(pos);
                }
            }
        }

        pairs
    }
}

impl Default for SweepAndPrune {
    fn default() -> Self {
        Self::new()
    }
}
