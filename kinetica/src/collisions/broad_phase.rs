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
    is_sorted: bool,
}

impl SweepAndPrune {
    pub fn new() -> Self {
        Self {
            endpoints_x: Vec::new(),
            is_sorted: false,
        }
    }

    fn update_endpoints(&mut self, aabbs: &[AABB]) {
        if self.endpoints_x.len() != aabbs.len() * 2 {
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
            self.is_sorted = false;
            return;
        }

        let mut i = 0;
        for (id, aabb) in aabbs.iter().enumerate() {
            self.endpoints_x[i].value = aabb.min.x;
            self.endpoints_x[i].body_id = id;
            self.endpoints_x[i + 1].value = aabb.max.x;
            self.endpoints_x[i + 1].body_id = id;
            i += 2;
        }
    }

    fn insertion_sort(&mut self) {
        for i in 1..self.endpoints_x.len() {
            let mut j = i;
            while j > 0 && self.endpoints_x[j - 1].value > self.endpoints_x[j].value {
                self.endpoints_x.swap(j - 1, j);
                j -= 1;
            }
        }
    }

    pub fn find_pairs(&mut self, aabbs: &[AABB]) -> Vec<PotentialPair> {
        if aabbs.is_empty() {
            return Vec::new();
        }

        self.update_endpoints(aabbs);

        if self.is_sorted {
            self.insertion_sort();
        } else {
            self.endpoints_x.sort_by(|a, b| a.value.partial_cmp(&b.value).unwrap());
            self.is_sorted = true;
        }

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
