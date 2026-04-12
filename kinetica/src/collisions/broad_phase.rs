#[derive(Clone, Debug)]
pub struct PotentialPair {
    pub a: usize,
    pub b: usize,
}

pub trait BroadPhase: Send + Sync {
    fn find_pairs(&self, n: usize) -> Vec<PotentialPair>;
}

pub struct NaiveBroadPhase;

impl BroadPhase for NaiveBroadPhase {
    fn find_pairs(&self, n: usize) -> Vec<PotentialPair> {
        let mut pairs = Vec::new();
        for i in 0..n {
            for j in (i + 1)..n {
                pairs.push(PotentialPair { a: i, b: j });
            }
        }
        pairs
    }
}