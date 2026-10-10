//! Reproducible performance benchmarks for Kinetica.
//!
//! Each benchmark builds a world, runs it for a while to reach a typical state, then times a
//! fixed number of iterations. Run in release mode:
//!
//! ```text
//! cargo run --release -p kinetica-bench              # every benchmark
//! cargo run --release -p kinetica-bench -- pyramid   # benchmarks whose name contains "pyramid"
//! ```
//!
//! Each benchmark runs [`ROUNDS`] times from scratch and the fastest round is reported, which
//! filters out most of the noise from other programs.
//!
//! The checksum hashes the final state (or the query results) bit for bit: an optimization
//! that leaves it unchanged did not change what the engine computes.

mod scenarios;

use std::time::Instant;

use kinetica::World;

/// Fixed time step used by every benchmark.
const DT: f32 = 1.0 / 60.0;

/// Times each benchmark is run; the fastest round is kept.
const ROUNDS: usize = 3;

struct Benchmark {
    name: &'static str,
    description: &'static str,
    build: fn() -> World,
    /// Steps run before timing.
    warmup: usize,
    /// Timed iterations.
    iterations: usize,
    iteration: Iteration,
}

/// What one timed iteration does.
enum Iteration {
    /// One world step.
    Step,
    /// Something else, returning a value folded into the checksum.
    Custom(fn(&mut World) -> u64),
}

impl Benchmark {
    const fn steps(
        name: &'static str,
        description: &'static str,
        build: fn() -> World,
        warmup: usize,
        iterations: usize,
    ) -> Self {
        Self { name, description, build, warmup, iterations, iteration: Iteration::Step }
    }
}

/// Timings of one benchmark, in milliseconds per iteration.
struct Report {
    mean: f64,
    median: f64,
    worst: f64,
    checksum: u64,
}

fn measure(benchmark: &Benchmark) -> (World, Report) {
    let mut world = (benchmark.build)();
    for _ in 0..benchmark.warmup {
        world.step(DT);
    }
    let mut times = Vec::with_capacity(benchmark.iterations);
    let mut checksum = Checksum::default();
    for _ in 0..benchmark.iterations {
        let start = Instant::now();
        match benchmark.iteration {
            Iteration::Step => world.step(DT),
            Iteration::Custom(run) => checksum.add(run(&mut world)),
        }
        times.push(start.elapsed().as_secs_f64() * 1000.0);
    }
    for (_, body) in world.bodies() {
        checksum.add(u64::from(body.position().x.to_bits()));
        checksum.add(u64::from(body.position().y.to_bits()));
        checksum.add(u64::from(body.angle().to_bits()));
    }

    let mean = times.iter().sum::<f64>() / times.len() as f64;
    times.sort_by(f64::total_cmp);
    let report = Report { mean, median: times[times.len() / 2], worst: times[times.len() - 1], checksum: checksum.0 };
    (world, report)
}

/// FNV-1a over 64-bit words.
struct Checksum(u64);

impl Default for Checksum {
    fn default() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }
}

impl Checksum {
    fn add(&mut self, value: u64) {
        for byte in value.to_le_bytes() {
            self.0 = (self.0 ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3);
        }
    }
}

fn main() {
    let filters: Vec<String> = std::env::args().skip(1).collect();
    let benchmarks: Vec<Benchmark> = scenarios::all()
        .into_iter()
        .filter(|benchmark| filters.is_empty() || filters.iter().any(|filter| benchmark.name.contains(filter.as_str())))
        .collect();
    if benchmarks.is_empty() {
        let names: Vec<&str> = scenarios::all().iter().map(|benchmark| benchmark.name).collect();
        eprintln!("No benchmark matches. Available: {}", names.join(", "));
        std::process::exit(1);
    }
    if cfg!(debug_assertions) {
        eprintln!("Warning: debug build, timings are meaningless. Use --release.");
    }

    println!(
        "{:<10} {:>7} {:>7} {:>7} {:>9}  {:>8} {:>8} {:>8}  {:<16}",
        "benchmark", "bodies", "awake", "joints", "contacts", "mean ms", "median", "worst", "checksum"
    );
    for benchmark in &benchmarks {
        let (world, report) = (0..ROUNDS)
            .map(|_| measure(benchmark))
            .min_by(|(_, a), (_, b)| a.mean.total_cmp(&b.mean))
            .expect("at least one round");
        println!(
            "{:<10} {:>7} {:>7} {:>7} {:>9}  {:>8.3} {:>8.3} {:>8.3}  {:016x}",
            benchmark.name,
            world.body_count(),
            world.bodies().filter(|(_, body)| !body.is_sleeping()).count(),
            world.joint_count(),
            world.contacts().count(),
            report.mean,
            report.median,
            report.worst,
            report.checksum,
        );
    }
    println!();
    for benchmark in &benchmarks {
        println!("{:<10} {}", benchmark.name, benchmark.description);
    }
}
