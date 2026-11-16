# Reproducibility Record

The audited artifacts use Rust 1.100.0 on x86_64 Linux. `rust-toolchain.toml` pins the toolchain and `Cargo.lock` pins dependencies. `run-all` reads all four files under `configs/`, runs the deterministic and 100 paired-noise experiments, and rewrites CSV/SVG artifacts. Base seed is `20261116`; replicate `i` uses `20261116+i`.

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
cargo run --release -- run-all
sha256sum results/data/metrics.csv results/summaries/linear-stability-grid.csv results/summaries/noise-paired-comparisons.csv
```

Expected hashes:

```text
f023a449cb9f5a55ef7f1a6fe0c9ea657cf45e0bdc5b779fcf51ac1dabe712a6  results/data/metrics.csv
75843955e1287a85da3dac87c3f52759a073665647f94d47764441a98b3c3ec7  results/summaries/linear-stability-grid.csv
caf1a4b65edec385c7c19649f838861838b5ce36da7fdb57a01a9a6d2a23971d  results/summaries/noise-paired-comparisons.csv
```

Numerical conventions: exact scalar ZOH; DARE update tolerance `1e-13` and residual tolerance `1e-8`; `N` left-held intervals for `N+1` endpoints; 95% paired intervals use `t(99)=1.984`; and stability uses the discrete unit-circle criterion on every integer capacity/heat-transfer deviation from -30% through +30%. Figures are produced directly by Rust/Plotters.

