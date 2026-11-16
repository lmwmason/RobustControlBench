# Reproducibility Record

The audited artifacts use Rust 1.100.0 on x86_64 Linux. `rust-toolchain.toml` pins the toolchain and `Cargo.lock` pins dependencies. `run-all` reads all four configuration files, performs the deterministic 64-candidate-per-controller tuning study, runs held-out deterministic and 100 paired-noise experiments, and rewrites CSV, SVG, and LaTeX-table artifacts. Base seed is `20261116`; replicate `i` uses `20261116+i`.

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
cargo run --release -- run-all
make -C paper
sha256sum results/data/metrics.csv results/data/tuning-study.csv \
  results/summaries/formal-stability-certificate.txt \
  paper/chemical-control-robustness.pdf
```

Expected hashes:

```text
b73c851485ae6d983c2549366d746620f72b7efb24b5fdb7c62e55a9b5265fae  results/data/metrics.csv
4b2be5f6cfb885e103c7c767db01bb1bae4b8388d5365fe289a59f3ddbb4e081  results/data/tuning-study.csv
ef97dfd15c7b2273565e5cc08630c5fdc601a24f10ab8d3bc7f50d76176a5040  results/summaries/formal-stability-certificate.txt
b54b5f07d9725a989d0ed69c86597684797802c4b7679fc35b887bc60f7244ea  paper/chemical-control-robustness.pdf
```

Numerical conventions are exact scalar ZOH; DARE update tolerance `1e-13` and residual tolerance `1e-8`; `N` left-held intervals for `N+1` endpoints; and paired 95% intervals using `t(99)=1.984`. LQR is certified with conservative scalar interval bounds. LQI uses second-order Jury inequalities with outward-conservative bounds over 128 by 128 covering subboxes. PID retains a 3721-plant grid and is not formally certified. Rust/Plotters produces SVG figures; the template build converts them to vector PDF with `rsvg-convert` and compiles with `latexmk`, pdfLaTeX, and Biber.
