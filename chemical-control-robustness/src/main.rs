use anyhow::{Result, bail};
use chemical_control_robustness::experiments::runner::run_all;
use std::path::PathBuf;

fn main() -> Result<()> {
    let command = std::env::args().nth(1).unwrap_or_else(|| "help".to_owned());
    match command.as_str() {
        "run-all" => {
            let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            run_all(&root)?;
            println!("Experiments complete: {}", root.join("results").display());
        }
        "help" | "--help" | "-h" => println!("Usage: cargo run --release -- run-all"),
        _ => bail!("unknown command '{command}'; expected run-all"),
    }
    Ok(())
}
