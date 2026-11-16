use anyhow::Result;
use std::{fs, path::Path};

pub fn write_generated_summary(path: &Path, text: &str) -> Result<()> {
    fs::write(path, text)?;
    Ok(())
}
