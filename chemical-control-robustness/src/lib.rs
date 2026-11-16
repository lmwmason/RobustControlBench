pub mod analysis;
pub mod controllers;
pub mod experiments;
pub mod model;
pub mod output;
pub mod simulation;

use anyhow::{Context, Result};
use serde::de::DeserializeOwned;
use std::{fs, path::Path};

pub fn read_toml<T: DeserializeOwned>(path: impl AsRef<Path>) -> Result<T> {
    let path = path.as_ref();
    toml::from_str(
        &fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?,
    )
    .with_context(|| format!("parsing {}", path.display()))
}
