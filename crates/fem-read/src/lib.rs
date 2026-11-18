//! Readers of finite element meshes, simulation results and structural analysis models, into
//! the neutral model of `fem-model`:
//!
//! - [`vtk`]: VTK files, legacy and XML, with their pieces, blocks and time steps.
//! - [`inp`]: the FEM input deck (`.inp`).
//! - [`bdf`]: FEM bulk data (`.bdf`, `.nas`, `.dat`).
//! - [`msh`]: the finite element mesh (`.msh`, versions 2 and 4, ASCII and binary).
//! - [`s2k`]: the structural analysis text model (`.s2k`, `.$2k`).
//!
//! Every reader takes the file's name and bytes, and a [`Side`] that gives the files it refers
//! to (the pieces of a parallel VTK file), so it reads from memory as well as from disk.

mod build;
pub mod bdf;
pub mod inp;
pub mod msh;
pub mod s2k;
pub mod vtk;
mod vtk_raw;
mod xml;

pub use fem_model;
use fem_model::Document;

/// Gives the bytes of a file a file refers to, by its path relative to the folder of the file
/// that was opened (with `/` between folders).
pub type Side<'a> = &'a dyn Fn(&str) -> Result<Vec<u8>, String>;

/// The extensions each reader reads, lower case without the dot.
pub const VTK: &[&str] = &["vtk", "vtu", "vtp", "vts", "vtr", "vti", "pvtu", "pvtp", "pvts", "pvtr", "pvti", "vtm", "pvd"];
pub const INP: &[&str] = &["inp"];
pub const BDF: &[&str] = &["bdf", "nas", "dat", "blk", "bulk"];
pub const MSH: &[&str] = &["msh"];
pub const S2K: &[&str] = &["s2k", "$2k"];

/// The file name without its folder and extension.
pub(crate) fn stem(name: &str) -> String {
    let file = name.rsplit(['/', '\\']).next().unwrap_or(name);
    file.rfind('.').map_or(file, |k| &file[..k]).to_string()
}

/// The lower-case extension of a file name.
pub fn extension(name: &str) -> String {
    let file = name.rsplit(['/', '\\']).next().unwrap_or(name);
    file.rfind('.').map_or(String::new(), |k| file[k + 1..].to_ascii_lowercase())
}

/// Reads a file by its extension.
pub fn read(name: &str, data: &[u8], side: Side) -> Result<Document, String> {
    let ext = extension(name);
    match ext.as_str() {
        e if VTK.contains(&e) => vtk::read(name, data, side),
        "inp" => inp::read(name, data, side),
        "bdf" | "nas" | "dat" | "blk" | "bulk" => bdf::read(name, data, side),
        "msh" => msh::read(name, data),
        "s2k" | "$2k" => s2k::read(name, data),
        _ => Err(format!("not a format these readers know: .{ext}")),
    }
}
