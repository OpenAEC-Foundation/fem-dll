//! The neutral model the readers of `fem-read` return. It knows nothing of the program that
//! shows it:
//!
//! - [`Mesh`]: points and cells (lines, faces and volumes) with the values the file gives at the
//!   points or cells ([`Field`]), per time step; [`Mesh::surfaces`] gives the outer faces to draw.
//! - [`Analysis`]: a structural analysis model of nodes with their supports, members, plates,
//!   sections, materials, load cases, loads and combinations, in SI units.

pub mod analysis;
pub mod mesh;

pub use analysis::*;
pub use mesh::*;

/// What a file holds: a mesh to draw, an analysis model, or both (a file whose beams and shells
/// are an analysis model and whose solids are a mesh).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Document {
    pub mesh: Option<Mesh>,
    pub analysis: Option<Analysis>,
    /// The format as a reader names it, e.g. "FEM input deck (.inp)".
    pub format: String,
    /// What the reader left out or assumed.
    pub notes: Vec<String>,
}
