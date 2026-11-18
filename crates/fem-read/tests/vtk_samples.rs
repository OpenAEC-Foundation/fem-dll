//! The VTK sample files of the `vtkio` crate (MIT OR Apache-2.0, see `samples/vtkio/LICENSE-MIT`),
//! read as an app reads them: from disk, with the files beside them.

use fem_read::fem_model::{CellKind, FieldAt};
use std::path::{Path, PathBuf};

fn samples() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/samples/vtkio")
}

fn open(name: &str) -> Result<fem_read::fem_model::Document, String> {
    let dir = samples();
    let data = std::fs::read(dir.join(name)).unwrap();
    let side = |rel: &str| std::fs::read(dir.join(rel)).map_err(|e| format!("{rel}: {e}"));
    fem_read::read(name, &data, &side)
}

#[test]
fn compressed_and_binary_unstructured_grids_read_alike() {
    let plain = open("hexahedron_binary.vtu").expect("binary").mesh.unwrap();
    for name in ["hexahedron_zlib.vtu", "hexahedron_lz4.vtu"] {
        let mesh = open(name).expect(name).mesh.unwrap();
        assert_eq!(mesh.points, plain.points, "{name}");
        assert_eq!(mesh.cells.kind, [CellKind::Hexa], "{name}");
        assert_eq!(mesh.surfaces()[0].triangles.len() / 3, 12, "{name}");
    }
}

#[test]
fn xz_compression_is_refused_with_a_message() {
    // the xz decoder needs a C library, which the browser build cannot have
    assert!(open("hexahedron_lzma_inline_binary.vtu").is_err());
}

#[test]
fn a_parallel_file_reads_its_pieces_beside_it() {
    let mesh = open("hexahedron_parallel.pvtu").expect("pvtu").mesh.unwrap();
    assert_eq!(mesh.cells.kind, [CellKind::Hexa]);
    assert_eq!(mesh.points.len(), 8);
}

#[test]
fn every_kind_of_data_set_gives_faces_to_draw() {
    for name in ["box.vtu", "cube.vtk", "structured_grid.vtk", "RectilinearGridCompressed.vtr", "RectilinearGridAppendedBase64.vtr", "polyEx0.vtp", "tet.vtu", "rectilinear_grid_binary.vtk", "point_cloud.vtp"] {
        let doc = open(name).unwrap_or_else(|e| panic!("{name}: {e}"));
        let mesh = doc.mesh.unwrap();
        let surfaces = mesh.surfaces();
        assert!(!surfaces.is_empty(), "{name}");
        let (triangles, dots): (usize, usize) = surfaces.iter().fold((0, 0), |(t, d), s| (t + s.triangles.len(), d + s.dots.len()));
        assert!(triangles > 0 || dots > 0, "{name}");
        for step in &mesh.steps {
            for f in &step.fields {
                let count = if f.at == FieldAt::Point { mesh.points.len() } else { mesh.cells.len() };
                assert_eq!(f.count(), count, "{name}: {}", f.name);
            }
        }
    }
}
