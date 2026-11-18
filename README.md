# fem-dll

Rust readers of finite element meshes, simulation results and structural analysis models, for
Open BIM Suite and Open 3D Viewer Studio. They read from bytes in memory, so they work in the
browser (WebAssembly) as well as on the desktop.

| Crate | What it holds |
|---|---|
| `fem-model` | The neutral model: meshes of cells with their fields per time step and the outer faces to draw them by; structural analysis models of nodes, supports, members, plates, sections, materials, load cases, loads and combinations, in SI units. |
| `fem-read` | The readers, one module per format, into `fem-model`. |

## Formats

| Format | Extensions | What is read |
|---|---|---|
| VTK, legacy and XML | `.vtk`, `.vtu`, `.vtp`, `.vts`, `.vtr`, `.vti` | Points and cells (lines, faces, volumes; quadratic cells by their corners), the point and cell data arrays as fields. A grid of three dimensions keeps its outer faces only. Raw, base64, zlib and LZ4 data; not xz. |
| VTK parallel files | `.pvtu`, `.pvtp`, `.pvts`, `.pvtr`, `.pvti` | Their pieces, from the files beside them, each a group. |
| VTK multiblock | `.vtm` | Every data set it names, each a group named after its block. |
| VTK time series | `.pvd` | The mesh of the first time step and the fields of every step. |

The VTK files are parsed by the [`vtkio`](https://crates.io/crates/vtkio) crate (MIT OR
Apache-2.0); `fem-read` turns them into the neutral model. Its tests use sample files of that
crate, kept with its licence in `crates/fem-read/tests/samples/vtkio`.

## Use

```rust
let data = std::fs::read("result.vtu")?;
let side = |rel: &str| std::fs::read(rel).map_err(|e| e.to_string());
let doc = fem_read::read("result.vtu", &data, &side)?;
for surface in doc.mesh.as_ref().unwrap().surfaces() {
    // surface.positions, surface.normals, surface.triangles, surface.lines
}
```

## Licence

LGPL-3.0-or-later.
