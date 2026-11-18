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
| FEM input deck | `.inp` | Nodes, elements, node and element sets, parts and instances, included files; beams and trusses as members, shells and membranes as plates with their thickness, solids as the mesh; beam sections (rectangle, circle, pipe, box, I), materials, boundaries, concentrated loads, pressures on shells and gravity, each step a load case. |
| FEM bulk data | `.bdf`, `.nas`, `.dat` | Free, small and large field cards with continuations, included files: `GRID`; `CBAR`, `CBEAM`, `CROD`, `CONROD` as members; `CQUAD4`, `CTRIA3` and their kin as plates with the thickness of their `PSHELL`; `CHEXA`, `CTETRA`, `CPENTA`, `CPYRAM` as the mesh; `PBAR`, `PBEAM`, `PROD`, `PBARL`, `PBEAML`, `MAT1`; `SPC`, `SPC1`, `FORCE`, `MOMENT`, `PLOAD4`, `GRAV`, `LOAD`. |
| Finite element mesh | `.msh` | Versions 2 and 4, ASCII and binary: nodes, elements of all orders (by their corners), physical groups and entities as groups, `$NodeData` and `$ElementData` as fields per time step. |
| Structural analysis text model | `.s2k`, `.$2k` | Its tables of joints, frames, areas and solids; frame and area sections, materials, restraints and springs, local axes and releases; load patterns, joint, frame and area loads, combinations; in SI units. |

The element files have no units of their own (`.inp`, `.bdf`): they are read as SI.

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
