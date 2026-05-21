# Module ransac-fit

A Viam module that performs RANSAC-based shape fitting on point clouds
streamed from any configured 3D camera (anything that implements
`NextPointCloud`).

Three shapes are supported — plane, sphere, and cylinder — each of which
is registered twice:

- As a **generic service** (`type: generic_service`) that exposes the fit
  result through `DoCommand` only.
- As a **generic component** (`type: generic`) that exposes the same
  `DoCommand` output **and** renders the fitted shape through
  `Geometries()` as a [`spatialmath`](https://pkg.go.dev/go.viam.com/rdk/spatialmath)
  primitive (`Box` for plane, `Sphere`, `Cylinder`). The component variant
  shows up in the Viam world tree and the remote-control 3D view.

The service and component variants of each shape share the same model
name; in the machine config they are disambiguated by `type`.

## Models

This module provides the following models:

| Model | APIs | Geometry | Docs |
|-------|------|----------|------|
| `viamlabs:ransac-fit:plane`    | `rdk:service:generic`, `rdk:component:generic` | Thin `Box` aligned with the plane normal | [docs](viamlabs_ransac-fit_plane.md) |
| `viamlabs:ransac-fit:sphere`   | `rdk:service:generic`, `rdk:component:generic` | `Sphere` at fitted center | [docs](viamlabs_ransac-fit_sphere.md) |
| `viamlabs:ransac-fit:cylinder` | `rdk:service:generic`, `rdk:component:generic` | `Capsule` aligned with fitted axis (used in place of `Cylinder` because the SDK's `Cylinder.ToProtobuf` is unimplemented) | [docs](viamlabs_ransac-fit_cylinder.md) |

## Common configuration

Every model accepts the same three attributes:

| Name         | Type    | Inclusion | Default | Description |
|--------------|---------|-----------|---------|-------------|
| `camera`     | string  | Required  | —       | Name of a configured 3D camera that implements `NextPointCloud`. The camera is registered as a required dependency. |
| `iterations` | int     | Optional  | 200     | Number of RANSAC iterations. |
| `tolerance`  | float64 | Optional  | 10      | Inlier distance threshold, in the point cloud's units (typically millimetres). |

See the per-model docs for the precise interpretation of `tolerance` and
the exact `DoCommand` / `Geometries()` outputs.

## Quick start

Add a 3D camera and one or more ransac-fit components to your machine
config:

```json
{
  "components": [
    {
      "name": "depth_cam",
      "type": "camera",
      "namespace": "rdk",
      "model": "<your-depth-cam-model>",
      "attributes": { /* ... */ }
    },
    {
      "name": "table-plane",
      "type": "generic",
      "namespace": "rdk",
      "model": "viamlabs:ransac-fit:plane",
      "depends_on": ["depth_cam"],
      "attributes": {
        "camera": "depth_cam",
        "iterations": 500,
        "tolerance": 10
      }
    }
  ]
}
```

Then from a client SDK:

```python
plane = await GenericComponent.from_robot(machine, "table-plane")
result = await plane.do_command({})
print(result)
# {'normal': [0.01, -0.99, 0.13], 'inlier_percentage': 0.72}

geometries = await plane.get_geometries()
# Box geometry oriented along the plane normal, visible in the
# Viam remote-control 3D viewer.
```

## Behavioural notes

- Every `DoCommand` and `Geometries()` call pulls a fresh point cloud and
  runs RANSAC from scratch. There is no caching. For a 100k-point cloud
  with `iterations=500` this is on the order of hundreds of milliseconds,
  so polling at high frequency is not recommended.
- The RANSAC sampler uses a fixed RNG seed (`1`), so a given point cloud
  + iterations + tolerance always produces the same fit. This makes
  results reproducible but does mean two consecutive calls on the same
  scene can fail to "average out" noise — bump `iterations` rather than
  expecting variance between calls.
- `inlier_percentage` is currently returned as a 0–1 fraction by the
  plane variant and as a 0–100 percentage by the sphere and cylinder
  variants. See each per-model doc for the exact units.

## Source files

- `plane-service.go`, `sphere-service.go`, `cylinder-service.go` — the
  service-API registrations and the shared RANSAC math (`fit*RANSAC`,
  inlier collection, helper functions).
- `plane-component.go`, `sphere-component.go`, `cylinder-component.go` —
  the component-API registrations and the `Geometries()` implementations
  that convert fit results into `spatialmath` primitives.
- `cmd/module/main.go` — registers all six `(API, Model)` pairs.
- `cmd/cli/main.go` — minimal example of constructing one of the
  resources directly without a running module server.
