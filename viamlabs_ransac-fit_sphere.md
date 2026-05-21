# Model viamlabs:ransac-fit:sphere

RANSAC sphere-fitting for 3D point clouds. The model takes the name of a
configured 3D camera, calls `NextPointCloud()` on every request, and fits
the dominant sphere using a 4-point RANSAC. Each iteration solves a 3×3
linear system (via Cramer's rule) for the unique sphere through four
sampled points; the candidate with the most inliers wins.

The same model name is registered against two APIs:

- `rdk:service:generic` — a service that exposes the fit through
  `DoCommand` only.
- `rdk:component:generic` — a component that exposes the same `DoCommand`
  output **and** renders the fitted sphere through `Geometries()` as a
  `Sphere` primitive at the fitted center with the fitted radius.

## Configuration

```json
{
  "camera": "<3d-camera-name>",
  "iterations": 200,
  "tolerance": 10
}
```

### Attributes

| Name         | Type    | Inclusion | Default | Description |
|--------------|---------|-----------|---------|-------------|
| `camera`     | string  | Required  | —       | Name of a configured 3D camera component (one that implements `NextPointCloud`). |
| `iterations` | int     | Optional  | 200     | Number of RANSAC iterations. |
| `tolerance`  | float64 | Optional  | 10      | Maximum signed distance from the candidate sphere surface for a point to count as an inlier, in the point cloud's units. |

### Example Configuration

As a generic service (DoCommand-only):

```json
{
  "name": "ball-fitter-service",
  "namespace": "rdk",
  "type": "generic_service",
  "model": "viamlabs:ransac-fit:sphere",
  "attributes": {
    "camera": "depth_cam"
  }
}
```

As a generic component (DoCommand + Geometries):

```json
{
  "name": "ball-fitter",
  "namespace": "rdk",
  "type": "generic",
  "model": "viamlabs:ransac-fit:sphere",
  "depends_on": ["depth_cam"],
  "attributes": {
    "camera": "depth_cam",
    "iterations": 500,
    "tolerance": 10
  }
}
```

## DoCommand

Both variants ignore the command payload and run a fresh RANSAC fit on
each call.

| Field               | Type      | Description |
|---------------------|-----------|-------------|
| `center`            | `[x,y,z]` | Fitted sphere center. |
| `radius`            | float     | Fitted sphere radius. |
| `inlier_percentage` | float     | Percentage of input points (0–100) within `tolerance` of the fitted sphere surface. |

### Example DoCommand

Request:

```json
{}
```

Response:

```json
{
  "center": [12.4, -8.1, 305.7],
  "radius": 73.9,
  "inlier_percentage": 64.2
}
```

## Geometries (component only)

The component variant implements `resource.Shaped`. Each call to
`Geometries()` runs a fresh RANSAC fit and returns a single `Sphere`
geometry:

- Pose: `NewPoseFromPoint(center)` (orientation is identity; a sphere has
  no preferred orientation).
- Radius: the fitted sphere radius.

The sphere's label is set to the component's resource name.
