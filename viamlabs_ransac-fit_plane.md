# Model viamlabs:ransac-fit:plane

RANSAC plane-fitting for 3D point clouds. The model takes the name of a
configured 3D camera, calls `NextPointCloud()` on every request, and fits
the dominant plane via a 3-point RANSAC.

The same model name is registered against two APIs:

- `rdk:service:generic` — a service that exposes the fit through
  `DoCommand` only.
- `rdk:component:generic` — a component that exposes the same `DoCommand`
  output **and** renders the fitted plane through `Geometries()` as a thin
  `Box` aligned with the plane normal. Sized from the inlier bounding
  rectangle in the plane, with thickness `2 × tolerance` so the box
  visualises the tolerance band.

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
| `camera`     | string  | Required  | —       | Name of a configured 3D camera component (one that implements `NextPointCloud`). The camera is wired in as a required dependency. |
| `iterations` | int     | Optional  | 200     | Number of RANSAC iterations. Higher values are more reliable but slower. |
| `tolerance`  | float64 | Optional  | 10      | Maximum perpendicular distance from the candidate plane for a point to count as an inlier, in the point cloud's units (typically millimeters). |

### Example Configuration

As a generic service (DoCommand-only):

```json
{
  "name": "table-plane-service",
  "namespace": "rdk",
  "type": "generic_service",
  "model": "viamlabs:ransac-fit:plane",
  "attributes": {
    "camera": "depth_cam"
  }
}
```

As a generic component (DoCommand + Geometries):

```json
{
  "name": "table-plane",
  "namespace": "rdk",
  "type": "generic",
  "model": "viamlabs:ransac-fit:plane",
  "depends_on": ["depth_cam"],
  "attributes": {
    "camera": "depth_cam",
    "iterations": 500,
    "tolerance": 10
  }
}
```

## DoCommand

Both variants ignore the command payload and run a fresh RANSAC fit on each
call. The returned map has the same shape in both:

| Field               | Type      | Description |
|---------------------|-----------|-------------|
| `normal`            | `[x,y,z]` | Unit normal to the fitted plane. |
| `inlier_percentage` | float     | Fraction of input points (0–1) that lie within `tolerance` of the plane. |

### Example DoCommand

Request:

```json
{}
```

Response:

```json
{
  "normal": [0.0102, -0.9901, 0.1402],
  "inlier_percentage": 0.7245
}
```

## Geometries (component only)

The component variant implements `resource.Shaped`. Each call to
`Geometries()` runs a fresh RANSAC fit and returns a single thin `Box`
geometry built like this:

- Pose orientation: local `Z` axis aligned with the plane normal
  (`OrientationVector` with `OX/OY/OZ` set to the unit normal).
- Pose position: midpoint of the inlier bounding rectangle in the plane.
- `X`/`Y` dimensions: width and height of the inlier bounding rectangle.
- `Z` dimension (thickness): `2 × tolerance`.

The box's label is set to the component's resource name, so it appears in
the Viam world view by name.
