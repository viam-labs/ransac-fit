# Model viamlabs:ransac-fit:cylinder

RANSAC cylinder-fitting for 3D point clouds. The model takes the name of a
configured 3D camera, calls `NextPointCloud()` on every request, and fits
the dominant cylinder using a 5-point RANSAC:

1. Five distinct random sample points are drawn from the cloud.
2. The candidate axis direction is taken as the dominant eigenvector
   (largest eigenvalue) of the sample's 3×3 covariance matrix, found by
   power iteration. For a long, well-sampled cylinder the largest spread
   direction is the axis.
3. The five sample points are projected into the plane perpendicular to
   that axis, where an algebraic least-squares circle fit recovers a 2D
   center and radius.
4. The 2D center is lifted back into 3D as a point on the candidate axis.
5. Inliers are counted as points whose perpendicular distance from the
   axis differs from the candidate radius by at most `tolerance`.

This approach is approximate — see the trade-offs note at the bottom of
this file — but works well when the cylinder is well-sampled along its
length and dominates the cloud.

The same model name is registered against two APIs:

- `rdk:service:generic` — a service that exposes the fit through
  `DoCommand` only.
- `rdk:component:generic` — a component that exposes the same `DoCommand`
  output **and** renders the fitted cylinder through `Geometries()` as a
  `Capsule` primitive (cylinder with hemispherical end caps) aligned with
  the fitted axis. See the *Why a Capsule and not a Cylinder?* note below.

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
| `iterations` | int     | Optional  | 200     | Number of RANSAC iterations. Cylinder fitting benefits more from extra iterations than plane/sphere; 500–1000 is reasonable for long, thin cylinders. |
| `tolerance`  | float64 | Optional  | 10      | Maximum |distance‑from‑axis − radius| for a point to count as an inlier, in the point cloud's units. |

### Example Configuration

As a generic service (DoCommand-only):

```json
{
  "name": "pipe-fitter-service",
  "namespace": "rdk",
  "type": "generic_service",
  "model": "viamlabs:ransac-fit:cylinder",
  "attributes": {
    "camera": "depth_cam"
  }
}
```

As a generic component (DoCommand + Geometries):

```json
{
  "name": "pipe-fitter",
  "namespace": "rdk",
  "type": "generic",
  "model": "viamlabs:ransac-fit:cylinder",
  "depends_on": ["depth_cam"],
  "attributes": {
    "camera": "depth_cam",
    "iterations": 1000,
    "tolerance": 10
  }
}
```

## DoCommand

Both variants ignore the command payload and run a fresh RANSAC fit on
each call.

| Field               | Type      | Description |
|---------------------|-----------|-------------|
| `center`            | `[x,y,z]` | A point on the fitted cylinder's axis (the lifted 2D circle center in the axis-perpendicular plane through the sample centroid). |
| `axis`              | `[x,y,z]` | Unit direction vector along the cylinder's axis. |
| `radius`            | float     | Fitted cylinder radius. |
| `inlier_percentage` | float     | Percentage of input points (0–100) within `tolerance` of the cylinder surface. |

### Example DoCommand

Request:

```json
{}
```

Response:

```json
{
  "center": [12.4, -8.1, 305.7],
  "axis": [0.02, 0.99, 0.10],
  "radius": 25.3,
  "inlier_percentage": 58.7
}
```

## Geometries (component only)

The component variant implements `resource.Shaped`. Each call to
`Geometries()` runs a fresh RANSAC fit and returns a single `Capsule`
primitive built like this:

- Pose orientation: local `Z` axis aligned with the fitted axis direction
  (`OrientationVector` with `OX/OY/OZ` set to the unit axis).
- Length: the axial extent of the inliers, computed as
  `max((p − center)·axis) − min((p − center)·axis)`. Clamped to a
  minimum of `2 × radius + ε` because `spatialmath.NewCapsule` requires
  `length ≥ 2 × radius`.
- Pose position: the midpoint of that axial extent on the axis line,
  i.e. `center + axis · (minT + maxT) / 2`. This keeps the rendered
  capsule centered on the observed inliers rather than the (often
  arbitrary) raw `center` point.
- Radius: the fitted cylinder radius.

The capsule's label is set to the component's resource name.

### Why a Capsule and not a Cylinder?

The Viam Go SDK ships a `spatialmath.Cylinder` primitive, and an earlier
version of this component used it. However, `Cylinder.ToProtobuf()`
currently panics — the SDK does not yet have a `Cylinder` message in
`commonpb`. Returning a `spatialmath.Cylinder` from `Geometries()`
therefore kills the gRPC response mid-encoding and the client sees an
"error reading from server: EOF".

`Capsule` is the closest primitive that has a valid protobuf
representation. The radius and axis-alignment match the fit exactly; the
only visible difference is that the rendered geometry has rounded end
caps instead of flat ones. When/if the SDK adds a `Cylinder` proto
message, this component can be switched back trivially.

## Algorithmic notes and trade-offs

This implementation uses a **5-point sample + dominant-eigenvector axis +
2D circle fit** strategy rather than the textbook-best approach of
**2-point + estimated surface normals** (PCL style, which requires
precomputing per-point surface normals via a KD-tree). The current
approach has these characteristics:

- Works best when the cylinder is well-sampled along its length and is
  the dominant geometry in the cloud.
- Fails gracefully when 5-point samples cluster in a small patch — those
  iterations produce bad axis estimates with low inlier counts and are
  discarded by RANSAC's max-inlier selection.
- Bumping `iterations` is the easiest way to improve reliability.

If you need a more rigorous fit (e.g. for short, fat cylinders or
multi-object clouds), the recommended upgrade path is to precompute
per-point normals using `pointcloud.KDTree` and switch to the 2-point
PCL-style algorithm.
