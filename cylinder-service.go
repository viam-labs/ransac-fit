package ransacfit

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
)

// Cylinder is the viam resource model for the RANSAC cylinder fitting service.
var Cylinder = resource.NewModel("viam-labs", "ransac-fit", "cylinder")

const (
	cylinderSampleSize      = 5
	powerIterationSteps     = 30
	maxDistinctDrawAttempts = 50
)

func init() {
	resource.RegisterService(generic.API, Cylinder,
		resource.Registration[resource.Resource, *CylinderConfig]{
			Constructor: newRansacFitCylinder,
		},
	)
}

// CylinderConfig is the user-provided configuration for the ransac-fit cylinder service.
//
// Camera is the name of a 3D camera (one that supports NextPointCloud) that
// this service will pull point clouds from.
//
// Iterations is the number of RANSAC iterations to run. Optional, defaults to 200.
//
// Tolerance is the maximum distance (in the point cloud's units, typically mm)
// from the candidate cylinder surface for a point to be counted as an inlier.
// Optional, defaults to 10.
type CylinderConfig struct {
	Camera     string   `json:"camera"`
	Iterations *int     `json:"iterations,omitempty"`
	Tolerance  *float64 `json:"tolerance,omitempty"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// It also returns the camera as a required dependency.
func (cfg *CylinderConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Camera == "" {
		return nil, nil, fmt.Errorf(`%s: "camera" is required`, path)
	}
	if cfg.Iterations != nil && *cfg.Iterations <= 0 {
		return nil, nil, fmt.Errorf(`%s: "iterations" must be > 0`, path)
	}
	if cfg.Tolerance != nil && *cfg.Tolerance <= 0 {
		return nil, nil, fmt.Errorf(`%s: "tolerance" must be > 0`, path)
	}
	return []string{cfg.Camera}, nil, nil
}

type ransacFitCylinder struct {
	resource.AlwaysRebuild
	resource.Named

	name resource.Name

	logger logging.Logger
	cfg    *CylinderConfig

	cam        camera.Camera
	iterations int
	tolerance  float64

	cancelCtx  context.Context
	cancelFunc func()
}

func newRansacFitCylinder(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*CylinderConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewCylinder(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// NewCylinder constructs a ransac-fit cylinder service.
func NewCylinder(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *CylinderConfig, logger logging.Logger) (resource.Resource, error) {
	cam, err := camera.FromDependencies(deps, conf.Camera)
	if err != nil {
		return nil, fmt.Errorf("could not get camera %q from dependencies: %w", conf.Camera, err)
	}

	iterations := defaultIterations
	if conf.Iterations != nil {
		iterations = *conf.Iterations
	}

	tolerance := defaultTolerance
	if conf.Tolerance != nil {
		tolerance = *conf.Tolerance
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &ransacFitCylinder{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cam:        cam,
		iterations: iterations,
		tolerance:  tolerance,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return s, nil
}

func (s *ransacFitCylinder) Name() resource.Name {
	return s.name
}

// DoCommand pulls a point cloud from the configured 3D camera and runs RANSAC
// to fit the dominant cylinder. It returns the best-fit center (a point on the
// axis), the unit axis direction, the radius, and the percentage of points
// that are inliers to that cylinder.
func (s *ransacFitCylinder) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cloud, err := s.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", s.cfg.Camera, err)
	}

	fit, err := fitCylinderRANSAC(cloud, s.iterations, s.tolerance)
	if err != nil {
		return nil, err
	}
	center := fit.Center
	axis := fit.Axis
	radius := fit.Radius
	inlierPct := fit.InlierFraction()

	s.logger.CDebugf(ctx,
		"ransac fit: iterations=%d tolerance=%.4f points=%d center=(%.6f,%.6f,%.6f) axis=(%.6f,%.6f,%.6f) radius=%.6f inliers=%.2f%%",
		s.iterations, s.tolerance, cloud.Size(),
		center.X, center.Y, center.Z,
		axis.X, axis.Y, axis.Z,
		radius, inlierPct*100,
	)

	return map[string]interface{}{
		"center":            []float64{center.X, center.Y, center.Z},
		"axis":              []float64{axis.X, axis.Y, axis.Z},
		"radius":            radius,
		"inlier_percentage": inlierPct * 100,
	}, nil
}

func (s *ransacFitCylinder) Close(context.Context) error {
	s.cancelFunc()
	return nil
}

// cylinderFit is the result of fitting a cylinder to a point cloud via RANSAC.
type cylinderFit struct {
	Center  r3.Vector // a point on the axis
	Axis    r3.Vector // unit vector along the axis
	Radius  float64
	Inliers []r3.Vector
	Total   int
}

// InlierFraction returns the fraction of total points (0-1) whose distance
// from the fitted cylinder surface is within tolerance.
func (f *cylinderFit) InlierFraction() float64 {
	if f.Total == 0 {
		return 0
	}
	return float64(len(f.Inliers)) / float64(f.Total)
}

// fitCylinderRANSAC runs RANSAC on the given point cloud to find the dominant
// cylinder and the points that support it.
func fitCylinderRANSAC(cloud pointcloud.PointCloud, iterations int, tolerance float64) (*cylinderFit, error) {
	n := cloud.Size()
	if n < cylinderSampleSize {
		return nil, fmt.Errorf("need at least %d points to fit a cylinder, got %d", cylinderSampleSize, n)
	}

	pts := make([]r3.Vector, 0, n)
	cloud.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		pts = append(pts, p)
		return true
	})

	//nolint:gosec // RANSAC sampling doesn't need a cryptographic RNG.
	rng := rand.New(rand.NewSource(1))

	bestInliers := -1
	var bestCenter r3.Vector
	var bestAxis r3.Vector
	var bestRadius float64

	sample := make([]r3.Vector, cylinderSampleSize)
	for i := 0; i < iterations; i++ {
		if !drawDistinct(rng, pts, sample) {
			continue
		}

		center, axis, radius, ok := cylinderFromPoints(sample)
		if !ok {
			continue
		}

		inliers := countCylinderInliers(pts, center, axis, radius, tolerance)
		if inliers > bestInliers {
			bestInliers = inliers
			bestCenter = center
			bestAxis = axis
			bestRadius = radius
		}
	}

	if bestInliers < 0 {
		return nil, errors.New("ransac failed to find any valid cylinder sample")
	}

	return &cylinderFit{
		Center:  bestCenter,
		Axis:    bestAxis,
		Radius:  bestRadius,
		Inliers: collectCylinderInliers(pts, bestCenter, bestAxis, bestRadius, tolerance),
		Total:   n,
	}, nil
}

// collectCylinderInliers returns the subset of pts within tolerance of the
// surface of the cylinder defined by center, unit axis, and radius.
func collectCylinderInliers(pts []r3.Vector, center, axis r3.Vector, radius, tolerance float64) []r3.Vector {
	inliers := make([]r3.Vector, 0)
	for _, p := range pts {
		d := p.Sub(center)
		alongAxis := d.Dot(axis)
		perpSq := d.Dot(d) - alongAxis*alongAxis
		if perpSq < 0 {
			perpSq = 0
		}
		dist := math.Abs(math.Sqrt(perpSq) - radius)
		if dist <= tolerance {
			inliers = append(inliers, p)
		}
	}
	return inliers
}

// drawDistinct fills out with cylinderSampleSize distinct random points from
// pts. It returns false if it could not draw enough distinct indices in a
// bounded number of attempts (which shouldn't happen for reasonable cloud
// sizes, but is here as a safety net).
func drawDistinct(rng *rand.Rand, pts []r3.Vector, out []r3.Vector) bool {
	n := len(pts)
	if n < len(out) {
		return false
	}
	picked := make(map[int]struct{}, len(out))
	filled := 0
	for attempt := 0; attempt < maxDistinctDrawAttempts*len(out) && filled < len(out); attempt++ {
		idx := rng.Intn(n)
		if _, dup := picked[idx]; dup {
			continue
		}
		picked[idx] = struct{}{}
		out[filled] = pts[idx]
		filled++
	}
	return filled == len(out)
}

// cylinderFromPoints fits a cylinder to a small set of sample points by:
//  1. Estimating the axis direction as the dominant eigenvector (largest
//     eigenvalue) of the sample's covariance matrix, via power iteration.
//     For points on a long cylinder, the axis is the direction of largest
//     spread.
//  2. Projecting the points onto a plane perpendicular to that axis.
//  3. Solving an algebraic least-squares circle fit in that 2D plane.
//  4. Lifting the 2D circle center back into 3D as a point on the axis.
//
// The second return is false if the points are degenerate (e.g. all
// effectively coincident, or projected points that don't define a circle).
func cylinderFromPoints(pts []r3.Vector) (r3.Vector, r3.Vector, float64, bool) {
	var centroid r3.Vector
	for _, p := range pts {
		centroid = centroid.Add(p)
	}
	centroid = centroid.Mul(1.0 / float64(len(pts)))

	var cov [3][3]float64
	for _, p := range pts {
		d := p.Sub(centroid)
		cov[0][0] += d.X * d.X
		cov[0][1] += d.X * d.Y
		cov[0][2] += d.X * d.Z
		cov[1][1] += d.Y * d.Y
		cov[1][2] += d.Y * d.Z
		cov[2][2] += d.Z * d.Z
	}
	cov[1][0] = cov[0][1]
	cov[2][0] = cov[0][2]
	cov[2][1] = cov[1][2]

	axis, ok := dominantEigenvector(cov, powerIterationSteps)
	if !ok {
		return r3.Vector{}, r3.Vector{}, 0, false
	}

	u, v, ok := perpBasis(axis)
	if !ok {
		return r3.Vector{}, r3.Vector{}, 0, false
	}

	pts2d := make([][2]float64, len(pts))
	for i, p := range pts {
		d := p.Sub(centroid)
		pts2d[i][0] = d.Dot(u)
		pts2d[i][1] = d.Dot(v)
	}

	cx2d, cy2d, radius, ok := fitCircle2D(pts2d)
	if !ok {
		return r3.Vector{}, r3.Vector{}, 0, false
	}

	center := centroid.Add(u.Mul(cx2d)).Add(v.Mul(cy2d))
	return center, axis, radius, true
}

// dominantEigenvector returns the unit eigenvector corresponding to the
// largest eigenvalue of a 3x3 symmetric matrix, computed by power iteration.
// The second return is false if the matrix is effectively zero (degenerate
// sample).
func dominantEigenvector(m [3][3]float64, steps int) (r3.Vector, bool) {
	v := r3.Vector{X: 1, Y: 1, Z: 1}.Mul(1.0 / math.Sqrt(3))
	for i := 0; i < steps; i++ {
		nv := r3.Vector{
			X: m[0][0]*v.X + m[0][1]*v.Y + m[0][2]*v.Z,
			Y: m[1][0]*v.X + m[1][1]*v.Y + m[1][2]*v.Z,
			Z: m[2][0]*v.X + m[2][1]*v.Y + m[2][2]*v.Z,
		}
		norm := nv.Norm()
		if norm < 1e-12 {
			return r3.Vector{}, false
		}
		v = nv.Mul(1.0 / norm)
	}
	return v, true
}

// perpBasis returns two orthonormal vectors spanning the plane perpendicular
// to axis. The second return is false if axis is effectively zero.
func perpBasis(axis r3.Vector) (r3.Vector, r3.Vector, bool) {
	if axis.Norm() < 1e-12 {
		return r3.Vector{}, r3.Vector{}, false
	}
	var ref r3.Vector
	if math.Abs(axis.X) < 0.9 {
		ref = r3.Vector{X: 1}
	} else {
		ref = r3.Vector{Y: 1}
	}
	u := axis.Cross(ref)
	un := u.Norm()
	if un < 1e-12 {
		return r3.Vector{}, r3.Vector{}, false
	}
	u = u.Mul(1.0 / un)
	v := axis.Cross(u)
	vn := v.Norm()
	if vn < 1e-12 {
		return r3.Vector{}, r3.Vector{}, false
	}
	v = v.Mul(1.0 / vn)
	return u, v, true
}

// fitCircle2D solves the algebraic least-squares fit for a 2D circle through
// the supplied points. The circle is written as x^2 + y^2 + A*x + B*y + C = 0
// which gives center (-A/2, -B/2) and radius sqrt(A^2/4 + B^2/4 - C). The
// second return is false if the normal-equation system is singular or the
// resulting radius squared is non-positive.
func fitCircle2D(pts [][2]float64) (float64, float64, float64, bool) {
	var mtm [3][3]float64
	var mtb [3]float64
	for _, p := range pts {
		x, y := p[0], p[1]
		z := -(x*x + y*y)
		mtm[0][0] += x * x
		mtm[0][1] += x * y
		mtm[0][2] += x
		mtm[1][1] += y * y
		mtm[1][2] += y
		mtm[2][2] += 1
		mtb[0] += x * z
		mtb[1] += y * z
		mtb[2] += z
	}
	mtm[1][0] = mtm[0][1]
	mtm[2][0] = mtm[0][2]
	mtm[2][1] = mtm[1][2]

	det := det3(
		mtm[0][0], mtm[0][1], mtm[0][2],
		mtm[1][0], mtm[1][1], mtm[1][2],
		mtm[2][0], mtm[2][1], mtm[2][2],
	)
	if math.Abs(det) < 1e-12 {
		return 0, 0, 0, false
	}
	a := det3(
		mtb[0], mtm[0][1], mtm[0][2],
		mtb[1], mtm[1][1], mtm[1][2],
		mtb[2], mtm[2][1], mtm[2][2],
	) / det
	b := det3(
		mtm[0][0], mtb[0], mtm[0][2],
		mtm[1][0], mtb[1], mtm[1][2],
		mtm[2][0], mtb[2], mtm[2][2],
	) / det
	c := det3(
		mtm[0][0], mtm[0][1], mtb[0],
		mtm[1][0], mtm[1][1], mtb[1],
		mtm[2][0], mtm[2][1], mtb[2],
	) / det

	cx := -a / 2
	cy := -b / 2
	r2 := cx*cx + cy*cy - c
	if r2 <= 0 {
		return 0, 0, 0, false
	}
	return cx, cy, math.Sqrt(r2), true
}

// countCylinderInliers returns how many of pts lie within tolerance of the
// surface of the cylinder defined by a point on the axis (center), a unit
// axis direction, and a radius. The signed distance from the axis is
// computed via ||(p - center) - ((p - center) . axis) * axis||.
func countCylinderInliers(pts []r3.Vector, center, axis r3.Vector, radius, tolerance float64) int {
	count := 0
	for _, p := range pts {
		d := p.Sub(center)
		alongAxis := d.Dot(axis)
		perpSq := d.Dot(d) - alongAxis*alongAxis
		if perpSq < 0 {
			perpSq = 0
		}
		dist := math.Abs(math.Sqrt(perpSq) - radius)
		if dist <= tolerance {
			count++
		}
	}
	return count
}
