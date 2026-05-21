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

var (
	Plane            = resource.NewModel("viamlabs", "ransac-fit", "plane")
	errUnimplemented = errors.New("unimplemented")
)

const (
	defaultIterations = 200
	defaultTolerance  = 10.0
	planeSampleSize   = 3
)

func init() {
	resource.RegisterService(generic.API, Plane,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newRansacFitPlane,
		},
	)
}

// Config is the user-provided configuration for the ransac-fit plane service.
//
// Camera is the name of a 3D camera (one that supports NextPointCloud) that
// this service will pull point clouds from.
//
// Iterations is the number of RANSAC iterations to run. Optional, defaults to 200.
//
// Tolerance is the maximum distance (in the point cloud's units, typically mm)
// from the candidate plane for a point to be counted as an inlier. Optional,
// defaults to 10.
type Config struct {
	Camera     string   `json:"camera"`
	Iterations *int     `json:"iterations,omitempty"`
	Tolerance  *float64 `json:"tolerance,omitempty"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns three values:
//  1. Required dependencies: other resources that must exist for this resource to work.
//  2. Optional dependencies: other resources that may exist but are not required.
//  3. An error if any Config fields are missing or invalid.
//
// The `path` parameter indicates
// where this resource appears in the machine's JSON configuration
// (for example, "components.0"). You can use it in error messages
// to indicate which resource has a problem.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
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

type ransacFitPlane struct {
	resource.AlwaysRebuild
	resource.Named

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cam        camera.Camera
	iterations int
	tolerance  float64

	cancelCtx  context.Context
	cancelFunc func()
}

func newRansacFitPlane(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewPlane(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewPlane(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
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

	s := &ransacFitPlane{
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

func (s *ransacFitPlane) Name() resource.Name {
	return s.name
}

// DoCommand pulls a point cloud from the configured 3D camera and runs RANSAC
// to fit the dominant plane. It returns the best-fit plane normal along with
// the percentage of points that are inliers to that plane.
func (s *ransacFitPlane) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cloud, err := s.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", s.cfg.Camera, err)
	}

	fit, err := fitPlaneRANSAC(cloud, s.iterations, s.tolerance)
	if err != nil {
		return nil, err
	}
	normal := fit.Normal()
	inlierPct := fit.InlierFraction()

	s.logger.CDebugf(ctx,
		"ransac fit: iterations=%d tolerance=%.4f points=%d normal=(%.6f,%.6f,%.6f) inliers=%.2f%%",
		s.iterations, s.tolerance, cloud.Size(), normal.X, normal.Y, normal.Z, inlierPct*100,
	)

	return map[string]interface{}{
		"normal":            []float64{normal.X, normal.Y, normal.Z},
		"inlier_percentage": inlierPct,
	}, nil
}

func (s *ransacFitPlane) Close(context.Context) error {
	s.cancelFunc()
	return nil
}

// planeFit is the result of fitting a plane to a point cloud via RANSAC.
// Equation is in the form ax + by + cz + d = 0 with (a, b, c) un-normalized.
type planeFit struct {
	Equation [4]float64
	Inliers  []r3.Vector
	Total    int
}

// Normal returns the unit normal to the fitted plane.
func (f *planeFit) Normal() r3.Vector {
	n := r3.Vector{X: f.Equation[0], Y: f.Equation[1], Z: f.Equation[2]}
	if norm := n.Norm(); norm > 0 {
		return n.Mul(1.0 / norm)
	}
	return n
}

// InlierFraction returns the fraction of total points (0-1) that lie within
// tolerance of the fitted plane.
func (f *planeFit) InlierFraction() float64 {
	if f.Total == 0 {
		return 0
	}
	return float64(len(f.Inliers)) / float64(f.Total)
}

// fitPlaneRANSAC runs RANSAC on the given point cloud to find the dominant
// plane and the points that support it.
func fitPlaneRANSAC(cloud pointcloud.PointCloud, iterations int, tolerance float64) (*planeFit, error) {
	n := cloud.Size()
	if n < planeSampleSize {
		return nil, fmt.Errorf("need at least %d points to fit a plane, got %d", planeSampleSize, n)
	}

	pts := make([]r3.Vector, 0, n)
	cloud.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		pts = append(pts, p)
		return true
	})

	//nolint:gosec // RANSAC sampling doesn't need a cryptographic RNG.
	rng := rand.New(rand.NewSource(1))

	bestInliers := -1
	var bestEq [4]float64

	for i := 0; i < iterations; i++ {
		i1, i2, i3 := rng.Intn(n), rng.Intn(n), rng.Intn(n)
		if i1 == i2 || i1 == i3 || i2 == i3 {
			continue
		}

		eq, ok := planeFromPoints(pts[i1], pts[i2], pts[i3])
		if !ok {
			continue
		}

		inliers := countInliers(pts, eq, tolerance)
		if inliers > bestInliers {
			bestInliers = inliers
			bestEq = eq
		}
	}

	if bestInliers < 0 {
		return nil, errors.New("ransac failed to find any valid plane sample")
	}

	return &planeFit{
		Equation: bestEq,
		Inliers:  collectPlaneInliers(pts, bestEq, tolerance),
		Total:    n,
	}, nil
}

// collectPlaneInliers returns the subset of pts that lie within tolerance of
// the plane defined by ax + by + cz + d = 0.
func collectPlaneInliers(pts []r3.Vector, eq [4]float64, tolerance float64) []r3.Vector {
	norm := math.Sqrt(eq[0]*eq[0] + eq[1]*eq[1] + eq[2]*eq[2])
	if norm == 0 {
		return nil
	}
	inv := 1.0 / norm
	inliers := make([]r3.Vector, 0)
	for _, p := range pts {
		dist := math.Abs(eq[0]*p.X+eq[1]*p.Y+eq[2]*p.Z+eq[3]) * inv
		if dist <= tolerance {
			inliers = append(inliers, p)
		}
	}
	return inliers
}

// planeFromPoints returns the plane equation ax + by + cz + d = 0 fit through
// three points. The second return is false if the points are collinear (and
// therefore do not define a unique plane).
func planeFromPoints(p1, p2, p3 r3.Vector) ([4]float64, bool) {
	v1 := p2.Sub(p1)
	v2 := p3.Sub(p1)
	normal := v1.Cross(v2)
	if normal.Norm() < 1e-9 {
		return [4]float64{}, false
	}
	d := -normal.Dot(p1)
	return [4]float64{normal.X, normal.Y, normal.Z, d}, true
}

// countInliers returns how many of pts lie within tolerance of the plane
// defined by ax + by + cz + d = 0.
func countInliers(pts []r3.Vector, eq [4]float64, tolerance float64) int {
	norm := math.Sqrt(eq[0]*eq[0] + eq[1]*eq[1] + eq[2]*eq[2])
	if norm == 0 {
		return 0
	}
	inv := 1.0 / norm
	count := 0
	for _, p := range pts {
		dist := math.Abs(eq[0]*p.X+eq[1]*p.Y+eq[2]*p.Z+eq[3]) * inv
		if dist <= tolerance {
			count++
		}
	}
	return count
}
