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

// Sphere is the viam resource model for the RANSAC sphere fitting service.
var Sphere = resource.NewModel("viam-labs", "ransac-fit", "sphere")

const sphereSampleSize = 4

func init() {
	resource.RegisterService(generic.API, Sphere,
		resource.Registration[resource.Resource, *SphereConfig]{
			Constructor: newRansacFitSphere,
		},
	)
}

// SphereConfig is the user-provided configuration for the ransac-fit sphere service.
//
// Camera is the name of a 3D camera (one that supports NextPointCloud) that
// this service will pull point clouds from.
//
// Iterations is the number of RANSAC iterations to run. Optional, defaults to 200.
//
// Tolerance is the maximum distance (in the point cloud's units, typically mm)
// from the candidate sphere surface for a point to be counted as an inlier.
// Optional, defaults to 10.
type SphereConfig struct {
	Camera     string   `json:"camera"`
	Iterations *int     `json:"iterations,omitempty"`
	Tolerance  *float64 `json:"tolerance,omitempty"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// It also returns the camera as a required dependency.
func (cfg *SphereConfig) Validate(path string) ([]string, []string, error) {
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

type ransacFitSphere struct {
	resource.AlwaysRebuild
	resource.Named

	name resource.Name

	logger logging.Logger
	cfg    *SphereConfig

	cam        camera.Camera
	iterations int
	tolerance  float64

	cancelCtx  context.Context
	cancelFunc func()
}

func newRansacFitSphere(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*SphereConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewSphere(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// NewSphere constructs a ransac-fit sphere service.
func NewSphere(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *SphereConfig, logger logging.Logger) (resource.Resource, error) {
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

	s := &ransacFitSphere{
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

func (s *ransacFitSphere) Name() resource.Name {
	return s.name
}

// DoCommand pulls a point cloud from the configured 3D camera and runs RANSAC
// to fit the dominant sphere. It returns the best-fit center, radius, and the
// percentage of points that are inliers to that sphere.
func (s *ransacFitSphere) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cloud, err := s.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", s.cfg.Camera, err)
	}

	fit, err := fitSphereRANSAC(cloud, s.iterations, s.tolerance)
	if err != nil {
		return nil, err
	}
	center := fit.Center
	radius := fit.Radius
	inlierPct := fit.InlierFraction()

	s.logger.CDebugf(ctx,
		"ransac fit: iterations=%d tolerance=%.4f points=%d center=(%.6f,%.6f,%.6f) radius=%.6f inliers=%.2f%%",
		s.iterations, s.tolerance, cloud.Size(), center.X, center.Y, center.Z, radius, inlierPct*100,
	)

	return map[string]interface{}{
		"center":            []float64{center.X, center.Y, center.Z},
		"radius":            radius,
		"inlier_percentage": inlierPct * 100,
	}, nil
}

func (s *ransacFitSphere) Close(context.Context) error {
	s.cancelFunc()
	return nil
}

// sphereFit is the result of fitting a sphere to a point cloud via RANSAC.
type sphereFit struct {
	Center  r3.Vector
	Radius  float64
	Inliers []r3.Vector
	Total   int
}

// InlierFraction returns the fraction of total points (0-1) whose distance
// from the fitted sphere surface is within tolerance.
func (f *sphereFit) InlierFraction() float64 {
	if f.Total == 0 {
		return 0
	}
	return float64(len(f.Inliers)) / float64(f.Total)
}

// fitSphereRANSAC runs RANSAC on the given point cloud to find the dominant
// sphere and the points that support it.
func fitSphereRANSAC(cloud pointcloud.PointCloud, iterations int, tolerance float64) (*sphereFit, error) {
	n := cloud.Size()
	if n < sphereSampleSize {
		return nil, fmt.Errorf("need at least %d points to fit a sphere, got %d", sphereSampleSize, n)
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
	var bestRadius float64

	for i := 0; i < iterations; i++ {
		i1, i2, i3, i4 := rng.Intn(n), rng.Intn(n), rng.Intn(n), rng.Intn(n)
		if i1 == i2 || i1 == i3 || i1 == i4 || i2 == i3 || i2 == i4 || i3 == i4 {
			continue
		}

		center, radius, ok := sphereFromPoints(pts[i1], pts[i2], pts[i3], pts[i4])
		if !ok {
			continue
		}

		inliers := countSphereInliers(pts, center, radius, tolerance)
		if inliers > bestInliers {
			bestInliers = inliers
			bestCenter = center
			bestRadius = radius
		}
	}

	if bestInliers < 0 {
		return nil, errors.New("ransac failed to find any valid sphere sample")
	}

	return &sphereFit{
		Center:  bestCenter,
		Radius:  bestRadius,
		Inliers: collectSphereInliers(pts, bestCenter, bestRadius, tolerance),
		Total:   n,
	}, nil
}

// collectSphereInliers returns the subset of pts within tolerance of the
// surface of the sphere defined by center and radius.
func collectSphereInliers(pts []r3.Vector, center r3.Vector, radius, tolerance float64) []r3.Vector {
	inliers := make([]r3.Vector, 0)
	for _, p := range pts {
		dist := math.Abs(p.Sub(center).Norm() - radius)
		if dist <= tolerance {
			inliers = append(inliers, p)
		}
	}
	return inliers
}

// sphereFromPoints solves for the unique sphere passing through 4 non-coplanar
// points. The general sphere equation is:
//
//	x^2 + y^2 + z^2 + D*x + E*y + F*z + G = 0
//
// which gives center (-D/2, -E/2, -F/2) and radius sqrt(D^2/4 + E^2/4 + F^2/4 - G).
// Subtracting the equation for p1 from each of p2, p3, p4 eliminates G and
// yields a 3x3 linear system in (D, E, F) that we solve via Cramer's rule.
// The second return is false if the 4 points are coplanar (no unique sphere)
// or the resulting radius squared is non-positive (numerical degeneracy).
func sphereFromPoints(p1, p2, p3, p4 r3.Vector) (r3.Vector, float64, bool) {
	s1 := p1.X*p1.X + p1.Y*p1.Y + p1.Z*p1.Z

	var coeff [3][3]float64
	var rhs [3]float64
	for i, p := range [3]r3.Vector{p2, p3, p4} {
		si := p.X*p.X + p.Y*p.Y + p.Z*p.Z
		coeff[i][0] = p.X - p1.X
		coeff[i][1] = p.Y - p1.Y
		coeff[i][2] = p.Z - p1.Z
		rhs[i] = -(si - s1)
	}

	det := det3(
		coeff[0][0], coeff[0][1], coeff[0][2],
		coeff[1][0], coeff[1][1], coeff[1][2],
		coeff[2][0], coeff[2][1], coeff[2][2],
	)
	if math.Abs(det) < 1e-9 {
		return r3.Vector{}, 0, false
	}

	detD := det3(
		rhs[0], coeff[0][1], coeff[0][2],
		rhs[1], coeff[1][1], coeff[1][2],
		rhs[2], coeff[2][1], coeff[2][2],
	)
	detE := det3(
		coeff[0][0], rhs[0], coeff[0][2],
		coeff[1][0], rhs[1], coeff[1][2],
		coeff[2][0], rhs[2], coeff[2][2],
	)
	detF := det3(
		coeff[0][0], coeff[0][1], rhs[0],
		coeff[1][0], coeff[1][1], rhs[1],
		coeff[2][0], coeff[2][1], rhs[2],
	)

	d := detD / det
	e := detE / det
	f := detF / det
	g := -(s1 + d*p1.X + e*p1.Y + f*p1.Z)

	cx := -d / 2
	cy := -e / 2
	cz := -f / 2
	r2 := cx*cx + cy*cy + cz*cz - g
	if r2 <= 0 {
		return r3.Vector{}, 0, false
	}
	return r3.Vector{X: cx, Y: cy, Z: cz}, math.Sqrt(r2), true
}

func det3(a, b, c, d, e, f, g, h, i float64) float64 {
	return a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g)
}

// countSphereInliers returns how many of pts lie within tolerance of the surface
// of the sphere defined by center and radius.
func countSphereInliers(pts []r3.Vector, center r3.Vector, radius, tolerance float64) int {
	count := 0
	for _, p := range pts {
		dist := math.Abs(p.Sub(center).Norm() - radius)
		if dist <= tolerance {
			count++
		}
	}
	return count
}
