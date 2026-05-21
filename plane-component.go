package ransacfit

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	componentgeneric "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// PlaneComponent is the viam resource model for the RANSAC plane fitting
// generic component. It is registered against the generic component API
// (distinct from the generic service of the same name) and exposes the fitted
// plane through Geometries() as a thin Box.
var PlaneComponent = resource.NewModel("viamlabs", "ransac-fit", "plane")

func init() {
	resource.RegisterComponent(componentgeneric.API, PlaneComponent,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newRansacFitPlaneComponent,
		},
	)
}

type ransacFitPlaneComponent struct {
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

func newRansacFitPlaneComponent(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewPlaneComponent(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// NewPlaneComponent constructs a ransac-fit plane generic component.
func NewPlaneComponent(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {
	cam, err := camera.FromProvider(deps, conf.Camera)
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

	c := &ransacFitPlaneComponent{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cam:        cam,
		iterations: iterations,
		tolerance:  tolerance,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}
	return c, nil
}

func (c *ransacFitPlaneComponent) Name() resource.Name {
	return c.name
}

// DoCommand exposes the same RANSAC summary as the service variant: the unit
// plane normal and the inlier fraction.
func (c *ransacFitPlaneComponent) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cloud, err := c.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", c.cfg.Camera, err)
	}
	fit, err := fitPlaneRANSAC(cloud, c.iterations, c.tolerance)
	if err != nil {
		return nil, err
	}
	normal := fit.Normal()
	return map[string]interface{}{
		"normal":            []float64{normal.X, normal.Y, normal.Z},
		"inlier_percentage": fit.InlierFraction(),
	}, nil
}

// Geometries pulls a fresh point cloud, runs RANSAC, and returns the fitted
// plane as a thin Box geometry. The box is oriented so its local Z axis is
// aligned with the plane normal; its in-plane (X, Y) extents are sized from
// the bounding rectangle of the inlier points projected into the plane; its
// Z thickness is 2 * tolerance, so the rendered box shows the tolerance band
// around the fitted plane.
func (c *ransacFitPlaneComponent) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	cloud, err := c.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", c.cfg.Camera, err)
	}
	fit, err := fitPlaneRANSAC(cloud, c.iterations, c.tolerance)
	if err != nil {
		return nil, err
	}
	if len(fit.Inliers) == 0 {
		return nil, fmt.Errorf("ransac fit found 0 inliers; cannot size plane geometry")
	}

	normal := fit.Normal()
	u, v, ok := perpBasis(normal)
	if !ok {
		return nil, fmt.Errorf("could not build perpendicular basis for plane normal %v", normal)
	}

	var centroid r3.Vector
	for _, p := range fit.Inliers {
		centroid = centroid.Add(p)
	}
	centroid = centroid.Mul(1.0 / float64(len(fit.Inliers)))

	minU, maxU, minV, maxV := projectExtents(fit.Inliers, centroid, u, v)
	width := maxU - minU
	height := maxV - minV
	if width <= 0 || height <= 0 {
		return nil, fmt.Errorf("degenerate inlier extents: width=%.4f height=%.4f", width, height)
	}
	thickness := 2 * c.tolerance

	boxCenter := centroid.
		Add(u.Mul((minU + maxU) / 2)).
		Add(v.Mul((minV + maxV) / 2))

	pose := spatialmath.NewPose(boxCenter, &spatialmath.OrientationVector{
		OX: normal.X, OY: normal.Y, OZ: normal.Z,
	})

	box, err := spatialmath.NewBox(pose, r3.Vector{X: width, Y: height, Z: thickness}, c.name.ShortName())
	if err != nil {
		return nil, fmt.Errorf("could not construct plane box geometry: %w", err)
	}

	c.logger.CDebugf(ctx,
		"plane geom: normal=(%.4f,%.4f,%.4f) center=(%.2f,%.2f,%.2f) dims=(%.2f,%.2f,%.2f) inliers=%d/%d",
		normal.X, normal.Y, normal.Z,
		boxCenter.X, boxCenter.Y, boxCenter.Z,
		width, height, thickness,
		len(fit.Inliers), fit.Total,
	)

	return []spatialmath.Geometry{box}, nil
}

func (c *ransacFitPlaneComponent) Close(context.Context) error {
	c.cancelFunc()
	return nil
}

// projectExtents projects each point onto the (u, v) basis relative to origin
// and returns the min/max u and min/max v coordinates.
func projectExtents(pts []r3.Vector, origin, u, v r3.Vector) (float64, float64, float64, float64) {
	first := pts[0].Sub(origin)
	minU := first.Dot(u)
	maxU := minU
	minV := first.Dot(v)
	maxV := minV
	for _, p := range pts[1:] {
		d := p.Sub(origin)
		cu := d.Dot(u)
		cv := d.Dot(v)
		if cu < minU {
			minU = cu
		}
		if cu > maxU {
			maxU = cu
		}
		if cv < minV {
			minV = cv
		}
		if cv > maxV {
			maxV = cv
		}
	}
	return minU, maxU, minV, maxV
}
