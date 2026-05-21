package ransacfit

import (
	"context"
	"fmt"
	"math"

	"go.viam.com/rdk/components/camera"
	componentgeneric "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// CylinderComponent is the viam resource model for the RANSAC cylinder
// fitting generic component, registered against the generic component API
// and exposing the fitted cylinder through Geometries().
var CylinderComponent = resource.NewModel("viamlabs", "ransac-fit", "cylinder")

func init() {
	resource.RegisterComponent(componentgeneric.API, CylinderComponent,
		resource.Registration[resource.Resource, *CylinderConfig]{
			Constructor: newRansacFitCylinderComponent,
		},
	)
}

type ransacFitCylinderComponent struct {
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

func newRansacFitCylinderComponent(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*CylinderConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewCylinderComponent(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// NewCylinderComponent constructs a ransac-fit cylinder generic component.
func NewCylinderComponent(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *CylinderConfig, logger logging.Logger) (resource.Resource, error) {
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

	c := &ransacFitCylinderComponent{
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

func (c *ransacFitCylinderComponent) Name() resource.Name {
	return c.name
}

func (c *ransacFitCylinderComponent) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cloud, err := c.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", c.cfg.Camera, err)
	}
	fit, err := fitCylinderRANSAC(cloud, c.iterations, c.tolerance)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"center":            []float64{fit.Center.X, fit.Center.Y, fit.Center.Z},
		"axis":              []float64{fit.Axis.X, fit.Axis.Y, fit.Axis.Z},
		"radius":            fit.Radius,
		"inlier_percentage": fit.InlierFraction() * 100,
	}, nil
}

// Geometries pulls a fresh point cloud, runs RANSAC, and returns the fitted
// cylinder as a Cylinder geometry primitive. The cylinder's local Z axis is
// aligned with the fitted axis; its height is the inlier extent along the
// axis; the geometry pose is centered at the midpoint of that axial extent.
func (c *ransacFitCylinderComponent) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	cloud, err := c.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", c.cfg.Camera, err)
	}
	fit, err := fitCylinderRANSAC(cloud, c.iterations, c.tolerance)
	if err != nil {
		return nil, err
	}
	if len(fit.Inliers) == 0 {
		return nil, fmt.Errorf("ransac fit found 0 inliers; cannot size cylinder geometry")
	}
	if fit.Radius <= 0 {
		return nil, fmt.Errorf("ransac fit produced non-positive cylinder radius %.4f", fit.Radius)
	}

	minT := math.Inf(1)
	maxT := math.Inf(-1)
	for _, p := range fit.Inliers {
		t := p.Sub(fit.Center).Dot(fit.Axis)
		if t < minT {
			minT = t
		}
		if t > maxT {
			maxT = t
		}
	}
	height := maxT - minT
	if height <= 0 {
		return nil, fmt.Errorf("degenerate cylinder height from inliers: %.4f", height)
	}

	cylCenter := fit.Center.Add(fit.Axis.Mul((minT + maxT) / 2))
	pose := spatialmath.NewPose(cylCenter, &spatialmath.OrientationVector{
		OX: fit.Axis.X, OY: fit.Axis.Y, OZ: fit.Axis.Z,
	})
	cyl, err := spatialmath.NewCylinder(pose, fit.Radius, height, c.name.ShortName())
	if err != nil {
		return nil, fmt.Errorf("could not construct cylinder geometry: %w", err)
	}

	c.logger.CDebugf(ctx,
		"cylinder geom: center=(%.2f,%.2f,%.2f) axis=(%.4f,%.4f,%.4f) radius=%.4f height=%.4f inliers=%d/%d",
		cylCenter.X, cylCenter.Y, cylCenter.Z,
		fit.Axis.X, fit.Axis.Y, fit.Axis.Z,
		fit.Radius, height, len(fit.Inliers), fit.Total,
	)

	return []spatialmath.Geometry{cyl}, nil
}

func (c *ransacFitCylinderComponent) Close(context.Context) error {
	c.cancelFunc()
	return nil
}
