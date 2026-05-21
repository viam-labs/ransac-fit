package ransacfit

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/camera"
	componentgeneric "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

// SphereComponent is the viam resource model for the RANSAC sphere fitting
// generic component, registered against the generic component API and
// exposing the fitted sphere through Geometries().
var SphereComponent = resource.NewModel("viam-labs", "ransac-fit", "sphere")

func init() {
	resource.RegisterComponent(componentgeneric.API, SphereComponent,
		resource.Registration[resource.Resource, *SphereConfig]{
			Constructor: newRansacFitSphereComponent,
		},
	)
}

type ransacFitSphereComponent struct {
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

func newRansacFitSphereComponent(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*SphereConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewSphereComponent(ctx, deps, rawConf.ResourceName(), conf, logger)
}

// NewSphereComponent constructs a ransac-fit sphere generic component.
func NewSphereComponent(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *SphereConfig, logger logging.Logger) (resource.Resource, error) {
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

	c := &ransacFitSphereComponent{
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

func (c *ransacFitSphereComponent) Name() resource.Name {
	return c.name
}

func (c *ransacFitSphereComponent) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cloud, err := c.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", c.cfg.Camera, err)
	}
	fit, err := fitSphereRANSAC(cloud, c.iterations, c.tolerance)
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"center":            []float64{fit.Center.X, fit.Center.Y, fit.Center.Z},
		"radius":            fit.Radius,
		"inlier_percentage": fit.InlierFraction() * 100,
	}, nil
}

// Geometries pulls a fresh point cloud, runs RANSAC, and returns the fitted
// sphere as a Sphere geometry primitive sitting at the fitted center with the
// fitted radius.
func (c *ransacFitSphereComponent) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	cloud, err := c.cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get point cloud from camera %q: %w", c.cfg.Camera, err)
	}
	fit, err := fitSphereRANSAC(cloud, c.iterations, c.tolerance)
	if err != nil {
		return nil, err
	}
	if fit.Radius <= 0 {
		return nil, fmt.Errorf("ransac fit produced non-positive sphere radius %.4f", fit.Radius)
	}

	pose := spatialmath.NewPoseFromPoint(fit.Center)
	sphere, err := spatialmath.NewSphere(pose, fit.Radius, c.name.ShortName())
	if err != nil {
		return nil, fmt.Errorf("could not construct sphere geometry: %w", err)
	}

	c.logger.CDebugf(ctx,
		"sphere geom: center=(%.2f,%.2f,%.2f) radius=%.4f inliers=%d/%d",
		fit.Center.X, fit.Center.Y, fit.Center.Z,
		fit.Radius, len(fit.Inliers), fit.Total,
	)

	return []spatialmath.Geometry{sphere}, nil
}

func (c *ransacFitSphereComponent) Close(context.Context) error {
	c.cancelFunc()
	return nil
}
