package main

import (
	"ransacfit"

	componentgeneric "go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	servicegeneric "go.viam.com/rdk/services/generic"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		// Generic services: DoCommand-only RANSAC fits.
		resource.APIModel{API: servicegeneric.API, Model: ransacfit.Plane},
		resource.APIModel{API: servicegeneric.API, Model: ransacfit.Sphere},
		resource.APIModel{API: servicegeneric.API, Model: ransacfit.Cylinder},
		// Generic components: same RANSAC fits, with the result also rendered
		// through Geometries() as a spatialmath primitive (Box / Sphere /
		// Cylinder).
		resource.APIModel{API: componentgeneric.API, Model: ransacfit.PlaneComponent},
		resource.APIModel{API: componentgeneric.API, Model: ransacfit.SphereComponent},
		resource.APIModel{API: componentgeneric.API, Model: ransacfit.CylinderComponent},
	)
}
