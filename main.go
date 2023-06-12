// package main is a module with a nau7802 sensor component
package main

import (
	"context"

	"github.com/edaniels/golog"
	"github.com/viam-labs/nau7802/nau7802module"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/module"
	"go.viam.com/utils"
)

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewDevelopmentLogger("nau7802"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	nau7802SensorModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	nau7802SensorModule.AddModelFromRegistry(ctx, sensor.API, nau7802module.Model)

	err = nau7802SensorModule.Start(ctx)
	defer nau7802SensorModule.Close(ctx)
	if err != nil {
		return err
	}

	<-ctx.Done()
	return nil
}
