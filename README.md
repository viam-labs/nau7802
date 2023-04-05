# nau7802
Module for a load cell bridge sensor amplifier

## Usage

### 1. Build binary

If you clone this repository to the target environment where you run your Viam robot, then you can build a binary named `nau7802` with:

```
go build -o nau7802
```

Alternatively, if you want to build a binary for a different target environment, please use the [viam canon tool](https://github.com/viamrobotics/canon).

### 2. Add to robot configuration

Copy the binary to the robot (system where viam-server is running) and add the following to your configuration:

```
  ...
  "modules": [
    ...,
    {
      "executable_path": "<path_to_binary>",
      "name": "nau7802-module"
    },
    ...,
  ],
  "components": [
    ...,
    {
      "name": "",
      "type": "sensor",
      "model": "viam-labs:sensor:nau7802-module"
    },
    ...,
  ],
  ...
```

For more information on how to configure modular components, [see this example](https://docs.viam.com/services/slam/run-slam-cartographer/#step-1-add-your-rdiplar-as-a-modular-component).
