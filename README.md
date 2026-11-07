# ft-tilt

[![CI](https://github.com/alvgaona/ft-tilt/actions/workflows/ci.yaml/badge.svg)](https://github.com/alvgaona/ft-tilt/actions/workflows/ci.yaml)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

A hexacopter with tilted rotors that keeps full attitude control after losing a motor, simulated in
ROS 2 Jazzy and Gazebo Harmonic. It reproduces the fault-tolerant design and flight experiments of
Giribet, Pose, Ghersin and Mas, and flies with Aerostack2.

## Quick start

Install [pixi](https://pixi.sh), then

```sh
pixi run build
pixi run test
pixi run sim gamma_deg:=107
```

Fly a mission with a motor failure

```sh
pixi run as2
pixi run mission --fault-motor 2
```

## Packages

- `ft_tilt_control` is the allocation, attitude and position control library (C++ and Eigen, no ROS)
- `ft_tilt_gazebo` is the Gazebo flight controller plugin
- `ft_tilt_description` holds the vehicle model and meshes
- `ft_tilt_bringup` has the world, launch files and paper experiments
- `ft_tilt_as2` is the Aerostack2 platform and motion controller

## References

- Giribet, Pose, Mas. *Fault tolerant control of an hexacopter with a tilted-rotor configuration*. RPIC 2017
- Giribet, Pose, Ghersin, Mas. *Experimental validation of a fault-tolerant hexacopter with tilted rotors*. ICRIS 2018
- Giribet, Sánchez-Peña, Ghersin. *Analysis and design of a tilted rotor hexacopter for fault tolerance*. IEEE TAES 52(4), 2016

## Assets

- DJI F550 frame from [Printables 1436460](https://www.printables.com/model/1436460) (public domain)
- Iris propellers from [PX4](https://github.com/PX4/PX4-SITL_gazebo-classic) (BSD 3-Clause)
- Jetson developer kit from NVIDIA's CAD model

## License

[MIT](LICENSE)
