# Kumo (雲)

[![latest version](https://img.shields.io/github/v/release/ichiro-its/kumo)](https://github.com/ichiro-its/kumo/releases/)
[![commits since latest version](https://img.shields.io/github/commits-since/ichiro-its/kumo/latest)](https://github.com/ichiro-its/kumo/commits/master)
[![license](https://img.shields.io/github/license/ichiro-its/kumo)](./LICENSE)
[![deploy stable status](https://img.shields.io/github/workflow/status/ichiro-its/kumo/Deploy%20Debian%20Stable?label=deploy%20stable)](https://repository.ichiro-its.org/)
[![deploy nightly status](https://img.shields.io/github/workflow/status/ichiro-its/kumo/Deploy%20Debian%20Nightly?label=deploy%20nightly)](https://repository.ichiro-its.org/)

Kumo (雲, cloud) is a [ROS 2](https://docs.ros.org/en/foxy/index.html) package that provides a bridge server that enables communications between [WebSocket](https://en.wikipedia.org/wiki/WebSocket) applications and ROS 2 nodes.

## Features

- Serve ROS 2 bridge on a specified port and host.
- Support Node, Publisher, Subscription, Client, and Service creation.
- Support Topics and Services communication.

## Requirement

- [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/).
- [Yakusha](https://github.com/ichiro-its/yakusha) (for JSON serialization).
- [websockets](https://github.com/aaugustin/websockets).

## Installation

### Binary Packages

- See [releases](https://github.com/ichiro-its/kumo/releases) for the latest version of this package.
- Alternatively, this package also available on [ICHIRO ITS Repository](https://repository.ichiro-its.org/) as `ros-foxy-kumo` package.

### Build From Source

- Install colcon as in [this guide](https://colcon.readthedocs.io/en/released/user/installation.html).
- Build the project and source the result.
  ```bash
  $ colcon build
  $ source install/setup.bash
  ```
  > See [this guide](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) for more information on how to setup a workspace in ROS 2.

## Usage

Run the bridge server using `bridge` program (use `-h` to list all available parameters).

```
$ ros2 run kumo bridge
```

## Supported Client Libraries

- [Kumo Client](https://github.com/ichiro-its/kumo-client) (JavaScript).

## License

This project is maintained by [ICHIRO ITS](https://github.com/ichiro-its) and licensed under the [MIT License](./LICENSE).
