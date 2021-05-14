# Kumo

[![latest version](https://img.shields.io/github/v/release/ichiro-its/kumo)](https://github.com/ichiro-its/kumo/releases/)
[![commits since latest version](https://img.shields.io/github/commits-since/ichiro-its/kumo/latest)](https://github.com/ichiro-its/kumo/commits/master)
[![milestone](https://img.shields.io/github/milestones/progress/ichiro-its/kumo/1?label=milestone)](https://github.com/ichiro-its/kumo/milestone/1)
[![license](https://img.shields.io/github/license/ichiro-its/kumo)](./LICENSE)
[![test status](https://img.shields.io/github/workflow/status/ichiro-its/kumo/Build%20and%20Test?label=test)](https://github.com/ichiro-its/kumo/actions)

This package provides a bridge server that enables communications between [WebSocket](https://en.wikipedia.org/wiki/WebSocket) applications and [ROS 2](https://docs.ros.org/en/foxy/index.html) nodes.

## Usage


- Follow [this guide](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) to setting up a ROS 2 workspace.
- Run the bridge server using the following command:
  ```
  $ ros2 run kumo bridge
  ```
  > Make sure [the Python websockets](https://pypi.org/project/websockets/) already installed before running the bridge server.

## License

This project is maintained by [ICHIRO ITS](https://github.com/ichiro-its) and licensed under the [MIT License](./LICENSE).
