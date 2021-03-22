# Kumo

[![latest version](https://img.shields.io/github/v/release/ichiro-its/kumo.svg)](https://github.com/ichiro-its/kumo/releases/)
[![license](https://img.shields.io/github/license/ichiro-its/kumo.svg)](./LICENSE)
[![build and test status](https://github.com/ichiro-its/kumo/actions/workflows/build-and-test.yml/badge.svg)](https://github.com/ichiro-its/kumo/actions)

This package provides a bridge server that enables communications between [WebSocket](https://en.wikipedia.org/wiki/WebSocket) applications and [ROS 2](https://docs.ros.org/en/foxy/index.html) nodes.

## Usage


- Follow [this guide](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) to setting up a ROS 2 workspace.
- Run the bridge server using the following command:
  ```
  $ ros2 run kumo bridge
  ```
  > Make sure [the Python websockets](https://pypi.org/project/websockets/) already installed before running the bridge server.

## License

This project is licensed under [the MIT License](./LICENSE).
