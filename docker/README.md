# Docker

Development container for the project.

## Dependencies

```bash
sudo apt install git make curl
curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER
```

## Usage

You need to build & run the container only the first time:
```bash
ROS_DISTRO=jazzy ROS_TESTING=0 WEBOTS_VERSION=2025a make build run exec
```

if you want to test a nightly build:
```bash
ROS_DISTRO=jazzy ROS_TESTING=0 WEBOTS_VERSION=nightly_25_12_2024/webots_2025a make build run exec
```

After that, you can just attach to the container with:
```bash
make exec
```

> [!NOTE]
> In case you get an error something like `qt.qpa.xcb: could not connect to display :0` then please run `xhost local:root` on the host.
