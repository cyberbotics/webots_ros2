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
ROS_DISTRO=jazzy ROS_TESTING=1 WEBOTS_VERSION=2023b make build run exec
```

After that, you can just attach to the container with:
```bash
make exec
```
