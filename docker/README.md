# Docker

Development container for the project.

## Dependencies

```bash
sudo apt install git make curl
curl https://get.docker.com | sh && sudo systemctl --now enable docker
```

## Usage

You need to build & run the container only the first time:
```bash
make build run exec
```

After that, you can just attach to the container with:
```bash
make exec
```
