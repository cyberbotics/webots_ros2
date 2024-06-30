UID:=$(shell id -u)
ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))/../
NVIDIA_GPU:=$(shell docker info | grep Runtimes | grep nvidia 1> /dev/null && nvidia-smi 1> /dev/null && echo '--runtime nvidia --gpus all' || echo '')
DOCKERUSER:=cyberbotics
BASENAME:=webots_ros2
DOCKERUSERCOMMENT:="Cyberbotics"
ROS_DISTRO:=jazzy


build:
	docker build ${ROOT_DIR} \
		-f ${ROOT_DIR}/docker/Dockerfile \
		-t ${BASENAME}-image ${DOCKER_ARGS} \
		--build-arg UID=${UID} \
		--build-arg DOCKERUSER=${DOCKERUSER} \
		--build-arg DOCKERUSERCOMMENT=${DOCKERUSERCOMMENT} \
		--build-arg ROS_DISTRO=${ROS_DISTRO} --progress=plain


run:
	@docker container rm -f ${BASENAME}-container
	docker run \
		--net=host \
		--ipc=host \
		--cap-add SYS_ADMIN \
		--name ${BASENAME}-container \
		--privileged \
		--restart unless-stopped \
		-e NVIDIA_DRIVER_CAPABILITIES=all ${NVIDIA_GPU} \
		-e DISPLAY=${DISPLAY} \
		-v ~/.Xauthority:/${DOCKERUSER}/.Xauthority:ro \
		-v /tmp/.X11-unix/:/tmp/.X11-unix:rw \
		-v /dev/dri:/dev/dri:ro \
		-v ${ROOT_DIR}:/${DOCKERUSER}/ros2_ws/src/${BASENAME} \
		-v /dev:/dev:rw \
		--entrypoint bash \
		-d -it ${BASENAME}-image


exec:
	docker exec -it ${BASENAME}-container bash
