#!/bin/bash
set -e

IMAGE_NAME="ros2_dev:latest"
CONTAINER_NAME="ros2_container"

# Проверяем, существует ли образ
if ! docker image inspect ${IMAGE_NAME} > /dev/null 2>&1; then
    echo "Образ ${IMAGE_NAME} не найден. Сначала запустите ./build_img.sh"
    exit 1
fi

docker rm -f ${CONTAINER_NAME} > /dev/null 2>&1 || true

if [ -x /opt/X11/bin/xhost ]; then
    /opt/X11/bin/xhost + 127.0.0.1 > /dev/null 2>&1 || true
fi

export DISPLAY=host.docker.internal:0

echo "Запускаем контейнер ${CONTAINER_NAME}..."
docker run -it --rm \
  --name ${CONTAINER_NAME} \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/home/rosdev/ros2_ws/src \
  -v ros2_ws_build:/home/rosdev/ros2_ws/build \
  -v ros2_ws_install:/home/rosdev/ros2_ws/install \
  -v ros2_ws_log:/home/rosdev/ros2_ws/log \
  ${IMAGE_NAME} \
  bash