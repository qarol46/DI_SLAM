#!/bin/bash
CONTAINER_NAME="ros2_container"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Контейнер ${CONTAINER_NAME} не запущен. Сначала выполните ./run.sh"
    exit 1
fi

echo "Подключаемся к контейнеру ${CONTAINER_NAME}..."
docker exec -it ${CONTAINER_NAME} bash