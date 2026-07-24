#!/bin/bash
set -e

IMAGE_NAME="ros2_dev"
IMAGE_TAG="latest"

echo "Собираем образ ${IMAGE_NAME}:${IMAGE_TAG}..."
docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .

echo "Образ ${IMAGE_NAME}:${IMAGE_TAG} успешно собран!"