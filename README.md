# ocs2_ros2_docker
Package the project https://github.com/legubiao/ocs2_ros2 into Docker

## Build image

``` bash
git clone https://github.com/AndrewDiyi/ocs2_ros2_docker.git

cd ocs2_ros2_docker

docker build -t ocs2-ros2 --target ocs2 -f Dockerfile .
```

## Run container

```bash
./run_docker.sh
```
