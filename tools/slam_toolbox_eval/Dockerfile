# slam_toolbox offline evaluation image
# Minimal: ros:jazzy-ros-base + slam_toolbox + nav2_map_server.
# nav2_map_server is REQUIRED: slam_toolbox's /slam_toolbox/save_map service
# pulls its map_io library at runtime to write the .pgm/.yaml. Without it
# the service silently fails with "Package 'nav2_map_server' not found".
# Build on the Pi (multi-arch ros:jazzy supports linux/arm64 natively).

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
# Force single-threaded Eigen / OpenMP / OpenBLAS — Ceres + slam_toolbox
# spawn worker threads per core that each pre-allocate scratch buffers.
# On a 4-core Pi Zero 2W with 416 MB RAM, this saves tens of MB.
ENV OMP_NUM_THREADS=1 \
    OPENBLAS_NUM_THREADS=1 \
    MKL_NUM_THREADS=1 \
    EIGEN_DONT_PARALLELIZE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-slam-toolbox \
        ros-jazzy-nav2-map-server \
        ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Replay node + slam params + entrypoint
COPY replay_node.py  /eval/replay_node.py
COPY slam_params.yaml /eval/slam_params.yaml
COPY run_eval.sh     /eval/run_eval.sh
RUN chmod +x /eval/run_eval.sh /eval/replay_node.py

# Mount points: input log -> /data/scan.jsonl, output dir -> /out
VOLUME ["/data", "/out"]

WORKDIR /eval
ENTRYPOINT ["/eval/run_eval.sh"]
