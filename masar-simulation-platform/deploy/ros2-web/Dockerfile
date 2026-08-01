FROM ros:humble-ros-base-jammy@sha256:afb40d6be65331c20a114d4e229a7ef099fed1b17bf6370daee193514b32aa16

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 \
        supervisor \
        ros-humble-rosbridge-suite \
        ros-humble-tf2-web-republisher \
        ros-humble-web-video-server \
    && rm -rf /var/lib/apt/lists/*

COPY deploy/sidecars/ros2-supervisord.conf /etc/supervisor/supervisord.conf
COPY deploy/sidecars/run-ros2-sidecar /usr/local/bin/run-ros2-sidecar
COPY deploy/sidecars/sidecar-health /usr/local/bin/sidecar-health
RUN chmod 0755 /usr/local/bin/run-ros2-sidecar /usr/local/bin/sidecar-health

HEALTHCHECK --interval=10s --timeout=4s --start-period=20s --retries=6 \
  CMD ["/usr/local/bin/sidecar-health", "18112", "18122"]

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
