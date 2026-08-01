FROM ros:noetic-ros-base-focal@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3 \
        supervisor \
        ros-noetic-rosbridge-suite \
        ros-noetic-tf2-web-republisher \
        ros-noetic-web-video-server \
    && rm -rf /var/lib/apt/lists/*

COPY deploy/sidecars/patch_ros1_readonly.py /tmp/patch_ros1_readonly.py
RUN python3 /tmp/patch_ros1_readonly.py && rm /tmp/patch_ros1_readonly.py

COPY deploy/sidecars/ros1-supervisord.conf /etc/supervisor/supervisord.conf
COPY deploy/sidecars/run-ros1-sidecar /usr/local/bin/run-ros1-sidecar
COPY deploy/sidecars/sidecar-health /usr/local/bin/sidecar-health
RUN chmod 0755 /usr/local/bin/run-ros1-sidecar /usr/local/bin/sidecar-health

HEALTHCHECK --interval=10s --timeout=4s --start-period=20s --retries=6 \
  CMD ["/usr/local/bin/sidecar-health", "18111", "18121"]

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
