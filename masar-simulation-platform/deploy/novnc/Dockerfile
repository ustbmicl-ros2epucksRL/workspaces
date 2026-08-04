FROM ubuntu@sha256:0e0a0fc6d18feda9db1590da249ac93e8d5abfea8f4c3c0c849ce512b5ef8982

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates=20260601~22.04.1 \
        curl=7.81.0-1ubuntu1.25 \
        openbox=3.6.1-10 \
        python3=3.10.6-1~22.04.1 \
        python3-jwcrypto=1.0.0-2 \
        python3-numpy=1:1.21.5-1ubuntu22.04.1 \
        python3-pip=22.0.2+dfsg-1ubuntu0.7 \
        python3-redis=3.5.3-2 \
        python3-requests=2.25.1+dfsg-2ubuntu0.3 \
        supervisor=4.2.1-1ubuntu1 \
        x11vnc=0.9.16-8 \
        xvfb=2:21.1.4-2ubuntu1.7~22.04.16 \
    && rm -rf /var/lib/apt/lists/*

RUN curl --http1.1 --retry 5 --retry-all-errors --connect-timeout 10 --max-time 120 -fL \
        https://codeload.github.com/novnc/noVNC/tar.gz/63107bd06d9e1f6136ff21aeda8cd62cbf0d433e \
        -o /tmp/novnc.tar.gz \
    && echo "06dac239be3973ed79a00e357a68aab996e6ed2fd496ee2cc6e6a34020344ac9  /tmp/novnc.tar.gz" \
        | sha256sum -c - \
    && mkdir /opt/noVNC \
    && tar -xzf /tmp/novnc.tar.gz --strip-components=1 -C /opt/noVNC \
    && curl --http1.1 --retry 5 --retry-all-errors --connect-timeout 10 --max-time 120 -fL \
        https://codeload.github.com/novnc/websockify/tar.gz/e81894751365afc19fe64fc9d0e5c6fc52655c36 \
        -o /tmp/websockify.tar.gz \
    && echo "4c7a9d88e9de1e9274f8f0f4ebae8d7dd2ffc61ceb6eac63e3e6e3ef0f711d0d  /tmp/websockify.tar.gz" \
        | sha256sum -c - \
    && mkdir /tmp/websockify \
    && tar -xzf /tmp/websockify.tar.gz --strip-components=1 -C /tmp/websockify \
    && python3 -m pip install --no-cache-dir --no-deps /tmp/websockify \
    && rm -rf /tmp/novnc.tar.gz /tmp/websockify /tmp/websockify.tar.gz

COPY deploy/display/display_manager.py /usr/local/bin/display_manager.py
COPY deploy/display/xstartup-gazebo /usr/local/bin/xstartup-gazebo
COPY deploy/display/xstartup-rviz /usr/local/bin/xstartup-rviz
COPY deploy/display/xstartup-webots /usr/local/bin/xstartup-webots
COPY deploy/novnc/supervisord.conf /etc/supervisor/conf.d/masar-display.conf

RUN chmod 0755 \
        /usr/local/bin/display_manager.py \
        /usr/local/bin/xstartup-gazebo \
        /usr/local/bin/xstartup-rviz \
        /usr/local/bin/xstartup-webots

CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/conf.d/masar-display.conf"]
