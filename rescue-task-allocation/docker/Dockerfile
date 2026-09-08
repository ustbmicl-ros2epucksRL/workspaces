FROM pytorch/pytorch:2.4.1-cuda12.1-cudnn9-runtime

ARG UID=1006
ARG GID=1006

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONPATH=/root/workspace/rescue-task-allocation:/root/workspace/rescue-task-allocation/chapter4

RUN if ! getent group "${GID}" >/dev/null; then groupadd --gid "${GID}" research; fi \
    && if ! id research >/dev/null 2>&1; then useradd --uid "${UID}" --gid "${GID}" --create-home --shell /bin/bash research; fi

# The research user starts in /root/workspace to match the existing MASAR
# layout. Make only the image's /root traversal possible; host data remains
# protected by the read-only parent bind mount below.
RUN chmod 755 /root

WORKDIR /root/workspace/rescue-task-allocation

COPY docker/requirements.lock /tmp/rescue-task-allocation-requirements.lock
RUN python -m pip install --no-cache-dir -r /tmp/rescue-task-allocation-requirements.lock

USER research
CMD ["bash"]
