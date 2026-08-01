FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

WORKDIR /opt/masar-platform

COPY pyproject.toml ./
COPY backend ./backend
RUN python -m pip install --no-cache-dir .

COPY scenarios ./scenarios
RUN mkdir -p /var/lib/masar-platform/runs && chown -R 65532:65532 /var/lib/masar-platform

USER 65532:65532

CMD ["masar-platform"]
