FROM alpine:3.22@sha256:14358309a308569c32bdc37e2e0e9694be33a9d99e68afb0f5ff33cc1f695dce

ARG MASAR_BUILD_PROXY=http://127.0.0.1:7897

RUN http_proxy="${MASAR_BUILD_PROXY}" \
    https_proxy="${MASAR_BUILD_PROXY}" \
    apk add --no-cache openssh-client-default netcat-openbsd

COPY --chmod=0755 deploy/public-tunnel/run-public-tunnel /usr/local/bin/run-public-tunnel
COPY --chmod=0755 deploy/public-tunnel/public-tunnel-health /usr/local/bin/public-tunnel-health

ENTRYPOINT ["/usr/local/bin/run-public-tunnel"]
