FROM caddy:2.10.2-alpine@sha256:4c6e91c6ed0e2fa03efd5b44747b625fec79bc9cd06ac5235a779726618e530d

RUN apk add --no-cache iptables=1.8.11-r1

COPY deploy/webots-guard/webots-port-guard /usr/local/bin/webots-port-guard

ENTRYPOINT ["/usr/local/bin/webots-port-guard"]
