FROM alpine@sha256:5291449c3df73caf6ed85e649dec1b9e818b39a5d8c871e97afc13e9cd5e8fa8

RUN apk add --no-cache autossh openssh-client \
    && addgroup -g 1067 relay \
    && adduser -D -H -u 1066 -G relay relay

USER 1066:1067
ENTRYPOINT ["autossh"]
