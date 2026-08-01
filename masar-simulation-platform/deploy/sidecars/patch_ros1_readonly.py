from __future__ import annotations

from pathlib import Path


ROOT = Path("/opt/ros/noetic/lib/python3/dist-packages/rosbridge_library")
TARGET = ROOT / "rosbridge_protocol.py"
ORIGINAL = (
    "rosbridge_capabilities = [CallService, Advertise, Publish, Subscribe, Defragment, "
    "AdvertiseService, ServiceResponse, UnadvertiseService]"
)
READ_ONLY = "rosbridge_capabilities = [Subscribe, Defragment]"
LISTEN_DEFAULT = "listenWS(factory, context_factory)"
LISTEN_EXPLICIT = "listenWS(factory, context_factory, interface=address)"


def main() -> None:
    source = TARGET.read_text(encoding="utf-8")
    if ORIGINAL not in source:
        raise RuntimeError(f"unexpected rosbridge protocol source: {TARGET}")
    protocol = source.replace(ORIGINAL, READ_ONLY, 1)
    TARGET.write_text(protocol, encoding="utf-8")

    server = Path("/opt/ros/noetic/lib/rosbridge_server/rosbridge_websocket")
    server_source = server.read_text(encoding="utf-8")
    if server_source.count(LISTEN_DEFAULT) != 1:
        raise RuntimeError(f"unexpected rosbridge server source: {server}")
    server.write_text(server_source.replace(LISTEN_DEFAULT, LISTEN_EXPLICIT, 1), encoding="utf-8")


if __name__ == "__main__":
    main()
