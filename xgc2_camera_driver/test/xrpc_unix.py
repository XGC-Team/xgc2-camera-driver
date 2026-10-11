"""A minimal XRPC http.v1 client over a Unix socket for the process-level tests."""

import http.client
import json
import socket
import uuid


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path, timeout=10.0):
        super().__init__("localhost", timeout=timeout)
        self._path = path

    def connect(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        try:
            sock.connect(self._path)
        except OSError:
            sock.close()
            raise
        self.sock = sock


def call(path, method, target, body=None, instance=None, timeout_ms=8000):
    """One call with the XRPC request id, deadline and optional instance fence."""
    connection = UnixConnection(path, timeout=timeout_ms / 1000.0 + 2.0)
    headers = {"X-Request-ID": uuid.uuid4().hex, "X-Xrpc-Timeout-Ms": str(timeout_ms)}
    if instance:
        headers["X-Xrpc-Instance-ID"] = instance
    payload = None
    if body is not None:
        headers["Content-Type"] = "application/json"
        payload = json.dumps(body)
    try:
        connection.request(method, target, body=payload, headers=headers)
        response = connection.getresponse()
        data = response.read()
        return response.status, dict(response.getheaders()), data
    finally:
        connection.close()
