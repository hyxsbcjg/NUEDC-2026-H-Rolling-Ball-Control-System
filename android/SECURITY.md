# Security scope

RTSP is disabled by default in [`../k230/main.py`](../k230/main.py)
(`ENABLE_RTSP = False`). The repository contains no usable hotspot password:
`AP_SSID` is the `CHANGE_ME` placeholder, and `AP_PASSWORD` is `None`.

If the user enables RTSP and configures a private SSID and strong password, the
K230 creates a local hotspot and serves an unauthenticated, unencrypted RTSP
stream. Risks include sharing the hotspot password, exposing the stream without
authentication or encryption, and allowing other devices on the local network
to connect.

The WPA2 hotspot protects association to the local Wi-Fi network, but it does
not add end-to-end authentication or encryption to RTSP. The Android manifest
allows cleartext traffic because the configured `rtsp://` stream is plaintext.

`WIFI_SSID_HINT` is display text only. The Android app does not authenticate or
verify the SSID; it accepts a Wi-Fi network when the configured K230 host and
RTSP port are reachable, then binds the app process to that network. A device
that can impersonate that endpoint may therefore supply the stream.

Use the current baseline only in a controlled test area. Before deployment:

1. Set a private `AP_SSID` and unique `AP_PASSWORD` in
   [`../k230/main.py`](../k230/main.py), then enable RTSP.
2. Keep the hotspot password private and rotate it if it is shared.
3. Restrict physical radio range or isolate the test network.
4. Add an authenticated gateway if unauthorized viewing matters.
5. Treat Debug Preview APKs as test artifacts, not production-signed releases.
6. Rebuild and repeat the complete delayed-client, reconnect, and denial-of-
   service tests.

The Android app never stores the hotspot password. The user connects through
Android system Wi-Fi settings.
