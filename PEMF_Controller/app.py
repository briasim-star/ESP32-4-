"""
Minimal subscription entitlement backend for MADD PEMF devices.

This answers exactly one question: "does this device currently have an
active subscription?" It's a deliberately small starting point, not a
finished billing system - real payment processing (Stripe is
recommended - see the note in the project chat for why) is NOT wired in
yet. Entitlements are set manually below, so the device side can be
tested end-to-end before building real billing on top of this.

Deploy anywhere that runs Python - a small VPS, Render, Railway,
PythonAnywhere, etc. This needs almost no resources; a subscription
check happens rarely (device-side, roughly once per boot or per WiFi
sync), not continuously.

SECURITY NOTE: the /admin/ endpoints below have NO authentication at
all. That's fine for local testing on your own machine, but DO NOT
deploy this file publicly as-is - anyone who found the URL could grant
themselves a free subscription. Before deploying somewhere reachable
from the internet, add a real API key or password check to both admin
endpoints (a single shared-secret header check is enough to start).
"""

from flask import Flask, request, jsonify
import json
import os

app = Flask(__name__)

ENTITLEMENTS_FILE = "entitlements.json"


def load_entitlements():
    if not os.path.exists(ENTITLEMENTS_FILE):
        return {}
    with open(ENTITLEMENTS_FILE) as f:
        return json.load(f)


def save_entitlements(data):
    with open(ENTITLEMENTS_FILE, "w") as f:
        json.dump(data, f, indent=2)


@app.route("/check-entitlement")
def check_entitlement():
    """This is the one endpoint the device itself actually calls.
    Returns the plain text "true" or "false" - kept intentionally simple
    since the device just needs a yes/no, not a JSON structure to parse."""
    device_id = request.args.get("device_id", "")
    if not device_id:
        return "false", 400
    entitlements = load_entitlements()
    is_entitled = entitlements.get(device_id, {}).get("active", False)
    return "true" if is_entitled else "false"


# --- Manual admin endpoints - for testing before real billing exists ---

@app.route("/admin/grant", methods=["POST"])
def admin_grant():
    """Manually mark a device as subscribed. Once real billing exists,
    this is what a Stripe webhook would call automatically instead of
    a person calling it by hand."""
    device_id = request.form.get("device_id", "")
    if not device_id:
        return jsonify({"error": "device_id required"}), 400
    entitlements = load_entitlements()
    entitlements[device_id] = {"active": True}
    save_entitlements(entitlements)
    return jsonify({"device_id": device_id, "active": True})


@app.route("/admin/revoke", methods=["POST"])
def admin_revoke():
    """Manually mark a device as no longer subscribed (e.g. cancelled,
    payment failed). Once real billing exists, a Stripe webhook calls
    this automatically when a subscription ends."""
    device_id = request.form.get("device_id", "")
    entitlements = load_entitlements()
    if device_id in entitlements:
        entitlements[device_id]["active"] = False
        save_entitlements(entitlements)
    return jsonify({"device_id": device_id, "active": False})


@app.route("/admin/list")
def admin_list():
    """See every device's current status at a glance, for testing."""
    return jsonify(load_entitlements())


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
