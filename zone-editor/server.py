import os
import json
from flask import Flask, request, jsonify, render_template

app = Flask(__name__)

DATA_FILE = "/data/map.json"

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/api/map", methods=["GET"])
def get_map():
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE, "r") as f:
            return jsonify(json.load(f))
    return jsonify({"type": "FeatureCollection", "features": []})

@app.route("/api/map", methods=["POST"])
def save_map():
    data = request.get_json()
    with open(DATA_FILE, "w") as f:
        json.dump(data, f, indent=2)
    return jsonify({"status": "ok"})

@app.route("/api/datum", methods=["GET"])
def get_datum():
    lat = float(os.environ.get("OM_DATUM_LAT", 30.0))
    lng = float(os.environ.get("OM_DATUM_LONG", 0.5))
    return jsonify({"lat": lat, "lng": lng})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)
