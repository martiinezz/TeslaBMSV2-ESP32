from flask import Flask, jsonify, send_from_directory, request
import random
import json
import time

app = Flask(__name__)

@app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
    response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
    return response

@app.route('/')
def index():
    return send_from_directory('data', 'bms.html')

@app.route('/api/data')
def api_data():
    time.sleep(6)  # Simulate slower response
    # Generate random BMS data
    modules = random.randint(5, 8)
    voltage = round(random.uniform(300, 400), 2)
    soc = random.randint(10, 90)
    current = round(random.uniform(-50, 50), 2)
    avgCellVolt = round(random.uniform(3.5, 4.2), 3)
    lowCellVolt = round(avgCellVolt - random.uniform(0, 0.5), 3)
    highCellVolt = round(avgCellVolt + random.uniform(0, 0.5), 3)
    delta = round((highCellVolt - lowCellVolt) * 1000, 1)
    avgTemp = round(random.uniform(20, 40), 1)

    moduleData = []
    for i in range(1, modules + 1):
        mod_volt = round(random.uniform(10, 15), 2)
        cells = [round(random.uniform(3.0, 4.2), 3) for _ in range(6)]
        temp1 = round(random.uniform(15, 45), 1)
        temp2 = round(random.uniform(15, 45), 1)
        moduleData.append({
            "id": i,
            "voltage": mod_volt,
            "cells": cells,
            "temp1": temp1,
            "temp2": temp2
        })

    return jsonify({
        "modules": modules,
        "voltage": voltage,
        "soc": soc,
        "current": current,
        "avgCellVolt": avgCellVolt,
        "lowCellVolt": lowCellVolt,
        "highCellVolt": highCellVolt,
        "delta": delta,
        "avgTemp": avgTemp,
        "moduleData": moduleData
    })

@app.route('/api/settings', methods=['GET'])
def get_settings():
    # Mock settings
    settings = {
        "OverVSetpoint": 4.2,
        "UnderVSetpoint": 3.0,
        "ChargeVsetpoint": 4.1,
        "DischVsetpoint": 3.2,
        "balanceVoltage": 3.9,
        "OverTSetpoint": 65.0,
        "UnderTSetpoint": -10.0,
        "CAP": 100,
        "Pstrings": 1,
        "Scells": 12
    }
    return jsonify(settings)

@app.route('/api/settings', methods=['POST'])
def post_settings():
    # In real implementation, update settings
    # For now, just return success
    return jsonify({"status": "success"})

@app.route('/api/logs')
def api_logs():
    return "BMS logs: System running\n"

@app.route('/api/ota/url', methods=['POST'])
def ota_url():
    data = request.get_json()
    url = data.get('url')
    # Mock OTA from URL
    return jsonify({"message": f"OTA update started from URL: {url}"})

@app.route('/api/ota/upload', methods=['POST'])
def ota_upload():
    if 'firmware' not in request.files:
        return jsonify({"message": "No file uploaded"}), 400
    file = request.files['firmware']
    if file.filename == '':
        return jsonify({"message": "No file selected"}), 400
    # Mock processing the file
    return jsonify({"message": f"Firmware {file.filename} uploaded and update started"})

@app.route('/api/wifi/scan')
def wifi_scan():
    # Mock WiFi networks
    networks = [
        {"ssid": "HomeWiFi", "rssi": -50},
        {"ssid": "OfficeNet", "rssi": -60},
        {"ssid": "Guest", "rssi": -70},
        {"ssid": "PublicWiFi", "rssi": -80}
    ]
    return jsonify(networks)

@app.route('/api/wifi/connect', methods=['POST'])
def wifi_connect():
    data = request.get_json()
    ssid = data.get('ssid')
    password = data.get('password')
    # Mock connection
    return jsonify({"message": f"Connected to {ssid}"})

@app.route('/api/wifi/status')
def wifi_status():
    # Mock status
    return jsonify({"connected": True, "ssid": "HomeWiFi"})

@app.route('/ccs/<path:filename>')
def css_files(filename):
    return send_from_directory('data/ccs', filename)

@app.route('/favicon.png')
def favicon():
    return send_from_directory('data', 'favicon.png')

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
