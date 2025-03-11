from flask import Flask, request, jsonify
import json

app = Flask(__name__)

# 存储 JSON 数据
json_file_path = "/home/jetson/code/data.json"
flag_file_path = "/home/jetson/code/flag.log"


# 初始化 JSON 文件
default_data = {"Status": "Awaiting"}
try:
    with open(json_file_path, "r") as f:
        data = json.load(f)
except (FileNotFoundError, json.JSONDecodeError):
    with open(json_file_path, "w") as f:
        json.dump(default_data, f)

# 读取 JSON 文件
def read_json():
    with open(json_file_path, "r") as f:
        return json.load(f)

# 写入 JSON 文件
def write_json(new_data):
    with open(json_file_path, "w") as f:
        json.dump(new_data, f, indent=4)

# 访问 JSON 数据
@app.route("/json", methods=["GET"])
def get_json():
    flag = f"访问时间: {request.remote_addr}\n"
    with open(flag_file_path, "a") as f:
        f.write(flag)
    return jsonify(read_json())

# 修改 JSON 数据
@app.route("/json", methods=["POST"])
def update_json():
    try:
        new_data = request.json
        write_json(new_data)
        return jsonify({"status": "success", "message": "JSON 已更新"}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)

