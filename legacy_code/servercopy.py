import json, time
json_file_path = "/home/jetson/code/data.json"
flag_file_path = "/home/jetson/code/flag.log"
while True:
    time.sleep(10)
    with open(json_file_path, "w") as f:
        json.dump({"Status": "Awaiting"},f)
    time.sleep(10)
    with open(json_file_path, "w") as f:
        json.dump({"Status": "OK"},f)
