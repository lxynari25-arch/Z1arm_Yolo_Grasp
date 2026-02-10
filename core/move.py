import requests
import json

# 请求地址
url = "http://192.168.123.162:18081/operate"

# 构造请求体（payload）
payload = {
    "api_id": 1105,
    "id": 12312312312,
    "data": {
        "version": 1,
        "topoAddress": "/home/unitree/dist/files/topoSingle/111/data.json",
        "nodeId": 4
    }
}

# 设置请求头（如果服务端需要指定 Content-Type）
headers = {
    "Content-Type": "application/json"
}