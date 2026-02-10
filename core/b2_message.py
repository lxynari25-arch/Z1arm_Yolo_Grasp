import asyncio
import websockets
import json
import multiprocessing
from multiprocessing import Queue


class RobotDogListener:
    def __init__(self, ws_uri="ws://192.168.123.162:18083"):
        self.ws_uri = ws_uri
        self.queue = None  # 只需要传队列进来

    async def listen_broadcast(self):
        print("连接 B2 广播：", self.ws_uri)
        async with websockets.connect(self.ws_uri) as ws:
            print("✅ 连接成功，正在接收广播...")
            while True:
                msg = await ws.recv()
                print("📡 广播数据:", msg)

                try:
                    data = json.loads(msg)
                    if data.get("data", {}).get("is_arrived") is True:
                        # 子进程只干一件事：往队列塞一个消息
                        self.queue.put("arrived")
                except Exception:
                    pass

    def start(self, queue):
        self.queue = queue
        asyncio.run(self.listen_broadcast())