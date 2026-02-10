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


# ======================
# 主进程
# ======================
def main():
    # 1. 创建一个队列
    q = Queue()

    # 2. 创建监听实例
    listener = RobotDogListener()

    # 3. 启动子进程，只传队列（超级干净）
    p = multiprocessing.Process(target=listener.start, args=(q,), daemon=True)
    p.start()

    print("🚀 主进程运行中，等待机器狗到达...")

    # 4. 主进程死循环监听队列，所有逻辑都在这里写
    while True:
        if not q.empty():
            msg = q.get()
            if msg == "arrived":
                # =============================================
                # 🔥 所有后续动作 100% 跑在【主进程】
                # =============================================
                print("=====================================")
                print("🎯 主进程收到：机器狗已到达！")
                print("=====================================")

                # 你在这里写任何逻辑都行：
                # 控制硬件、发指令、调用函数、操作全局变量
                # do_something()
        asyncio.run(asyncio.sleep(0.1))


if __name__ == "__main__":
    main()