"""
假实机 bridge。

用途：
- 模拟真实电机 bridge 的 HTTP 接收端。
- 接收角度命令并打印。
- 不打开串口，不控制电机。

这个脚本用于测试“发送流程”：
  stream 脚本 -> HTTP POST -> mock bridge

注意：
如果当前接口仍是 /set_angles 且字段是 theta_hip/theta1/motor2，
这是旧字段名。后续建议统一改成：
  /set_motor_commands
  hip_motor / thigh_motor / calf_motor

测试：
终端 1：
cd ~/Dev/Projects/MPC/real_go2_control
conda activate go2-convex-mpc
python scripts/28_mock_real_leg_bridge.py
终端 2：
cd ~/Dev/Projects/MPC/real_go2_control
conda activate go2-convex-mpc
python scripts/29_stream_real_leg_commands.py --mode http --rate 20 --duration 2.0
如果正常，终端 1 会收到：
hip_motor=...
thigh_motor=...
calf_motor=...

"""
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer


STATE = {
    "ok": True,
    "updated_at": time.time(),
    "target": {
        "hip_motor": 0.0,
        "thigh_motor": 0.0,
        "calf_motor": 0.0,
    },
}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def send_json(self, data, code=200):
        body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/status":
            self.send_json(STATE)
            return
        self.send_json({"ok": False, "error": "not found"}, 404)

    def do_POST(self):
        if self.path != "/set_motor_commands":
            self.send_json({"ok": False, "error": "not found"}, 404)
            return

        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length))

            hip_motor = float(body["hip_motor"])
            thigh_motor = float(body["thigh_motor"])
            calf_motor = float(body["calf_motor"])
        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, 400)
            return

        STATE["updated_at"] = time.time()
        STATE["target"] = {
            "hip_motor": hip_motor,
            "thigh_motor": thigh_motor,
            "calf_motor": calf_motor,
        }

        print(
            f"recv "
            f"hip_motor={hip_motor:7.2f}deg "
            f"thigh_motor={thigh_motor:7.2f}deg "
            f"calf_motor={calf_motor:7.2f}deg"
        )

        self.send_json({"ok": True, "target": STATE["target"]})


def main():
    server = HTTPServer(("127.0.0.1", 8765), Handler)
    print("mock real-leg bridge")
    print("GET  http://127.0.0.1:8765/status")
    print("POST http://127.0.0.1:8765/set_motor_commands")
    print("Ctrl+C stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
