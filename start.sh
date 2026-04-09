#!/usr/bin/env bash
set -e

echo ""
echo " ╔══════════════════════════════════════╗"
echo " ║   GR00T 机械臂自然语言控制服务        ║"
echo " ╚══════════════════════════════════════╝"
echo ""
echo " 启动后请在浏览器打开: http://127.0.0.1:5001"
echo " 按 Ctrl+C 停止服务"
echo ""

export PYTHONPATH=/g/robot/Isaac-GR00T:/g/robot/lerobot/src
PYTHON='/c/Users/Grape/.conda/envs/lerobot/python.exe'

cd /g/robot/lerobot

"$PYTHON" demo/gr00t_lerobot_service.py \
  --robot.type=so101_follower \
  --robot.port=COM4 \
  --robot.id=my_awesome_follower_arm \
  --robot.max_relative_target=8.0 \
  --server_host=192.168.31.103 \
  --server_port=5555 \
  --service_host=0.0.0.0 \
  --service_port=5001 \
  '--robot.cameras={"exterior": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30}}'
