@echo off
chcp 65001 > nul
title GR00T 机械臂控制台

echo.
echo  ╔══════════════════════════════════════╗
echo  ║   GR00T 机械臂自然语言控制服务        ║
echo  ╚══════════════════════════════════════╝
echo.
echo  启动后请在浏览器打开: http://127.0.0.1:5001
echo  按 Ctrl+C 停止服务
echo.

set PYTHONPATH=G:\robot\Isaac-GR00T;G:\robot\lerobot\src
set PYTHON=C:\Users\Grape\.conda\envs\lerobot\python.exe

cd /d G:\robot\lerobot

"%PYTHON%" demo\gr00t_lerobot_service.py ^
  --robot.type=so101_follower ^
  --robot.port=COM4 ^
  --robot.id=my_awesome_follower_arm ^
  --robot.max_relative_target=8.0 ^
  --server_host=192.168.31.103 ^
  --server_port=5555 ^
  --service_host=0.0.0.0 ^
  --service_port=5001 ^
  "--robot.cameras={\"exterior\": {\"type\": \"opencv\", \"index_or_path\": 0, \"width\": 640, \"height\": 480, \"fps\": 30}}"

pause
