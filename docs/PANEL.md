otwierasz 2 powershelle 

w jednym: \

ssh robot@172.20.10.4


cd ~/hackaton && source .venv/bin/activate
ROBOT_DRIVE_PORT=/dev/robot-drive ROBOT_ARM_PORT=/dev/robot-arm python web_control.py



w drugim odpalasz : 


ssh robot@172.20.10.4


cd ~/hackaton && source .venv/bin/activate
python tools/arm_web.py --port /dev/robot-arm --no-home
