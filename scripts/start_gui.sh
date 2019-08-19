# start_gui.sh

export FLASK_APP=boat_gui.py
export FLASK_DEBUG=1
#export ROS_MASTER_URI=http://192.168.86.150:11311/

flask run --no-reload --host=0.0.0.0