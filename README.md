# Execute the following in their own terminals:

## create the roscore master
'$roscore'

## start the LIDAR and the serial links to the Arduinos.
'$./bonesterm.sh'  "shell script for ssh into bones (jetson)

### On bones, execute the following:
'cd bones/docker'
'./run.sh'

### Open a browser and connect to the server that just started
'10.0.0.66:8888'

#### Open a terminal inside the browser window
'source bones/catkin_ws/devel/setup.bash'
'roslaunch arduino_jetson ard.launch'

## Open terminals and start the following
'rqt'
'rosrun rviz rviz'

## start the raspberry pi server
'ssh -X pi@10.0.0.5'
'cd share' to access bones.html
'python -m SimpleHTTPServer 8000'

## Create a client of the pi server
'roslaunch rosbridge_server rosbridge_websocket.launch'

## Start Groot
'roscd'
'./lib/groot/Groot'

## Start the following in their own terminals:
'roslaunch bones_move bones.launch
'hector_slam_launch pr2os.launch'
'roslaunch bones_nav move_base_flex.launch'
'roscd && ./lib/bones_bt/bones_bt'

# Control Bones through the webserver

