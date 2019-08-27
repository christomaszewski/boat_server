#!/usr/bin/env python3.7

import asyncio
import serial_asyncio
import signal
import json
import rospy

from functools import partial
from std_msgs.msg import String
from boat_server import msg

async def main():
	outgoing_msgs = asyncio.Queue()
	incoming_msgs = asyncio.Queue()

	rospy.init_node('serial_comms', anonymous=True)
	rospy.Subscriber('motor_cmd', msg.motor_cmd, partial(motor_cmd_callback, outgoing_msgs))
	rospy.Subscriber('eboard_cmd', msg.eboard_cmd, partial(eboard_cmd_callback, outgoing_msgs))

	reader, writer = await serial_asyncio.open_serial_connection(url='/dev/ttyACM0', baudrate=115200)
	await asyncio.sleep(3)
	send = asyncio.create_task(send_serial(writer, outgoing_msgs))
	recv = asyncio.create_task(recv_serial(reader, incoming_msgs))
	pub = asyncio.create_task(publisher(incoming_msgs))
	await asyncio.gather(send, recv, pub)

async def send_serial(w, msgs):
	while True:
		outgoing_msg = await msgs.get()
		w.write(outgoing_msg.encode())
		msgs.task_done()

async def recv_serial(r, msgs):
	while True:
		incoming_msg = await r.readuntil(b'\n')
		try:
			rospy.loginfo(f'received: {incoming_msg.rstrip().decode()}')
			msgs.put_nowait(incoming_msg.rstrip().decode())
		except:
			rospy.loginfo('An error occurred while decoding message')

async def publisher(msgs):
	pub = rospy.Publisher('serial_in', String, queue_size=10)
	imu_pub = rospy.Publisher('imu', msg.imu, queue_size=10)
	gps_pub = rospy.Publisher('gps', msg.gps, queue_size=10)

	while not rospy.is_shutdown():
		curr_msg = await msgs.get()
		parsed_msg = dict()
		parsed_msg['type'] = None

		try:
			parsed_msg = json.loads(curr_msg)
		except:
			rospy.loginfo(f"An error ocurred while parsing json message: {msg}")
			parsed_msg['type'] = 'error'

		if 'type' not in parsed_msg:
			rospy.loginfo("Should not enter this if anymore!!!")
			pub.publish(curr_msg)

		elif parsed_msg['type'] == 'imu':
			data_array = parsed_msg['data'].split(',')[:7]
			pub_msg = msg.imu(*data_array)
			
			"""
			pub_msg.x = float(data_array[0])
			pub_msg.y = float(data_array[1])
			pub_msg.z = float(data_array[2])
			pub_msg.sys_calib = int(data_array[3])
			pub_msg.gyro_calib = int(data_array[4])
			pub_msg.mag_calib = int(data_array[5])
			pub_msg.accel_calib = int(data_array[6])
			"""

			imu_pub.publish(pub_msg)
		elif parsed_msg['type'] == 'gps':
			data_array = parsed_msg['data'].split(',')
			if data_array[0] == '$GPRMC' and data_array[2] == 'A':
				# Message type is GPRMC and GPS has a fix
				lat_str = data_array[3]
				lat = float(lat_str[:2]) + float(lat_str[2:])/60.
				lat *= -1. if data_array[4] == 'S' else 1.
				lon_str = data_array[5]
				lon = float(lon_str[:3]) + float(lon_str[3:])/60.
				lon *= -1. if data_array[6] == 'W' else 1.
				pub_msg = msg.gps(lat=lat, long=lon)

				gps_pub.publish(pub_msg)
		else:
			pub.publish(curr_msg)

		msgs.task_done()

def motor_cmd_callback(msg_queue, motor_cmd):
	eol = "\n"
	# Convert message to string
	msg_string = f'{{"m0":{{"v":{motor_cmd.m0}}},"m1":{{"v":{motor_cmd.m1}}}}}{eol}'
	# Push coverted message string to output queue
	msg_queue.put_nowait(msg_string)
	rospy.loginfo(f"Pushing formatted motor message to output queue: {msg_string}")

def eboard_cmd_callback(msg_queue, eboard_cmd):
	eol = "\n"
	msg_string = f'{{"e":{{"cmd":"{eboard_cmd.cmd}"}}}}{eol}'
	msg_queue.put_nowait(msg_string)
	rospy.loginfo(f"Pushing formatted eboard message to output queue: {msg_string}")

def sigint_handler(signum, frame):
	for task in asyncio.Task.all_tasks():
		task.cancel()

if __name__=='__main__':
	signal.signal(signal.SIGINT, sigint_handler)
	asyncio.run(main())