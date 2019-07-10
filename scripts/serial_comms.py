#!/usr/bin/env python3.7

import asyncio
import serial_asyncio
import signal
import json
import rospy

from functools import partial
from std_msgs.msg import String
from boat_server.msg import imu as imu_msg, gps as gps_msg

supported_sensors = ['imu', 'gps', 'battery']

async def main(callback_func):
	outgoing_msgs = asyncio.Queue()
	incoming_msgs = asyncio.Queue()
	
	rospy.init_node('serial_comms', anonymous=True)
	rospy.Subscriber('serial_out', String, partial(subscriber_callback, outgoing_msgs))

	reader, writer = await serial_asyncio.open_serial_connection(url='/dev/ttyACM0', baudrate=115200)
	await asyncio.sleep(3)
	send = asyncio.create_task(send_serial(writer, outgoing_msgs))
	recv = asyncio.create_task(recv_serial(reader, incoming_msgs))
	pub = asyncio.create_task(publisher(incoming_msgs))
	await asyncio.gather(send, recv, pub)

async def send_serial(w, msgs):
	while True:
		msg = await msgs.get()
		w.write(msg.encode())
		msgs.task_done()

async def recv_serial(r, msgs):
	while True:
		msg = await r.readuntil(b'\n')
		try:
			rospy.loginfo(f'received: {msg.rstrip().decode()}')
			msgs.put_nowait(msg.rstrip().decode())
		except:
			rospy.loginfo('an error occurred while decoding message')

async def publisher(msgs):
	pub = rospy.Publisher('serial_in', String, queue_size=10)
	imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)
	gps_pub = rospy.Publisher('gps', gps_msg, queue_size=10)

	while not rospy.is_shutdown():
		msg = await msgs.get()
		parsed_msg = dict()
		try:
			parsed_msg = json.loads(msg)
		except:
			rospy.loginfo('an error ocurred while parsing json message')
			parsed_msg['type'] = 'error'

		if parsed_msg['type'] == 'imu':
			data_array = parsed_msg['data'].split(',')
			pub_msg = imu_msg()
			pub_msg.x = float(data_array[0])
			pub_msg.y = float(data_array[1])
			pub_msg.z = float(data_array[2])
			pub_msg.sys_calib = int(data_array[3])
			pub_msg.gyro_calib = int(data_array[4])
			pub_msg.mag_calib = int(data_array[5])
			pub_msg.accel_calib = int(data_array[6])

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
				pub_msg = gps_msg()
				pub_msg.lat = lat
				pub_msg.long = lon

				gps_pub.publish(pub_msg)

		else:
			pub.publish(msg)

		msgs.task_done()

def subscriber_callback(msgs, msg):
	msgs.put_nowait(msg.data)
	rospy.loginfo(f"forwarding {msg.data} to serial")

def sigint_handler(signum, frame):
	for task in asyncio.Task.all_tasks():
		task.cancel()

if __name__=='__main__':
	signal.signal(signal.SIGINT, sigint_handler)
	asyncio.run(main(subscriber_callback))