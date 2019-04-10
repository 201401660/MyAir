import sys, os
import Adafruit_DHT
import datetime, time
from multiprocessing import Process
from picamera import PiCamera
import RPi.GPIO as GPIO
import serial
import cv2
import csv

def PiCam():
	camera = PiCamera()
	cap0 = cv2.VideoCapture(0)
	cap1 = cv2.VideoCapture(1)

	while True:
		now = datetime.datetime.now()
		nowDate = now.strftime('%Y-%m-%d')
		nowTime = now.strftime('%H%M%S')
		nowPath = ('/home/pi/FDMD/Camera/' + nowDate)

		try:
			if not(os.path.isdir(nowPath)):
				os.mkdir(nowPath)
		except:
			pass

		camera.capture(nowPath+'/'+nowTime +'_F.jpg')

		ret0, img0 = cap0.read()
		ret1, img1 = cap1.read()

		cv2.imwrite(nowPath+'/'+nowTime+'_R.jpg',img0)
		cv2.imwrite(nowPath+'/'+nowTime+'_L.jpg',img1)

		time.sleep(0.9)

def DustSensor():
	dust_port = '/dev/ttyACM0'
	dust_baud = 9600
	ser = serial.Serial(dust_port, dust_baud)

	while True:
		now = datetime.datetime.now()
		nowTxt = now.strftime('%Y-%m-%d %Hh')
		nowDateTime = now.strftime('%Y-%m-%d %H:%M:%S')

		PMData = ser.readline()[:-2].split(" ")

		PM_list = [str(nowDateTime),'PM1.0',PMData[0],'PM2.5',PMData[1],'PM10',PMData[2]]

		f = open('/home/pi/FDMD/DustSensor/' + nowTxt + '.csv','a')
		wr = csv.writer(f)
		wr.writerow(PM_list)
		f.close

		time.sleep(1.8)

def HTSensor():
	HT_sensor = 22
	HT_pin = 24 # pin 18

	while True:
		now = datetime.datetime.now()
		nowTxt = now.strftime('%Y-%m-%d %Hh')
		nowDateTime = now.strftime('%Y-%m-%d %H:%M:%S')

		humidity, temperature = Adafruit_DHT.read_retry(HT_sensor, HT_pin)
		Temp = float("{0:.2f}".format(temperature))
		Humid = float("{0:.2f}".format(humidity))

		HT_list = [str(nowDateTime),'temp',str(Temp),'Humid',str(Humid)]

		f = open('/home/pi/FDMD/HTSensor/' + nowTxt + '.csv','a')
		wr = csv.writer(f)
		wr.writerow(HT_list)
		f.close

		time.sleep(10)

def End_process():
	sw = 25 # pin 22
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(sw, GPIO.IN, pull_up_down = GPIO.PUD_UP)
	GPIO.add_event_detect(sw, GPIO.FALLING, bouncetime = 500)

	while True:
		try:
			if GPIO.event_detected(sw):
				p1.terminate()
				p2.terminate()
				p3.terminate()

				time.sleep(3)
				#os.system("shutdown now")
		except :
			pass

if __name__ == '__main__':
	p1 = Process(target=PiCam)
	p2 = Process(target=DustSensor)
	p3 = Process(target=HTSensor)
	p4 = Process(target=End_process)

	p1.start()
	p2.start()
	p3.start()
	p4.start()
	
	p1.join()
	p2.join()
	p3.join()
	p4.join()
