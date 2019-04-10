import sys, os
import Adafruit_DHT
import datetime, time
from multiprocessing import Process
from picamera import PiCamera
import RPi.GPIO as GPIO
import serial

def Picture():
	camera = PiCamera()

	while True:
		now = datetime.datetime.now()
		nowDate = now.strftime('%Y-%m-%d')
		nowTime = now.strftime('%H:%M:%S')
		nowPath = ('/home/pi/FDMD/Camera/' + nowDate)
		try:
			if not(os.path.isdir(nowPath)):
				os.mkdir(nowPath)
		except:
			pass
		#camera.start_preview()
		#time.sleep(2)
		camera.capture(nowPath+'/'+nowTime +'.jpg', use_video_port=True)

		time.sleep(2.7)

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

		f = open('/home/pi/FDMD/DustSensor/' + nowTxt + '.txt','a')
		f.write(' '.join(PM_list))
		f.write('\n')
		f.close

		time.sleep(1.8)

def HTSensor():
	HT_sensor = 22
	HT_pin = 4

	while True:
		now = datetime.datetime.now()
		nowTxt = now.strftime('%Y-%m-%d %Hh')
		nowDateTime = now.strftime('%Y-%m-%d %H:%M:%S')

		humidity, temperature = Adafruit_DHT.read_retry(HT_sensor, HT_pin)
		Temp = float("{0:.2f}".format(temperature))
		Humid = float("{0:.2f}".format(humidity))

		HT_list = [str(nowDateTime),'temp',str(Temp),'Humid',str(Humid)]

		f = open('/home/pi/FDMD/HTSensor/' + nowTxt + '.txt','a')
		f.write(' '.join(HT_list))
		f.write('\n')
		f.close

		time.sleep(10)

def End_process():
	sw = 17 # pin 11
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
				os.system("shutdown now")
		except :
			pass

if __name__ == '__main__':
	p1 = Process(target=Picture)
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

