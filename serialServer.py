import serial
from time import sleep

serieServ = serial.Serial(port="COM11", baudrate=115200)
serieClient = serial.Serial(port="COM4", baudrate=115200)

while True:    
	sleep(0.1)
	mesure = serieServ.readline().strip()
	str = mesure.decode("utf-8")
	
	if "u" in str or "d" in str or "s" in str or "t" in str:
		serieClient.flush()
		serieClient.write(str.encode())
		