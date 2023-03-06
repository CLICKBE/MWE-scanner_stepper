import serial

serieServ = serial.Serial(port="COM11", baudrate=115200)
serieClient = serial.Serial(port="COM4", baudrate=115200)

while True:    
	mesure = serieServ.readline().strip()
	str = mesure.decode("utf-8")
	
	if str == "u" or str == "d" or str == "p1" or str == "p2":
		print(type((str.encode())))
		serieClient.write(str.encode())
		serieClient.readline()