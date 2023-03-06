import csv
from time import sleep

import serial

dataSerie = serial.Serial(port="COM12", baudrate=115200)

listG = []

## Double while True loops need to be tested
while True :
    data = input()
    dataSerie.write(str.encode(data))

    while True:
        # sleep(0.1)

        mesure = dataSerie.readline().strip()
        mesureUtf = mesure.decode("utf-8")
        mesure1 = mesureUtf.split()

        print(mesure1)

        if mesureUtf == "stop":
            with open('data.csv', 'w') as f:
                writer = csv.writer(f)
                print(len(listG))
                writer.writerows(listG)
                break
        else: ## TODO : add all elements not just xYZ and then during parsing put them in the right dict keys
            listG.append([mesure1[6], mesure1[7], mesure1[8] + ";"])




