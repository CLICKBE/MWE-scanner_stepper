import csv
from time import sleep

import serial

# TODO : pass port and baudrate as argument with init values
dataSerie = serial.Serial(port="COM4", baudrate=115200)

listG = []
nb_of_scan = 0

## Double while True loops need to be tested
while True :
    data = input()
    #print( data )
    dataSerie.write(str.encode(data))
	
    if( data == 's' or data == 'y' ) :
        nb_of_scan += 1
        dimension = ''
		suffix = ''
        if( nb_of_scan == 1 ):
            suffix = '-background'
        elif( nb_of_scan == 2 ):
            suffix = '-with_objects'
        if( data == 's' ):
            dimension = '2D'
        elif( data =='y' ):
            dimension == '1D'
        filename = "stepperscanner-" + dimension + suffix + ".csv"

        while True:
            # sleep(0.1)

            mesure = dataSerie.readline().strip()
            mesureUtf = mesure.decode("utf-8")
            mesure1 = mesureUtf.split()

            print(mesure1)

            if mesureUtf == "stop":

                with open( filename, 'w' ) as f :
                    writer = csv.writer( f )
                    print( len( listG ) )
                    writer.writerows( listG )
                    break
            else: ## TODO : add all elements not just xYZ and then during parsing put them in the right dict keys
                #listG.append( [ mesure1[ 6 ], mesure1[ 7 ], mesure1[ 8 ] + ";"] )
                listG.append( mesureUtf )
    #else :
    #    while True:
    #        bytesToRead = dataSerie.inWaiting()
    #        print( dataSerie.read( bytesToRead ) )


