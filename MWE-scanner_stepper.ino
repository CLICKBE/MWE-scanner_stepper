
#include <SoftwareSerial.h>   //header file of software serial port
//#include "TFMini.h"
#include "TFMPlus.h"
//TFMini tfmini;
TFMPlus tfmini;
SoftwareSerial SerialTFMini(10, 11);  //define software serial port name as Serial1 and define pin2 as RX & pin3 as TX

int dist;                     //actual distance measurements of LiDAR
int strength;                 //signal strength of LiDAR
int check;                    //save check value
int i;
int uart[9];                   //save data measured by LiDAR
const int HEADER = 0x59;      //frame header of data package
int posx=0;
int posy=0;
int posz=0;
byte inbyte;

int ticks_per_revolution = 200;
float subdivision = 1. / 8.;
float total_nb_of_steps = ticks_per_revolution / subdivision;
float degree_per_step = 360 / total_nb_of_steps; 

int vertical_nb_of_steps = total_nb_of_steps / 5;// Vertical rotation of 90°
int horizontal_nb_of_steps = total_nb_of_steps / 12; // Horizontal rotation of 30° - factor of 9 for 40°
int vertical_offset = 2;
int horizontal_offset = 10;

void setup()
{
  //motor X
  pinMode(2, OUTPUT);
  digitalWrite(2,LOW);
  
  pinMode(5, OUTPUT);
  digitalWrite(5,LOW);

  //motor Y  
  pinMode(3, OUTPUT);
  digitalWrite(3,LOW);
  
  pinMode(6, OUTPUT);
  digitalWrite(6,LOW);
  //motor Z
  pinMode(4, OUTPUT);
  digitalWrite(4,LOW);
  
  pinMode(7, OUTPUT);
  digitalWrite(7,LOW);
  
  //motor enable
  pinMode(8, OUTPUT);
  digitalWrite(8,LOW);
  
  Serial.begin(115200);         //set bit rate of serial port connecting Arduino with computer

  SerialTFMini.begin(115200);    //Initialize the data rate for the SoftwareSerial port
  tfmini.begin(&SerialTFMini);            //Initialize the TF Mini sensor}

  // For now the steppers are manually zeroed. The zero pointface the ground (vertically) and is paralleled to the back of the structure (horizontal)
  // The total nb of horizontal steps is share out around this manually set horizontal zero 
  

  xmove( -horizontal_nb_of_steps / 2 );
}

int16_t distance = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
float x = -1., y = -1, z = -1;
bool scan = true;

void loop() {
  while(Serial.available()==0){}
  inbyte=Serial.read();
  //Serial.println( "Command " + String( inbyte ) );
  switch (inbyte){
  
  case 117: //u
    zmove(10);
    posz=posz+10;
    //Serial.print(posx);
    //Serial.write(9);
    //Serial.println(posy);
  break;
  
  case 100: //d
    zmove(-10);
    posz=posz-10;
    //Serial.print(posx);
    //Serial.write(9);
    //Serial.println(posy);
  break;
  
  case 112: //p to allow scan reboot
    scan = true;
    break;

  case 115: //s for scan
    
    while(Serial.available()==0){
      //Serial.println( "scanning" );
      int distance = 0;
      int strength = 0;
      // For use of TFMini library
      // getTFminiData(&distance, &strength);
      
      // For use of TFMPlus library
      tfmini.getData( distance, tfFlux, tfTemp);

      while ( !distance && scan )
      {
        //Serial.println( "No distance" );
        // For use of TFMini library
        // getTFminiData(&distance, &strength);
        // For use of TFMPlus library
        tfmini.getData( distance, tfFlux, tfTemp );
        if (distance){
          //Serial.println( distance );
          // ymove( vertical_offset );
          ymove( vertical_offset );
          posy = posy + vertical_offset;
          computeXYZ( &x, &y, &z, distance, posx * degree_per_step, posy * degree_per_step );
          delay(10);
          Serial.print(posx);
          Serial.write(9);
          Serial.print(posy);
          Serial.write(9);
          Serial.print(posz);
          Serial.write(9);
          Serial.print( posx * degree_per_step );
          Serial.write( 9 );
          Serial.print( posy * degree_per_step );
          Serial.write( 9 );
          Serial.print(distance);
          Serial.write(9);
          Serial.print( x );
          Serial.write( 9 );
          Serial.print( y );
          Serial.write( 9 );
          Serial.println( z );
          if ( posy >= vertical_nb_of_steps ) { ymove( -vertical_nb_of_steps ); posy = 0; xmove( horizontal_offset ); posx = posx + horizontal_offset;} // déplacement vertical ()
          if ( posx >= horizontal_nb_of_steps ) { xmove( -horizontal_nb_of_steps ); posx = 0; scan = false; Serial.println( "stop" ); break;} // 
          //Serial.print("cm\t");
          //Serial.print("strength: ");
          //Serial.println(strength);
        }
      }
    }
  Serial.println( "stop" );
  break;

  case 105 : //i pour info
    displayInfo();
    
    break;

  case 118 : // v to set vertical offset
    inbyte = Serial.read();
    vertical_offset = Serial.parseInt();
    Serial.println( "---- Modifying vertical offset, new value : " + String( vertical_offset ) );
    //displayInfo();
    break;

  case 104 : // h to set horizontal offset
    inbyte = Serial.read();
    horizontal_offset = Serial.parseInt();
    Serial.println( "---- Modifying horizontal offset, new value : " + String( horizontal_offset ) );
    
    break;
  }
}

void zmove(long steps){
  if (steps<0){
    digitalWrite(7,HIGH);
  }
  else{
    digitalWrite(7,LOW);
  }
  
  for(long i=0;i<abs(steps);i++){
    digitalWrite(4,HIGH);
    delay(7);
    digitalWrite(4,LOW);
    delay(7);
  } 
}


void ymove(long steps){
  if (steps<0){
    digitalWrite(6,HIGH);
  }
  else{
    digitalWrite(6,LOW);
  }
  
  for(long i=0;i<abs(steps);i++){
    digitalWrite(3,HIGH);
    delay(5);
    digitalWrite(3,LOW);
    delay(5);
  } 
}

void xmove(long steps){
  if (steps<0){
    digitalWrite(5,HIGH);
  }
  else{
    digitalWrite(5,LOW);
  }
  
  for(long i=0;i<abs(steps);i++){
    digitalWrite(2,HIGH);
    delay(5);
    digitalWrite(2,LOW);
    delay(5);
  } 
}

void displayInfo()
{
  Serial.println( "vertical_nb_of_steps " + String( vertical_nb_of_steps ) );
  Serial.println( "horizontal_nb_of_steps " + String( horizontal_nb_of_steps ) );
  Serial.println( "ticks_per_revolution " + String( ticks_per_revolution ) );
  Serial.println( "subdivision " + String( subdivision ) );
  Serial.println( "total_nb_of_steps " + String( total_nb_of_steps ) );
  Serial.println( "degree_per_step " + String( degree_per_step ) ); 
  Serial.println( "Vertical offset (in steps) : " + String( vertical_offset ) );
  Serial.println( "Horizontal offset (in steps) : " + String( horizontal_offset ) );
  Serial.println( "Vertical precision (in degree) : " + String( degree_per_step * vertical_offset ) );
  Serial.println( "Horizontal precision (in degree) : " + String( degree_per_step * horizontal_offset ) );
}

void computeXYZ( float *x, float *y, float *z, int16_t distance, int hAngleDegree, int vAngleDegree ) {
/***
 * Convert horiz_step_idx, vert_step_idx and distance to (x, y, z) coordinates.
 * Computation comes from : https://github.com/bitluni/3DScannerESP8266
 * And https://www.youtube.com/watch?v=vwUGPjQ_5t4
 * Arguments : 
 *  *x (float) : the float pointer to the x coordinates
 *  *y (float) : the float pointer to the y coordinates
 *  *z (float) : the float pointer to the z coordinates
 *  distance (int16_t) : distance retrieved by LiDAR sensor
 *  int hAngleDegree : horizontal angle in degree of the pantilt sweeping
 *  int vAngleDegree : vertical angle in degree of the pantilt sweeping
 * Return :
 *  void
 */ 

//  float yawf = hAngleDegree * M_PI / 180;
//  float pitchf = vAngleDegree * M_PI / 180;

  float pitchf = hAngleDegree * M_PI / 180;
  float yawf = vAngleDegree * M_PI / 180;


//  float yawf   = hAngleDegree;
//  float pitchf = vAngleDegree;

  //Serial.println(yawf);
  //Serial.println(pitchf);

  // *x = -sin( yawf ) * distance * cos( pitchf );
  // *y = cos( yawf ) * distance * cos( pitchf );
  // *z = distance * sin( pitchf );

  *x = distance * -sin( yawf ) * cos( pitchf );
  *y = distance * sin( yawf ) * sin( pitchf );
  *z = distance * cos( pitchf );

  //Serial.println( " Computing xyz from angles and distances : " 
  //                    + String( *x ) + " " 
  //                    + String( *y ) + " "
  //                    + String( *z ) 
  //                    + " -- point indexes "
  //                    + hAngleDegree + " " 
  //                    + vAngleDegree 
  //              );

}

void getTFminiData(int* distance, int* strength)
{
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (SerialTFMini.available())
  {
    rx[i] = SerialTFMini.read();
    if (rx[0] != 0x59)
    {
      i = 0;
    }
    else if (i == 1 && rx[1] != 0x59)
    {
      i = 0;
    }
    else if (i == 8)
    {
      for (j = 0; j < 8; j++)
      {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256))
      {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    }
    else
    {
      i++;
    }
  }
}