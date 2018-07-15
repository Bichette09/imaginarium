/* This program is a parsing routine of TF02 product standard output protocol on Arduino.
The format of data package is 0x59 0x59 Dist_L Dist_H Strength_L Strength_H Sequence_L Sequence_H
CheckSum_L
Refer to the product specification for detailed description.
For Arduino board with one serial port, use software to virtualize serial port’s functions: such as UNO board.
*/
/*For Arduino board with multiple serial ports such as DUE board, comment out the above two codes, and directly use
Serial1 port*/
#include<SoftwareSerial.h>// soft serial port header file
SoftwareSerial SerialLidar(2,3); // define the soft serial port as Serial1, pin2 as RX, and pin3 as TX

/*Constant for LiDAR TFMini*/
float distance1_cm;// LiDAR actually measured distance value
int check;// check numerical value storage
int i;
int uart[9];// store data measured by LiDAR
const int HEADER=0x59;// data package frame header

void setup()
{
  Serial.begin(9600);//set the Baud rate of Arduino and computer serial port
  SerialLidar.begin(115200);//set the Baud rate of LiDAR and Arduino serial port
}
void loop()
{
  if (SerialLidar.available())//check whether the serial port has data input
  {
    if(SerialLidar.read()==HEADER)// determine data package frame header 0x59
    {
      uart[0]=HEADER;
      if(SerialLidar.read()==HEADER)//determine data package frame header 0x59
      {
        uart[1]=HEADER;
        for(i=2;i<9;i++)// store data to array
        {
          uart[i]=SerialLidar.read();
        }
        check=uart[0]+uart[1]+uart[2]+uart[3]+uart[4]+uart[5]+uart[6]+uart[7];
        if(uart[8]==(check&0xff))// check the received data as per protocols
        {
          distance1_cm=uart[2]+uart[3]*256;// calculate distance value
          Serial.print(F("["));   
          Serial.print(distance1_cm, 2);
          Serial.println(("]"));
         }
        }
      }
    }
}
