#include <Arduino.h>

int ID1 = 72; // Decimal representation of the ID of the first LED
int ID2 = 31; // Decimal representation of the ID of the Second LED
int ID3 = 1023; // Decimal representation of the ID of the Third LED

bool binaryID1[10]; // Here we will store the binary representation of LED1's ID
bool binaryID2[10]; // Here we will store the binary representation of LED2's ID
bool binaryID3[10]; // Here we will store the binary representation of LED3's ID

int CameraFPS = 40; // used such that every frame bit will be displayed in one frame
double delayTime = (1.0/CameraFPS)*1000; // delay time between each bit in ms

// Function to convert decimal to binary
void DecimalToBinary(int value, bool order[10]){
  const int bits = 10;
  char binary[10+2] = {0}; //This is where the binary representation will be stored
  value += pow(2,10); //Adding 2**10 so that there will always be 11 digits in the string
  itoa(value,binary,2); //Convert value to a string using base 2 and save it in the string named binary
  char* string = binary + 1 ; //get rid of the most significant digit as you only want 10 bits

  for (int i=10-1, j=0; i >= 0; i--, j++){
    order[j] = string[i] - '0';
    }
}

void setup()
{
 Serial.begin(9600);
 pinMode(2,OUTPUT);
 pinMode(3,OUTPUT);
 pinMode(4,OUTPUT);

 // converting Decimal IDs to binary ones
 DecimalToBinary(ID1, binaryID1); 
 DecimalToBinary(ID2, binaryID2);
 DecimalToBinary(ID3, binaryID3);

}



void loop()
{
  for (int i=0; i<10; i++){
    digitalWrite(2, binaryID1[i]);
    digitalWrite(3, binaryID2[i]);
    digitalWrite(4, binaryID3[i]);
//    Serial.println(delayTime);
    delay(delayTime);
    }
}