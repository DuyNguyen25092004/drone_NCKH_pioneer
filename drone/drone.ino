#include<Servo.h>
#include <control_ps2.h>


#define ESC_PIN1 2
#define ESC_PIN2 3
#define ESC_PIN3 4
#define ESC_PIN4 5

Servo m1,m2,m3,m4;

bool check = true;


void calibrate(){
  Serial.begin(9600);
  m1.write(180);
  m2.write(180);
  m3.write(180);
  m4.write(180);
  delay(2000);
  m1.write(0);
  m2.write(0);
  m3.write(0);
  m4.write(0);
  delay(2000);
}

void setup() 
{
  Serial.begin(9600);
  m1.attach(ESC_PIN1,  1000, 2000);
  m2.attach(ESC_PIN2, 1000, 2000);
  m3.attach(ESC_PIN3,  1000, 2000);
  m4.attach(ESC_PIN4, 1000, 2000);

  
  
  setup_ps2(); // clock - cmd - atten - data
 while(check) { 
  ps2x.read_gamepad(false, 0);
  if(ps2x.ButtonPressed(PSB_RED)) check = false;
  delay(50);
 };
 if ( !check) Serial.println("da bam calibrate");
 delay(500);
 calibrate();
}

void loop() 
{
  control_motor();
  
  //  if(ps2x.ButtonPressed(PSB_RED) && check)    {
  //         Serial.println("Circle just pressed va calibrate");  
  //         calibrate();
  //         check=false;
  //         }  
//  if (!check){ 
  
  drone_height= max(min(drone_height,180),0) ;
  Serial.println(drone_height);
    m1.write(drone_height); 
    m2.write(drone_height);
    m3.write(drone_height); 
    m4.write(drone_height);
  

   delay(10);
}