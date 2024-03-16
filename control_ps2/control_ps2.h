
#include<PS2X_lib.h>

//các biến khởi tạo để đọc giá trị tay cầm ps2
PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;
// -------------------------------------------------------

double height;

int drone_height=0;



void setup_ps2(){
    error = ps2x.config_gamepad(13,11,10,12, true, true);// clock - cmd - atten - data
  if (error == 0){
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
  }
  if (error == 1){
    Serial.println("No controller found, check wiring, see readme.txt to enable debugs");
  }
  if (error == 2){
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debugs");
  }
  if (error == 3){
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  }
  type = ps2x.readType();
  switch (type){
    case 0:
    Serial.println("Unknown Controller type");
    break;
    case 1:
    Serial.println("DualShock Controller Found");
    break;
    case 2:
    Serial.println("GuitarHero Controller Found");
    break;
  }
}


void control_motor(){
    if (error == 1) return;
    if(type == 2){ //Guitar Hero Controller
 
        ps2x.read_gamepad(); //read controller
 
        if(ps2x.ButtonPressed(GREEN_FRET))      Serial.println("Green Fret Pressed");
        if(ps2x.ButtonPressed(RED_FRET))        Serial.println("Red Fret Pressed");
        if(ps2x.ButtonPressed(YELLOW_FRET))     Serial.println("Yellow Fret Pressed");
        if(ps2x.ButtonPressed(BLUE_FRET))       Serial.println("Blue Fret Pressed");
        if(ps2x.ButtonPressed(ORANGE_FRET))     Serial.println("Orange Fret Pressed");
        if(ps2x.ButtonPressed(STAR_POWER))      Serial.println("Star Power Command");
        if(ps2x.Button(UP_STRUM))       Serial.println("Up Strum");             //will be TRUE as long as button is pressed
        if(ps2x.Button(DOWN_STRUM))     Serial.println("DOWN Strum");
        if(ps2x.Button(PSB_START))      Serial.println("Start is being held");  //will be TRUE as long as button is pressed
        if(ps2x.Button(PSB_SELECT))     Serial.println("Select is being held");
        if(ps2x.Button(ORANGE_FRET)){ // print stick value IF TRUE
    
            Serial.print("Wammy Bar Position:");
            Serial.println(ps2x.Analog(WHAMMY_BAR), DEC);
        }
    }
    else { // dualShock
        ps2x.read_gamepad(false, vibrate);
        if(ps2x.Button(PSB_START))      Serial.println("Start is being held");  //will be TRUE as long as button is pressed
        if(ps2x.Button(PSB_SELECT))     Serial.println("Select is being held");
        if(ps2x.Button(PSB_PAD_UP)) { //will be TRUE as long as button is pressed
            Serial.print("Up held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
        }
        if(ps2x.Button(PSB_PAD_RIGHT)){
            Serial.print("Right held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
        }
        if(ps2x.Button(PSB_PAD_LEFT)){
            Serial.print("LEFT held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
        }
        if(ps2x.Button(PSB_PAD_DOWN)){
            Serial.print("DOWN held this hard: ");
            Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
        }

        vibrate = ps2x.Analog(PSAB_BLUE);       //this will set the large motor vibrate speed based on
                                                //how hard you press the blue (X) button

        if (ps2x.NewButtonState()) //will be TRUE if any button changes state (on to off, or off to on)
        {
            if(ps2x.Button(PSB_L3))     Serial.println("L3 pressed");
            if(ps2x.Button(PSB_R3)){
                Serial.println("R3 pressed");
               
            }
            if(ps2x.Button(PSB_L2))     Serial.println("L2 pressed");
            if(ps2x.Button(PSB_R2))     Serial.println("R2 pressed");
            if(ps2x.Button(PSB_GREEN))  Serial.println("Triangle pressed");

        }

        if(ps2x.ButtonPressed(PSB_RED))    {
          Serial.println("Circle just pressed");  
          
          }        //will be TRUE if button was JUST pressed
        if(ps2x.ButtonReleased(PSB_PINK))  Serial.println("Square just released");         //will be TRUE if button was JUST released
        if(ps2x.NewButtonState(PSB_BLUE))  Serial.println("X just changed");               //will be TRUE if button was JUST pressed OR released
    

        if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) // print stick values if either is TRUE
        {

            height = ps2x.Analog(PSS_LY);
           if(height == 0) drone_height ++;
           else if(height == 255) drone_height --;
    // Serial.print("Stick Values:");
    // Serial.print(- ps2x.Analog(PSS_LY ) + 128); //Left stick, Y axis. Other options: LX, RY, RX
    // Serial.print(",");
    // Serial.print(ps2x.Analog(PSS_LX) - 128);
    // Serial.print(",");
    // Serial.print(- ps2x.Analog(PSS_RY) + 128);
    // Serial.print(",");
    // Serial.println(ps2x.Analog(PSS_RX) - 128);
  
    //         double LeftX = ps2x.Analog(PSS_LX) - 128;
    //         double LeftY = - ps2x.Analog(PSS_LY) + 128;
    //         if (LeftX == 127) LeftX += 1;
    //         if(LeftY == -127) LeftY-=1;
    //         double alpha = acos((double)(abs(LeftY) / 128)) - asin((double)(LeftX / 128));
    //         double beta = acos((double)(abs(LeftY) / 128)) + asin((double)(LeftX / 128)); // dieu khien van toc
    // // Serial.print("alpha: ");
    // // Serial.print(alpha);
    // // Serial.print("\t");
    // // Serial.print("beta: ");
    // // Serial.println(beta);
    
    //         if (LeftX ==0 && LeftY == 0){
    //             speedL = 0;
    //             speedR = 0;
    //         }
    //         else {
    //             speedR = speed0 + (alpha-beta) * 55 / PI ;
    //             speedL = speed0 - (alpha-beta) * 55 / PI ;
    //         }
    //         if( LeftY< 0 ) {
    //             speedL = -speedL;
    //             speedR = - speedR;
    //         }
    
    //         speedL = max(min(speedL,255),-255);
    //         speedR = max(min(speedR,255),-255);
    //         if(speedL >= 205 ){
    //             speedR/=4;
    //         }
    //         if(speedR >= 205){
    //             speedL/=4;
    //         }
    //         control[0] = speedL;
    //         control[1] = speedR;
    //         Serial.print("SpeedR: ");
    //         Serial.print(speedR);
    //         Serial.print("\t");
    //         Serial.print("SpeedL: ");
    //         Serial.println(speedL);
    //         //radio.write(&speedmotor, sizeof(speedmotor));
    //         //kenh.write(speedmotor);

    //         double rightX = map(ps2x.Analog(PSS_RX),0,255,0,180);
    //         double rightY =map(abs( ps2x.Analog(PSS_RY)-255 ),0,255,0,90);


    //         Serial.print("servo1 : ");    Serial.println(rightX);
    //         Serial.print("servo2 : ");    Serial.println(rightY);
    //         Serial.print("servo3 : ");    Serial.println(control[4]);
    //         control[2]=rightX;
    //         control[3]= rightY;
   
        }

    }
   // delay(300);
}