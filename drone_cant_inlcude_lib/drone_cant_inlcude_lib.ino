
/******************************************************************
*  Super amazing PS2 controller Arduino Library v1.8
*		details and example sketch: 
*			http://www.billporter.info/?p=240
*
*    Original code by Shutter on Arduino Forums
*
*    Revamped, made into lib by and supporting continued development:
*              Bill Porter
*              www.billporter.info
*
*	 Contributers:
*		Eric Wetzel (thewetzel@gmail.com)
*		Kurt Eckhardt
*
*  Lib version history
*    0.1 made into library, added analog stick support. 
*    0.2 fixed config_gamepad miss-spelling
*        added new functions:
*          NewButtonState();
*          NewButtonState(unsigned int);
*          ButtonPressed(unsigned int);
*          ButtonReleased(unsigned int);
*        removed 'PS' from begining of ever function
*    1.0 found and fixed bug that wasn't configuring controller
*        added ability to define pins
*        added time checking to reconfigure controller if not polled enough
*        Analog sticks and pressures all through 'ps2x.Analog()' function
*        added:
*          enableRumble();
*          enablePressures();
*    1.1  
*        added some debug stuff for end user. Reports if no controller found
*        added auto-increasing sentence delay to see if it helps compatibility.
*    1.2
*        found bad math by Shutter for original clock. Was running at 50kHz, not the required 500kHz. 
*        fixed some of the debug reporting. 
*	1.3 
*	    Changed clock back to 50kHz. CuriousInventor says it's suppose to be 500kHz, but doesn't seem to work for everybody. 
*	1.4
*		Removed redundant functions.
*		Fixed mode check to include two other possible modes the controller could be in.
*       Added debug code enabled by compiler directives. See below to enable debug mode.
*		Added button definitions for shapes as well as colors.
*	1.41
*		Some simple bug fixes
*		Added Keywords.txt file
*	1.5
*		Added proper Guitar Hero compatibility
*		Fixed issue with DEBUG mode, had to send serial at once instead of in bits
*	1.6
*		Changed config_gamepad() call to include rumble and pressures options
*			This was to fix controllers that will only go into config mode once
*			Old methods should still work for backwards compatibility 
*    1.7
*		Integrated Kurt's fixes for the interrupts messing with servo signals
*		Reorganized directory so examples show up in Arduino IDE menu
*    1.8
*		Added Arduino 1.0 compatibility. 
*    1.9
*       Kurt - Added detection and recovery from dropping from analog mode, plus
*       integreated Chipkit (pic32mx...) support
*
*
*
*This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
<http://www.gnu.org/licenses/>
*  
******************************************************************/


// $$$$$$$$$$$$ DEBUG ENABLE SECTION $$$$$$$$$$$$$$$$
// to debug ps2 controller, uncomment these two lines to print out debug to uart

//#define PS2X_DEBUG
//#define PS2X_COM_DEBUG


#ifndef PS2X_lib_h
#define PS2X_lib_h
#if ARDUINO > 22
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#ifdef __AVR__
#include <avr/io.h>

#define CTRL_CLK        4
#define CTRL_BYTE_DELAY 4 //3
#else
// Pic32...
#include <pins_arduino.h>
#define CTRL_CLK        6 //5
#define CTRL_CLK_HIGH   6 //5
#define CTRL_BYTE_DELAY 5 //4
#endif 

//These are our button constants
#define PSB_SELECT      0x0001
#define PSB_L3          0x0002
#define PSB_R3          0x0004
#define PSB_START       0x0008
#define PSB_PAD_UP      0x0010
#define PSB_PAD_RIGHT   0x0020
#define PSB_PAD_DOWN    0x0040
#define PSB_PAD_LEFT    0x0080
#define PSB_L2          0x0100
#define PSB_R2          0x0200
#define PSB_L1          0x0400
#define PSB_R1          0x0800
#define PSB_GREEN       0x1000
#define PSB_RED         0x2000
#define PSB_BLUE        0x4000
#define PSB_PINK        0x8000
#define PSB_TRIANGLE    0x1000
#define PSB_CIRCLE      0x2000
#define PSB_CROSS       0x4000
#define PSB_SQUARE      0x8000

//Guitar  button constants
#define GREEN_FRET		0x0200
#define RED_FRET		0x2000
#define YELLOW_FRET		0x1000
#define BLUE_FRET		0x4000
#define ORANGE_FRET		0x8000
#define STAR_POWER		0x0100
#define UP_STRUM		0x0010
#define DOWN_STRUM		0x0040
#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

//These are analog buttons
#define PSAB_PAD_RIGHT   9
#define PSAB_PAD_UP      11
#define PSAB_PAD_DOWN    12
#define PSAB_PAD_LEFT    10
#define PSAB_L2          19
#define PSAB_R2          20
#define PSAB_L1          17
#define PSAB_R1          18
#define PSAB_GREEN       13
#define PSAB_RED         14
#define PSAB_BLUE        15
#define PSAB_PINK        16
#define PSAB_TRIANGLE    13
#define PSAB_CIRCLE      14
#define PSAB_CROSS       15
#define PSAB_SQUARE      16


#define SET(x,y) (x|=(1<<y))
#define CLR(x,y) (x&=(~(1<<y)))
#define CHK(x,y) (x & (1<<y))
#define TOG(x,y) (x^=(1<<y))



class PS2X {
public:
boolean Button(uint16_t);
unsigned int ButtonDataByte();
boolean NewButtonState();
boolean NewButtonState(unsigned int);
boolean ButtonPressed(unsigned int);
boolean ButtonReleased(unsigned int);
void read_gamepad();
boolean  read_gamepad(boolean, byte);
byte readType();
byte config_gamepad(uint8_t, uint8_t, uint8_t, uint8_t);
byte config_gamepad(uint8_t, uint8_t, uint8_t, uint8_t, bool, bool);
void enableRumble();
bool enablePressures();
byte Analog(byte);
void reconfig_gamepad();
private:

inline void CLK_SET(void);
inline void CLK_CLR(void);
inline void CMD_SET(void);
inline void CMD_CLR(void);
inline void ATT_SET(void);
inline void ATT_CLR(void);
inline bool DAT_CHK(void);

unsigned char _gamepad_shiftinout (char);
unsigned char PS2data[21];
void sendCommandString(byte*, byte);
unsigned char i;
unsigned int last_buttons;
unsigned int buttons;
#ifdef __AVR__
uint8_t maskToBitNum(uint8_t);
uint8_t _clk_mask; 
volatile uint8_t *_clk_oreg;
uint8_t _cmd_mask; 
volatile uint8_t *_cmd_oreg;
uint8_t _att_mask; 
volatile uint8_t *_att_oreg;
uint8_t _dat_mask; 
volatile uint8_t *_dat_ireg;
#else
uint8_t maskToBitNum(uint8_t);
uint16_t 				_clk_mask; 
volatile uint32_t *		_clk_lport_set;
volatile uint32_t *		_clk_lport_clr;
uint16_t 				_cmd_mask; 
volatile uint32_t *		_cmd_lport_set;
volatile uint32_t *		_cmd_lport_clr;
uint16_t 				_att_mask; 
volatile uint32_t *		_att_lport_set;
volatile uint32_t *		_att_lport_clr;
uint16_t 				_dat_mask; 
volatile uint32_t *		_dat_lport;
#endif
unsigned long last_read;
byte read_delay;
byte controller_type;
boolean en_Rumble;
boolean en_Pressures;

};

#endif



#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "pins_arduino.h"




static byte enter_config[]={0x01,0x43,0x00,0x01,0x00};
static byte set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
static byte set_bytes_large[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
static byte exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
static byte enable_rumble[]={0x01,0x4D,0x00,0x00,0x01};
static byte type_read[]={0x01,0x45,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};

boolean PS2X::NewButtonState() {
    return ((last_buttons ^ buttons) > 0);

}

boolean PS2X::NewButtonState(unsigned int button) {
    return (((last_buttons ^ buttons) & button) > 0);
}

boolean PS2X::ButtonPressed(unsigned int button) {
    return(NewButtonState(button) & Button(button));
}

boolean PS2X::ButtonReleased(unsigned int button) {
    return((NewButtonState(button)) & ((~last_buttons & button) > 0));
}

boolean PS2X::Button(uint16_t button) {
    return ((~buttons & button) > 0);
}

unsigned int PS2X::ButtonDataByte() {
    return (~buttons);
}

byte PS2X::Analog(byte button) {
    return PS2data[button];
}
unsigned char PS2X::_gamepad_shiftinout (char byte) {


    unsigned char tmp = 0;
    for(i=0;i<8;i++) {

        if(CHK(byte,i)) CMD_SET();
        else  CMD_CLR();
        CLK_CLR();

        delayMicroseconds(CTRL_CLK);

        if(DAT_CHK()) SET(tmp,i);
        CLK_SET();
#if CTRL_CLK_HIGH
        delayMicroseconds(CTRL_CLK_HIGH);
#endif	  
    }
    CMD_SET();
    delayMicroseconds(CTRL_BYTE_DELAY);
    return tmp;
}

void PS2X::read_gamepad() {
    read_gamepad(false, 0x00);
}


boolean PS2X::read_gamepad(boolean motor1, byte motor2) {
    double temp = millis() - last_read;

    if (temp > 1500) //waited to long
        reconfig_gamepad();

    if(temp < read_delay)  //waited too short
        delay(read_delay - temp);



    if(motor2 != 0x00)
        motor2 = map(motor2,0,255,0x40,0xFF); //noting below 40 will make it spin

    byte dword[9] = {0x01,0x42,0,motor1,motor2,0,0,0,0};
    byte dword2[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

    // Try a few times to get valid data...
    for (byte RetryCnt = 0; RetryCnt < 5; RetryCnt++) {
        CMD_SET();
        CLK_SET();
        ATT_CLR(); // low enable joystick

        delayMicroseconds(CTRL_BYTE_DELAY);
        //Send the command to send button and joystick data;

        for (int i = 0; i<9; i++) {
            PS2data[i] = _gamepad_shiftinout(dword[i]);
        }


        if(PS2data[1] == 0x79) {  //if controller is in full data return mode, get the rest of data
            for (int i = 0; i<12; i++) {
                PS2data[i+9] = _gamepad_shiftinout(dword2[i]);
            }
        }

        ATT_SET(); // HI disable joystick
        // Check to see if we received valid data or not.  We should be in analog mode for our data
        // to be valie
        if ((PS2data[1] & 0xf0) == 0x70)
            break;

        // If we got to here, we are not in analog mode, try to recover...
        reconfig_gamepad();	// try to get back into Analog mode.
        delay(read_delay);
    }

    // If we get here and still not in analog mode, try increasing the read_delay...
    if ((PS2data[1] & 0xf0) != 0x70) {
        if (read_delay < 10)
            read_delay++;	// see if this helps out...
    }	


#ifdef PS2X_COM_DEBUG
    Serial.println("OUT:IN");
    for(int i=0; i<9; i++){
        Serial.print(dword[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i], HEX);
        Serial.print(" ");
    }
    for (int i = 0; i<12; i++) {
        Serial.print(dword2[i], HEX);
        Serial.print(":");
        Serial.print(PS2data[i+9], HEX);
        Serial.print(" ");
    }
    Serial.println("");	
#endif

    last_buttons = buttons; //store the previous buttons states

#if defined(__AVR__)
    uint16_t * tmp = (uint16_t*)(PS2data+3);
    buttons = *tmp;
#else
    buttons =  (uint16_t)(PS2data[4] << 8) + PS2data[3];   //store as one value for multiple functions
#endif
    last_read = millis();
    return ((PS2data[1] & 0xf0) == 0x70);
}

byte PS2X::config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat) {
    return config_gamepad(clk, cmd, att, dat, false, false);
}


byte PS2X::config_gamepad(uint8_t clk, uint8_t cmd, uint8_t att, uint8_t dat, bool pressures, bool rumble) {

    byte temp[sizeof(type_read)];

#ifdef __AVR__
    _clk_mask = digitalPinToBitMask(clk);
    _clk_oreg = portOutputRegister(digitalPinToPort(clk));
    _cmd_mask = digitalPinToBitMask(cmd);
    _cmd_oreg = portOutputRegister(digitalPinToPort(cmd));
    _att_mask = digitalPinToBitMask(att);
    _att_oreg = portOutputRegister(digitalPinToPort(att));
    _dat_mask = digitalPinToBitMask(dat);
    _dat_ireg = portInputRegister(digitalPinToPort(dat));
#else

    uint32_t            lport;                   // Port number for this pin
    _clk_mask = digitalPinToBitMask(clk); 
    lport = digitalPinToPort(clk);
    _clk_lport_set = portOutputRegister(lport) + 2;
    _clk_lport_clr = portOutputRegister(lport) + 1;

    _cmd_mask = digitalPinToBitMask(cmd); 
    lport = digitalPinToPort(cmd);
    _cmd_lport_set = portOutputRegister(lport) + 2;
    _cmd_lport_clr = portOutputRegister(lport) + 1;

    _att_mask = digitalPinToBitMask(att); 
    lport = digitalPinToPort(att);
    _att_lport_set = portOutputRegister(lport) + 2;
    _att_lport_clr = portOutputRegister(lport) + 1;

    _dat_mask = digitalPinToBitMask(dat); 
    _dat_lport = portInputRegister(digitalPinToPort(dat));

#endif  

    pinMode(clk, OUTPUT); //configure ports
    pinMode(att, OUTPUT);
    pinMode(cmd, OUTPUT);
    pinMode(dat, INPUT);

#if defined(__AVR__)
    digitalWrite(dat, HIGH); //enable pull-up 
#endif

    CMD_SET(); // SET(*_cmd_oreg,_cmd_mask);
    CLK_SET();

    //new error checking. First, read gamepad a few times to see if it's talking
    read_gamepad();
    read_gamepad();

    //see if it talked
    if(PS2data[1] != 0x41 && PS2data[1] != 0x73 && PS2data[1] != 0x79){ //see if mode came back. If still anything but 41, 73 or 79, then it's not talking
#ifdef PS2X_DEBUG
        Serial.println("Controller mode not matched or no controller found");
        Serial.print("Expected 0x41 or 0x73, got ");
        Serial.println(PS2data[1], HEX);
#endif

        return 1; //return error code 1
    }

    //try setting mode, increasing delays if need be. 
    read_delay = 1;

    for(int y = 0; y <= 10; y++)
    {
        sendCommandString(enter_config, sizeof(enter_config)); //start config run

        //read type
        delayMicroseconds(CTRL_BYTE_DELAY);

        CMD_SET();
        CLK_SET();
        ATT_CLR(); // low enable joystick

        delayMicroseconds(CTRL_BYTE_DELAY);

        for (int i = 0; i<9; i++) {
            temp[i] = _gamepad_shiftinout(type_read[i]);
        }

        ATT_SET(); // HI disable joystick

        controller_type = temp[3];

        sendCommandString(set_mode, sizeof(set_mode));
        if(rumble){ sendCommandString(enable_rumble, sizeof(enable_rumble)); en_Rumble = true; }
        if(pressures){ sendCommandString(set_bytes_large, sizeof(set_bytes_large)); en_Pressures = true; }
        sendCommandString(exit_config, sizeof(exit_config));

        read_gamepad();

        if(pressures){
            if(PS2data[1] == 0x79)
                break;
            if(PS2data[1] == 0x73)
                return 3;
        }

        if(PS2data[1] == 0x73)
            break;

        if(y == 10){
#ifdef PS2X_DEBUG
            Serial.println("Controller not accepting commands");
            Serial.print("mode stil set at");
            Serial.println(PS2data[1], HEX);
#endif
            return 2; //exit function with error
        }

        read_delay += 1; //add 1ms to read_delay
    }

    return 0; //no error if here
}



void PS2X::sendCommandString(byte string[], byte len) {


#ifdef PS2X_COM_DEBUG
    byte temp[len];
    ATT_CLR(); // low enable joystick
    delayMicroseconds(CTRL_BYTE_DELAY);

    for (int y=0; y < len; y++)
        temp[y] = _gamepad_shiftinout(string[y]);

    ATT_SET(); //high disable joystick  
    delay(read_delay);                  //wait a few

    Serial.println("OUT:IN Configure");
    for(int i=0; i<len; i++){
        Serial.print(string[i], HEX);
        Serial.print(":");
        Serial.print(temp[i], HEX);
        Serial.print(" ");
    }
    Serial.println("");

#else
    ATT_CLR(); // low enable joystick
    for (int y=0; y < len; y++)
        _gamepad_shiftinout(string[y]);

    ATT_SET(); //high disable joystick  
    delay(read_delay);                  //wait a few
#endif
}



byte PS2X::readType() {
    /*
       byte temp[sizeof(type_read)];

       sendCommandString(enter_config, sizeof(enter_config));

       delayMicroseconds(CTRL_BYTE_DELAY);

       CMD_SET();
       CLK_SET();
       ATT_CLR(); // low enable joystick

       delayMicroseconds(CTRL_BYTE_DELAY);

       for (int i = 0; i<9; i++) {
       temp[i] = _gamepad_shiftinout(type_read[i]);
       }

       sendCommandString(exit_config, sizeof(exit_config));

       if(temp[3] == 0x03)
       return 1;
       else if(temp[3] == 0x01)
       return 2;

       return 0;
     */

    if(controller_type == 0x03)
        return 1;
    else if(controller_type == 0x01)
        return 2;

    return 0;

}

void PS2X::enableRumble() {

    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(enable_rumble, sizeof(enable_rumble));
    sendCommandString(exit_config, sizeof(exit_config));
    en_Rumble = true;

}

bool PS2X::enablePressures() {

    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));

    read_gamepad();
    read_gamepad();

    if(PS2data[1] != 0x79)
        return false;

    en_Pressures = true;
    return true;
}

void PS2X::reconfig_gamepad(){

    sendCommandString(enter_config, sizeof(enter_config));
    sendCommandString(set_mode, sizeof(set_mode));
    if (en_Rumble)
        sendCommandString(enable_rumble, sizeof(enable_rumble));
    if (en_Pressures)
        sendCommandString(set_bytes_large, sizeof(set_bytes_large));
    sendCommandString(exit_config, sizeof(exit_config));

}


#ifdef __AVR__
inline void  PS2X::CLK_SET(void) {

    register uint8_t old_sreg = SREG;
    cli();
    *_clk_oreg |= _clk_mask;
    SREG = old_sreg;
}

inline void  PS2X::CLK_CLR(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *_clk_oreg &= ~_clk_mask;
    SREG = old_sreg;
}

inline void  PS2X::CMD_SET(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *_cmd_oreg |= _cmd_mask; // SET(*_cmd_oreg,_cmd_mask);
    SREG = old_sreg;
}

inline void  PS2X::CMD_CLR(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *_cmd_oreg &= ~_cmd_mask; // SET(*_cmd_oreg,_cmd_mask);
    SREG = old_sreg;
}

inline void  PS2X::ATT_SET(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *_att_oreg |= _att_mask ; 	
    SREG = old_sreg;
}

inline void PS2X::ATT_CLR(void) {
    register uint8_t old_sreg = SREG;
    cli();
    *_att_oreg &= ~_att_mask; 
    SREG = old_sreg;
}

inline bool PS2X::DAT_CHK(void) {
    return (*_dat_ireg & _dat_mask)? true : false;
}

#else
// On pic32, use the set/clr registers to make them atomic...inline void  PS2X::CLK_SET(void) {
*_clk_lport_set |= _clk_mask;
}

inline void  PS2X::CLK_CLR(void) {
    *_clk_lport_clr |= _clk_mask;
}

inline void  PS2X::CMD_SET(void) {
    *_cmd_lport_set |= _cmd_mask;
}

inline void  PS2X::CMD_CLR(void) {
    *_cmd_lport_clr |= _cmd_mask;
}

inline void  PS2X::ATT_SET(void) {
    *_att_lport_set |= _att_mask;
}

inline void PS2X::ATT_CLR(void) {
    *_att_lport_clr |= _att_mask;
}

inline bool PS2X::DAT_CHK(void) {
    return (*_dat_lport & _dat_mask)? true : false;

}

#endif

//--------------------------------------------------------------------------------------

PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;


double height;

int drone_height=0;

//--------------------------------------------------------------------------------------

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

        // if(ps2x.ButtonPressed(PSB_RED))    {
        //   Serial.println("Circle just pressed");  
          
        //   }        //will be TRUE if button was JUST pressed
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

#include<Servo.h>


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
