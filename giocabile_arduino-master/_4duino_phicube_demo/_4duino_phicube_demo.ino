//
// NB! This is a file generated from the .4Dino file, changes will be lost
//     the next time the .4Dino file is built
//
#define RESETLINE     30

#define DisplaySerial Serial1


#include "_4duino_phicube_demoConst.h"
#include "Communication.h"
#include <Wire.h>
#include "Picaso_Serial_4DLib.h"
#include "Picaso_Const4D.h"


Picaso_Serial_4DLib Display(&DisplaySerial);
word hndl;
int state, n, i;
bool displayed = false;
State current_state = STARTUP;
Mode current_mode = NO_MODE;
Mode desired_mode = NO_MODE;


//routine to handle Serial errors
void mycallback(int ErrCode, unsigned char Errorbyte)
{
  #ifdef LOG_MESSAGES
    const char *Error4DText[] = {"OK\0", "Timeout\0", "NAK\0", "Length\0", "Invalid\0"};
    LOG_MESSAGES.print(F("Serial 4D Library reports error "));
    LOG_MESSAGES.print(Error4DText[ErrCode]);
    if (ErrCode == Err4D_NAK) {
      LOG_MESSAGES.print(F(" returned data= "));
      LOG_MESSAGES.println(Errorbyte);
    }
    else {
      LOG_MESSAGES.println(F(""));
    }
    while (1);
  #else
    #define led 13
    while (1)
    {
      digitalWrite(led, HIGH);
      delay(200);
      digitalWrite(led, LOW);
      delay(200);
    }
  #endif
}


//routine to handle I2C received msgs
void receiveEvent(int bytes)
{
  I2Cmsg msg = Wire.read();
  if (msg == I2C_ERROR) {
    current_state = PHICUBE_ERROR;
  }
  else {
    if (current_state == CURRENTLY_HOMING) {
      if (msg == I2C_SUCCESS) {
        current_state = HOMED;
      }
      else {
        current_state = PHICUBE_ERROR;
      }
    }
    if (current_state == INITIALIZING) {
      if (msg == I2C_SUCCESS) {
        current_state = INITIALIZED;
        current_mode = desired_mode;
      }
      else {
        current_state = PHICUBE_ERROR;
      }
    }
    if (current_state == PHICUBE_ERROR) {
      if (msg == I2C_RESET) {
        current_state = INITIALIZED;
        current_mode = NO_MODE;
      }
    }
  }
}


void stateMachine()
{
  switch (current_state) {

    case STARTUP:
      Display.gfx_BGcolour(0xFFFF) ;
      Display.gfx_Cls();
      Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
      Display.img_Show(hndl,iImage1);
      delay(5000);
      Display.img_ClearAttributes(hndl, iWinbutton1, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton1, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton1);
      Display.touch_Set(TOUCH_ENABLE);
      current_state = NOT_HOMED;
    break;

    case NOT_HOMED:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED && n == iWinbutton1) {
        Display.img_SetWord(hndl, iWinbutton1, IMAGE_INDEX, 1);
        Display.img_Show(hndl,iWinbutton1);
      }
      if (state == TOUCH_RELEASED && n == iWinbutton1) {
        Display.img_SetWord(hndl, iWinbutton1, IMAGE_INDEX, 2);
        Display.img_Show(hndl,iWinbutton1);
        Wire.beginTransmission(0x70);
        Wire.write(I2C_HOME);
        int error = Wire.endTransmission();
        current_state = CURRENTLY_HOMING;
        if (error != 0) {
          current_state = PHICUBE_ERROR;
        }
      }
    break;

    case CURRENTLY_HOMING:
      if (!displayed) {
        Display.gfx_BGcolour(0xFFFF);
        Display.gfx_Cls();
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iStatictext1);
        Display.img_Show(hndl,iImage2);
        displayed = true;
      }
    break;

    case HOMED:
      Display.gfx_BGcolour(0xFFFF);
      Display.gfx_Cls();
      Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
      Display.img_Show(hndl,iImage3);
      Display.img_ClearAttributes(hndl, iWinbutton2, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton2, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton2);
      Display.img_ClearAttributes(hndl, iWinbutton3, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton3, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton3);
      Display.img_ClearAttributes(hndl, iWinbutton4, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton4, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton4);
      Display.img_ClearAttributes(hndl, iWinbutton5, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton5, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton5);
      Display.img_ClearAttributes(hndl, iWinbutton6, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton6, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton6);
      Display.img_ClearAttributes(hndl, iWinbutton7, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton7, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton7);
      displayed = false;
      current_state = READY;
    break;

    case INITIALIZING:
      if (!displayed) {
        Display.gfx_BGcolour(0xFFFF);
        Display.gfx_Cls();
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iStatictext3);
        Display.img_Show(hndl,iImage4);
        displayed = true;
      }
    break;

    case INITIALIZED:
      Display.gfx_BGcolour(0xFFFF);
      Display.gfx_Cls();
      Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
      Display.img_Show(hndl,iImage3);

      Display.img_ClearAttributes(hndl, iWinbutton2, I_TOUCH_DISABLE);
      if (current_mode == CONFIGURATION_1) {
        Display.img_SetWord(hndl, iWinbutton2, IMAGE_INDEX, 3);
      }
      else {
         Display.img_SetWord(hndl, iWinbutton2, IMAGE_INDEX, 0);
      }
      Display.img_Show(hndl,iWinbutton2);

      Display.img_ClearAttributes(hndl, iWinbutton3, I_TOUCH_DISABLE);
      if (current_mode == CONFIGURATION_2) {
        Display.img_SetWord(hndl, iWinbutton3, IMAGE_INDEX, 3);
      }
      else {
        Display.img_SetWord(hndl, iWinbutton3, IMAGE_INDEX, 0);
      }
      Display.img_Show(hndl,iWinbutton3);

      Display.img_ClearAttributes(hndl, iWinbutton4, I_TOUCH_DISABLE);
      if (current_mode == CONFIGURATION_3) {
        Display.img_SetWord(hndl, iWinbutton4, IMAGE_INDEX, 3);
      }
      else {
        Display.img_SetWord(hndl, iWinbutton4, IMAGE_INDEX, 0);
      }
      Display.img_Show(hndl,iWinbutton4);
      Display.img_ClearAttributes(hndl, iWinbutton5, I_TOUCH_DISABLE);
      if (current_mode == CONFIGURATION_4) {
        Display.img_SetWord(hndl, iWinbutton5, IMAGE_INDEX, 3);
      }
      else {
        Display.img_SetWord(hndl, iWinbutton5, IMAGE_INDEX, 0);
      }
      Display.img_Show(hndl,iWinbutton5);
      Display.img_ClearAttributes(hndl, iWinbutton6, I_TOUCH_DISABLE);
      if (current_mode == CONFIGURATION_5) {
        Display.img_SetWord(hndl, iWinbutton6, IMAGE_INDEX, 3);
      }
      else {
        Display.img_SetWord(hndl, iWinbutton6, IMAGE_INDEX, 0);
      }
      Display.img_Show(hndl,iWinbutton6);
      Display.img_ClearAttributes(hndl, iWinbutton7, I_TOUCH_DISABLE);
      if (current_mode == CONFIGURATION_6) {
        Display.img_SetWord(hndl, iWinbutton7, IMAGE_INDEX, 3);
      }
      else {
        Display.img_SetWord(hndl, iWinbutton7, IMAGE_INDEX, 0);
      }
      Display.img_Show(hndl,iWinbutton7);
      current_state = READY;
    break;

    case READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton2 && n <= iWinbutton7) {
          for (i = iWinbutton7; i <= iWinbutton7; i++) {
            if (i == n) {
              Display.img_SetWord(hndl, i, IMAGE_INDEX, 1);
            }
            else {
              Display.img_SetWord(hndl, i, IMAGE_INDEX, 0);
            }
            Display.img_Show(hndl,i);
          }
        }
      }
      if (state == TOUCH_RELEASED) {
        if (n >= iWinbutton2 && n <= iWinbutton7) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case 4:
              desired_mode = CONFIGURATION_1;
              mode_msg = I2C_CONFIGURATION_1;
            break;
            case 5:
              desired_mode = CONFIGURATION_2;
              mode_msg = I2C_CONFIGURATION_2;
            break;
            case 6:
              desired_mode = CONFIGURATION_3;
              mode_msg = I2C_CONFIGURATION_3;
            break;
            case 7:
              desired_mode = CONFIGURATION_4;
              mode_msg = I2C_CONFIGURATION_4;
            break;
            case 8:
              desired_mode = CONFIGURATION_5;
              mode_msg = I2C_CONFIGURATION_5;
            break;
            case 9:
              desired_mode = CONFIGURATION_6;
              mode_msg = I2C_CONFIGURATION_6;
            break;
          }
          Wire.beginTransmission(0x70);
          Wire.write(mode_msg);
          int error = Wire.endTransmission();
          if (error != 0) {
            current_state = PHICUBE_ERROR;
          }
          else {
            displayed = false;
            current_state = INITIALIZING;
          }
        }
      }
    break;

    case PHICUBE_ERROR:
      Display.gfx_BGcolour(0xFFFF);
      Display.gfx_Cls();
      Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
      delay(500);
      Display.img_Show(hndl,iStatictext2);
      delay(500);
    break;
  }
}


void setup()
{
  Wire.begin(0x09);
  Wire.onReceive(receiveEvent);

  //reset display
  pinMode(RESETLINE, OUTPUT);
  digitalWrite(RESETLINE, 1);       // Reset Display, using shield
  delay(100);
  digitalWrite(RESETLINE, 0);       // Release Display Reset, using shield
  delay(3000);

  //start display
  DisplaySerial.begin(200000) ;     // Hardware serial to Display, same as SPE on display is set to
  Display.TimeLimit4D = 5000;
  Display.Callback4D = mycallback;

  //init display
  Display.gfx_ScreenMode(PORTRAIT);
  Display.putstr("Mounting...\n");
  if (!(Display.file_Mount())) {
    while(!(Display.file_Mount())) {
      Display.putstr("Drive not mounted...");
      delay(200);
      Display.gfx_Cls();
      delay(200);
    }
  }

  hndl = Display.file_LoadImageControl("_4DUIN~1.dat", "_4DUIN~1.gci", 1);
  Display.img_Show(hndl,iImage1) ;  // Image1
} // end Setup **do not alter, remove or duplicate this line**


void loop()
{
  stateMachine();
}
