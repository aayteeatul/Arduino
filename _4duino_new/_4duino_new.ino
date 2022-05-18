//
// NB! This is a file generated from the .4Dino file, changes will be lost
//     the next time the .4Dino file is built
//
#define RESETLINE     30

#define DisplaySerial Serial1


#include "_4duino_newConst.h"
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


// Routine to handle Serial errors
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
// end of routine to handle Serial errors



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
  Display.img_Show(hndl,iImage1);
}


// routine to handle I2C received msgs
void receiveEvent(int bytes)
{
  I2Cmsg msg = Wire.read();
  if (msg == I2C_ERROR) {
    current_state = PHICUBE_ERROR;
    //Display.putstr("ERROR received");
  }
  else {
    if (current_state == CURRENTLY_HOMING) {
      if (msg == I2C_SUCCESS) {
        //Display.putstr("HOMED received");
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
        current_state = HOMED;
        current_mode = NO_MODE;
      }
    }
  }
}


// routine to stateMachine
void stateMachine() {
  switch (current_state) {

    case STARTUP:
      Display.gfx_BGcolour(0xFFFF) ;
      Display.gfx_Cls() ;
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
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iStatictext1);
        Display.img_Show(hndl,iImage2);
        displayed = true;
      }
    break;

    case HOMED:
      Display.gfx_BGcolour(0xFFFF) ;
      Display.gfx_Cls() ;
      Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
      Display.img_Show(hndl,iImage3);
      Display.img_Show(hndl,iStatictext2);
      Display.img_ClearAttributes(hndl, iWinbutton2, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton2, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton2) ;
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
      Display.img_ClearAttributes(hndl, iWinbutton8, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton8, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton8) ;
      Display.img_ClearAttributes(hndl, iWinbutton9, I_TOUCH_DISABLE);
      Display.img_SetWord(hndl, iWinbutton9, IMAGE_INDEX, 0);
      Display.img_Show(hndl,iWinbutton9);
      displayed = false;
      current_state = CHOOSE;
    break;

    case CHOOSE:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);
      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton2 && n <= iWinbutton9) {
          for ( i = iWinbutton9; i <= iWinbutton9; i++) {
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
        if (n >= iWinbutton2 && n <= iWinbutton9) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          switch(n) {
            case iWinbutton2:
              current_state = PLATFORM;
            break;
            case iWinbutton3:
              current_state = GIUNGLA;
            break;
            case iWinbutton4:
              current_state = BOARDING;
            break;
            case iWinbutton5:
              current_state = ESCAVATORE;
            break;
            case iWinbutton6:
              current_state = AEROPLANO;
            break;
            case iWinbutton7:
              current_state = PESCA;
            break;
            case iWinbutton8:
              current_state = SCALATA;
            break;
            case iWinbutton9:
              current_state = DIAMANTE;
            break;
          }
        }
      }
    break;

    case PLATFORM:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage5);
        Display.img_Show(hndl,iStatictext4) ;
        Display.img_ClearAttributes(hndl, iWinbutton10, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton10, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton10);
        Display.img_ClearAttributes(hndl, iWinbutton11, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton11, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton11);
        Display.img_ClearAttributes(hndl, iWinbutton12, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton12, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton12);
        Display.img_ClearAttributes(hndl, iWinbutton13, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton13, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton13) ;
        displayed = true;
        current_state = PLATFORM_READY;
      }
    break;

    case PLATFORM_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton10 && n <= iWinbutton13) {
          for (i = iWinbutton13; i <= iWinbutton13; i++) {
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
        if (n >= iWinbutton10 && n <= iWinbutton13) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton10:
              desired_mode = CONFIGURATION_1;
              mode_msg = I2C_CONFIGURATION_1;
            break;
            case iWinbutton11:
              desired_mode = CONFIGURATION_2;
              mode_msg = I2C_CONFIGURATION_2;
            break;
            case iWinbutton12:
              desired_mode = CONFIGURATION_3;
              mode_msg = I2C_CONFIGURATION_3;
            break;
            case iWinbutton13:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
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

    case GIUNGLA:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls();
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage6) ;
        Display.img_Show(hndl,iStatictext5) ;
        Display.img_ClearAttributes(hndl, iWinbutton14, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton14, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton14) ;
        Display.img_ClearAttributes(hndl, iWinbutton15, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton15, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton15) ;
        Display.img_ClearAttributes(hndl, iWinbutton16, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton16, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton16) ;
        Display.img_ClearAttributes(hndl, iWinbutton17, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton17, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton17);
        Display.img_ClearAttributes(hndl, iWinbutton18, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton18, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton18);
        Display.img_ClearAttributes(hndl, iWinbutton19, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton19, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton19);
        current_state = GIUNGLA_READY;
        displayed = true;
      }
    break;

    case GIUNGLA_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton14 && n <= iWinbutton19) {
          for (i = iWinbutton19; i <= iWinbutton19; i++) {
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
        if (n >= iWinbutton14 && n <= iWinbutton19) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton14:
              desired_mode = CONFIGURATION_4;
              mode_msg = I2C_CONFIGURATION_4;
            break;
            case iWinbutton15:
              desired_mode = CONFIGURATION_5;
              mode_msg = I2C_CONFIGURATION_5;
            break;
            case iWinbutton16:
              desired_mode = CONFIGURATION_6;
              mode_msg = I2C_CONFIGURATION_6;
            break;
            case iWinbutton17:
              desired_mode = CONFIGURATION_7;
              mode_msg = I2C_CONFIGURATION_7;
            break;
            case iWinbutton18:
              desired_mode = CONFIGURATION_8;
              mode_msg = I2C_CONFIGURATION_8;
            break;
            case iWinbutton19:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
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

    case BOARDING:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage7) ;
        Display.img_Show(hndl,iStatictext6) ;
        Display.img_ClearAttributes(hndl, iWinbutton20, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton20, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton20);
        Display.img_ClearAttributes(hndl, iWinbutton21, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton21, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton21);
        Display.img_ClearAttributes(hndl, iWinbutton22, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton22, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton22);
        current_state = BOARDING_READY;
        displayed = true;
      }
    break;

    case BOARDING_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton20 && n <= iWinbutton22) {
          for (i = iWinbutton22; i <= iWinbutton22; i++) {
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
        if (n >= iWinbutton20 && n <= iWinbutton22) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton20:
              desired_mode = CONFIGURATION_9;
              mode_msg = I2C_CONFIGURATION_9;
            break;
            case iWinbutton21:
              desired_mode = CONFIGURATION_10;
              mode_msg = I2C_CONFIGURATION_10;
            break;
            case iWinbutton22:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
            // define the back button action here
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

    case ESCAVATORE:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage8) ;
        Display.img_Show(hndl,iStatictext7) ;
        Display.img_ClearAttributes(hndl, iWinbutton23, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton23, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton23);
        Display.img_ClearAttributes(hndl, iWinbutton24, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton24, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton24);
        current_state = ESCAVATORE_READY;
        displayed = true;
      }
    break;

    case ESCAVATORE_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton23 && n <= iWinbutton24) {
          for (i = iWinbutton24; i <= iWinbutton24; i++) {
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
        if (n >= iWinbutton23 && n <= iWinbutton24) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton23:
              desired_mode = CONFIGURATION_11;
              mode_msg = I2C_CONFIGURATION_11;
            break;
            case iWinbutton24:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
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

    case AEROPLANO:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage9) ;
        Display.img_Show(hndl,iStatictext8) ;
        Display.img_ClearAttributes(hndl, iWinbutton25, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton25, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton25) ;
        Display.img_ClearAttributes(hndl, iWinbutton26, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton26, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton26) ;
        Display.img_ClearAttributes(hndl, iWinbutton27, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton27, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton27);
        Display.img_ClearAttributes(hndl, iWinbutton28, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton28, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton28);
        Display.img_ClearAttributes(hndl, iWinbutton29, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton29, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton29);
        Display.img_ClearAttributes(hndl, iWinbutton30, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton30, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton30);
        current_state = AEROPLANO_READY;
        displayed = true;
      }
    break;

    case AEROPLANO_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton25 && n <= iWinbutton30) {
          for (i = iWinbutton30; i <= iWinbutton30; i++) {
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
        if (n >= iWinbutton25 && n <= iWinbutton30) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton25:
              desired_mode = CONFIGURATION_12;
              mode_msg = I2C_CONFIGURATION_12;
            break;
            case iWinbutton26:
              desired_mode = CONFIGURATION_13;
              mode_msg = I2C_CONFIGURATION_13;
            break;
            case iWinbutton27:
              desired_mode = CONFIGURATION_14;
              mode_msg = I2C_CONFIGURATION_14;
            break;
            case iWinbutton28:
              desired_mode = CONFIGURATION_15;
              mode_msg = I2C_CONFIGURATION_15;
            break;
            case iWinbutton29:
              desired_mode = CONFIGURATION_16;
              mode_msg = I2C_CONFIGURATION_16;
            break;
            case iWinbutton30:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
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

    case PESCA:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage10) ;
        Display.img_Show(hndl,iStatictext9) ;
        Display.img_ClearAttributes(hndl, iWinbutton31, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton31, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton31) ;
        Display.img_ClearAttributes(hndl, iWinbutton32, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton32, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton32) ;
        current_state = PESCA_READY;
        displayed = true;
      }
    break;

    case PESCA_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton31 && n <= iWinbutton32) {
          for (i = iWinbutton32; i <= iWinbutton32; i++) {
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
        if (n >= iWinbutton31 && n <= iWinbutton32) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton31:
              desired_mode = CONFIGURATION_17;
              mode_msg = I2C_CONFIGURATION_17;
            break;
            case iWinbutton32:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
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

    case SCALATA:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage11) ;
        Display.img_Show(hndl,iStatictext10) ;
        Display.img_ClearAttributes(hndl, iWinbutton33, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton33, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton33) ;
        Display.img_ClearAttributes(hndl, iWinbutton34, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton34, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton34);
        current_state = SCALATA_READY;
        displayed = true;
      }
    break;

    case SCALATA_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton33 && n <= iWinbutton34) {
          for (i = iWinbutton34; i <= iWinbutton34; i++) {
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
        if (n >= iWinbutton33 && n <= iWinbutton34) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton33:
              desired_mode = CONFIGURATION_18;
              mode_msg = I2C_CONFIGURATION_18;
            break;
            case iWinbutton34:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
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

    case DIAMANTE:
      if(!displayed) {
        Display.gfx_BGcolour(0xFFFF) ;
        Display.gfx_Cls() ;
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
        Display.img_Show(hndl,iImage12) ;
        Display.img_Show(hndl,iStatictext11) ;
        Display.img_ClearAttributes(hndl, iWinbutton35, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton35, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton35) ;
        Display.img_ClearAttributes(hndl, iWinbutton36, I_TOUCH_DISABLE);
        Display.img_SetWord(hndl, iWinbutton36, IMAGE_INDEX, 0);
        Display.img_Show(hndl,iWinbutton36) ;
        current_state = DIAMANTE_READY;
        displayed = true;
      }
    break;

    case DIAMANTE_READY:
      state = Display.touch_Get(TOUCH_STATUS);
      n = Display.img_Touched(hndl,-1);

      if (state == TOUCH_PRESSED) {
        if (n >= iWinbutton35 && n <= iWinbutton36) {
          for (i = iWinbutton36; i <= iWinbutton36; i++) {
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
        if (n >= iWinbutton35 && n <= iWinbutton36) {
          Display.img_SetWord(hndl, n, IMAGE_INDEX, 3);
          Display.img_Show(hndl, n);
          I2Cmsg mode_msg;
          switch (n) {
            case iWinbutton35:
              desired_mode = CONFIGURATION_19;
              mode_msg = I2C_CONFIGURATION_19;
            break;
            case iWinbutton36:
               desired_mode = BACK_MODE;
               mode_msg = I2C_BACK_MODE;
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

    case INITIALIZING:
      if (!displayed) {
        Display.gfx_BGcolour(0xFFFF);
        Display.gfx_Cls();
        Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);

        current_mode = desired_mode;
        if (current_mode == BACK_MODE) {
          current_state = HOMED;
        }
        else {
          Display.img_Show(hndl,iStatictext12);
          Display.img_Show(hndl,iImage13);
          //current_state = INITIALIZED; // Added just for the test}
        }
        displayed = true;
      }
    break;

    case INITIALIZED:
      Display.gfx_BGcolour(0xFFFF);
      Display.gfx_Cls();
      Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);

      // Platform Game
      if (current_mode >= CONFIGURATION_1 && current_mode <= CONFIGURATION_3) {
        Display.img_Show(hndl,iImage5);
        Display.img_Show(hndl,iStatictext4);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton10, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_1) {
          Display.img_SetWord(hndl, iWinbutton10, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton10, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton10);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton11, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_2) {
          Display.img_SetWord(hndl, iWinbutton11, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton11, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton11);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton12, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_3) {
          Display.img_SetWord(hndl, iWinbutton12, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton12, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton12);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton13, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton13, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton13, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton13);
        current_state = PLATFORM_READY;
      }

      // Giungla Game
      if (current_mode >= CONFIGURATION_4 && current_mode <= CONFIGURATION_8) {
        Display.img_Show(hndl,iImage6);
        Display.img_Show(hndl,iStatictext5);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton14, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_4) {
          Display.img_SetWord(hndl, iWinbutton14, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton14, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton14);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton15, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_5) {
          Display.img_SetWord(hndl, iWinbutton15, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton15, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton15);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton16, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_6) {
          Display.img_SetWord(hndl, iWinbutton16, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton16, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton16);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton17, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_7) {
          Display.img_SetWord(hndl, iWinbutton17, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton17, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton17);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton18, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_8) {
          Display.img_SetWord(hndl, iWinbutton18, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton18, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton18);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton19, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton19, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton19, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton19);
        current_state = GIUNGLA_READY;
      }

      // Boarding Game
      if (current_mode >= CONFIGURATION_9 && current_mode <= CONFIGURATION_10) {
        Display.img_Show(hndl,iImage7);
        Display.img_Show(hndl,iStatictext6);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton20, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_9) {
          Display.img_SetWord(hndl, iWinbutton20, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton20, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton20);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton21, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_10) {
          Display.img_SetWord(hndl, iWinbutton21, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton21, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton21);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton22, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton22, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton22, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton22);
        current_state = BOARDING_READY;
      }

      // Escavatore Game
      if (current_mode == CONFIGURATION_11) {
        Display.img_Show(hndl,iImage8);
        Display.img_Show(hndl,iStatictext7);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton23, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_11) {
          Display.img_SetWord(hndl, iWinbutton23, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton23, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton23);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton24, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton24, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton24, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton24);
        current_state = ESCAVATORE_READY;
      }

      // Aeroplano Game
      if (current_mode >= CONFIGURATION_12 && current_mode <= CONFIGURATION_16) {
        Display.img_Show(hndl,iImage9);
        Display.img_Show(hndl,iStatictext8);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton25, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_12) {
          Display.img_SetWord(hndl, iWinbutton25, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton25, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton25);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton26, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_13) {
          Display.img_SetWord(hndl, iWinbutton26, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton26, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton26);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton27, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_14) {
          Display.img_SetWord(hndl, iWinbutton27, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton27, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton27);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton28, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_15) {
          Display.img_SetWord(hndl, iWinbutton28, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton28, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton28);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton29, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_16) {
          Display.img_SetWord(hndl, iWinbutton29, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton29, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton29);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton30, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton30, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton30, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton30);
        current_state = AEROPLANO_READY;
      }

       // Pesca Game
      if (current_mode == CONFIGURATION_17) {
        Display.img_Show(hndl,iImage10);
        Display.img_Show(hndl,iStatictext9);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton31, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_17) {
          Display.img_SetWord(hndl, iWinbutton31, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton31, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton31);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton32, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton32, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton32, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton32);
        current_state = PESCA_READY;
      }

      // Scalata Game
      if (current_mode == CONFIGURATION_18) {
        Display.img_Show(hndl,iImage11);
        Display.img_Show(hndl,iStatictext10);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton33, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_18) {
          Display.img_SetWord(hndl, iWinbutton33, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton33, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton33);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton34, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton34, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton34, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton34);
        current_state = SCALATA_READY;
      }

      // Diamante Game
      if (current_mode == CONFIGURATION_19) {
        Display.img_Show(hndl,iImage12);
        Display.img_Show(hndl,iStatictext11);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton35, I_TOUCH_DISABLE);
        if (current_mode == CONFIGURATION_19) {
          Display.img_SetWord(hndl, iWinbutton35, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton35, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton35);
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% //
        Display.img_ClearAttributes(hndl, iWinbutton36, I_TOUCH_DISABLE);
        if (current_mode == BACK_MODE) {
          Display.img_SetWord(hndl, iWinbutton36, IMAGE_INDEX, 3);
        }
        else {
           Display.img_SetWord(hndl, iWinbutton36, IMAGE_INDEX, 0);
        }
        Display.img_Show(hndl,iWinbutton36);
        current_state = DIAMANTE_READY;
      }

    break;

    case PHICUBE_ERROR:
      Display.gfx_BGcolour(0xFFFF);
      Display.gfx_Cls();
      Display.img_SetAttributes(hndl, -1, I_TOUCH_DISABLE);
      delay(500);
      Display.img_Show(hndl,iStatictext3);
      Display.img_Show(hndl,iImage4);
      delay(500);
    break;

  }
}


void loop()
{
  stateMachine();
}


