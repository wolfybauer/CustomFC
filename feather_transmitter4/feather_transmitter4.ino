// OK I am rewriting this Feather 32u4 LoRa simple RC controller code.
// 1. clean up in general, rename buttons
// 2. implement distinct modes for PID Tuning, and Direct Throttle Output (0-255)

//#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce2.h>
#include <EEPROMAnything.h>
#include "util.h"
#include "util_modes.h"


uint8_t current_mode = MODE_STANDBY;

void setup() {
  full_init_routine();
}

void loop() {

  // update the timers
  curr_data_time = micros();
  curr_draw_time = micros();

  // send LoRa packet
  if(current_mode == MODE_PID) {
    sendPID();
  } else {
    sendData(data_out);
  }
  
  // data loop, set by DATA_CYCLE_TIME
  if(curr_data_time - prev_data_time >= DATA_CYCLE_TIME) {
    getPotData();
    getButtonData();
    if(current_mode == MODE_PID) {
      printSerialPID();
    } else {
      printSerialData();
    }
    prev_data_time = curr_data_time;
  }

  // draw loop, set by DRAW_CYCLE_TIME
  if(curr_draw_time - prev_draw_time >= DRAW_CYCLE_TIME) {
    display.clearDisplay();
    screenStuff();
    display.display();
    prev_draw_time = curr_draw_time;
  }
}

void getPotData() {
  
  // get LX, RX, RY
  data_out[L_xpos] = 255 - map(analogRead(L_X_PIN), 0, 1023, 0, 255);
  uint8_t LY = map(analogRead(L_Y_PIN), 0, 1023, 0, 255);
  data_out[R_xpos] = map(analogRead(R_X_PIN), 0, 1023, 0, 255);
  data_out[R_ypos] = 255 - map(analogRead(R_Y_PIN), 0, 1023, 0, 255);
  // set 0 if within range
  if(data_out[L_xpos] > 114 && data_out[L_xpos] < 140) data_out[L_xpos] = 127;
  if(data_out[R_xpos] > 114 && data_out[R_xpos] < 140) data_out[R_xpos] = 127;
  if(data_out[R_ypos] > 114 && data_out[R_ypos] < 140) data_out[R_ypos] = 127;
  //if(data_out[P_pos] < 13) data_out[P_pos] = 0;

  // get LY OR SLIDER data
  if(LINEAR_ON) {
    // use LY for slider out
    if(LY > 140) {
      if(data_out[P_pos] <= (255 - MOVE_SPEED)) {
        data_out[P_pos] += MOVE_SPEED;
      }
    }
    if(LY < 114) {
      if(data_out[P_pos] >= (0 + MOVE_SPEED)) {
        data_out[P_pos] -= MOVE_SPEED;
      }
    }
  } else {
    // use slider for slider out
    data_out[P_pos]  = 255 - map(analogRead(POT_PIN), 0, 1023, 0, 255);
    data_out[L_ypos] = LY;
    if(data_out[L_ypos] > 114 && data_out[L_ypos] < 140) data_out[L_ypos] = 127;
  }
}

void sendData(byte pk[]) {
  
  LoRa.beginPacket();
  LoRa.write(pk, PACKET_SIZE);
  LoRa.endPacket();
}

void sendPID() {
  LoRa.beginPacket();
  LoRa.write(pid_out.bytes_out, PID_PACKET_SIZE);
  LoRa.endPacket();
}

void screenStuff() {

  switch(current_mode) {
    case MODE_STANDBY:
      draw_STANDBY();
      break;
    case MODE_ARM:
      draw_ARM();
      break;
    case MODE_PID:
      draw_PID();
      break;
    case MODE_ESC:
      draw_ESC();
      break;
    case MODE_PAIR:
      draw_PAIR();
      break;
    default:
      //
      break;
  }
  
}

void getButtonData() {
  
  switch(current_mode) {

    // --- STANDBY MODE --- //
    case MODE_STANDBY:
      L_button.update();
      R_button.update();
      A_button.update();
      B_button.update();
      C_button.update();
      D_button.update();
      
      if(L_button.fell()) L_tog = !L_tog;
      if(R_button.fell()) R_tog = !R_tog;
      if(A_button.fell()) A_tog = !A_tog;
      if(B_button.fell()) B_tog = !B_tog;
      if(C_button.fell()) C_tog = !C_tog;
      if(D_button.fell()) LINEAR_ON = !LINEAR_ON;
      
      // if both stick buttons TRUE, go to arm mode
      if(A_tog) current_mode = MODE_PID;
      if(B_tog) current_mode = MODE_ESC;
      if(C_tog) current_mode = MODE_PAIR;
      
      if(L_tog && R_tog) current_mode = MODE_ARM;
      break;

    // --- ARM MODE --- //
    case MODE_ARM:
      L_button.update();
      R_button.update();
      // go back to standby mode
      if(L_button.fell()) L_tog = !L_tog;
      if(R_button.fell()) R_tog = !R_tog;
      if(!L_tog && !R_tog) current_mode = MODE_STANDBY;
      break;

    // --- PID TUNING MODE --- //
    case MODE_PID:    // A button
      B_tog = false; C_tog = false;
      A_button.update();
      B_button.update();
      C_button.update();
      D_button.update();
      // go back to standby mode
      if(A_button.fell()) A_tog = !A_tog;
      if(!A_tog) {
        // SAVE TO EEPROM HERE
        EEPROM_writeAnything(0, pid_out.data);
        current_mode = MODE_STANDBY;
      }

      if(B_button.fell()) pid_cursor_pos++;

      if(C_button.fell()) {
        switch(pid_cursor_pos) {
          case 0:
            Kp += 0.01;
            break;
          case 1:
            Ki += 0.01;
            break;
          case 2:
            Kd += 0.01;
            break;
          default:
            break;
        }
      }
      if(D_button.fell()) {
        switch(pid_cursor_pos) {
          case 0:
            Kp -= 0.01;
            break;
          case 1:
            Ki -= 0.01;
            break;
          case 2:
            Kd -= 0.01;
            break;
          default:
            break;
        }
      }

      // update PID output packet data
      pid_out.data.p_val = Kp;
      pid_out.data.i_val = Ki;
      pid_out.data.d_val = Kd;
      break;

    // --- MOTOR/ESC CALIBRATION MODE --- //
    case MODE_ESC:    // B button
      A_tog = false; C_tog = false;
      B_button.update();
      // go back to standby mode
      if(B_button.fell()) B_tog = !B_tog;
      if(!B_tog) current_mode = MODE_STANDBY;
      break;

    // --- PAIRING MODE (TODO) --- //
    case MODE_PAIR:   // C Button
      A_tog = false; B_tog = false;
      C_button.update();
      // go back to standby mode
      if(C_button.fell()) C_tog = !C_tog;
      if(!C_tog) current_mode = MODE_STANDBY;

      // do pairing stuff here
      
      break;
    default:
      break;
  }

  // update packet data from bools
  data_out[L_click] = L_tog;
  data_out[R_click] = R_tog;
  data_out[A_click] = A_tog;
  data_out[B_click] = B_tog;
  data_out[C_click] = C_tog;
  data_out[D_click] = D_tog;

  // update PID output packet data
  /*pid_out.data.p_val = Kp;
  pid_out.data.i_val = Ki;
  pid_out.data.d_val = Kd;*/
}
