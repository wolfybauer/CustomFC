

// MODES
enum RC_MODES { MODE_STANDBY, MODE_ARM, MODE_PID, MODE_ESC, MODE_PAIR };
uint8_t pid_cursor_pos = 0;


void draw_STANDBY() {
  
  display.setTextSize(1);             
  display.setTextColor(WHITE);
    
  // print L stick position
  display.setCursor(20, 0);             
  display.print("LX:");
  display.print(data_out[L_xpos]);
  display.setCursor(20, 16);             
  if(!LINEAR_ON) {
    display.print("LY:");
    display.print(data_out[L_ypos]);
  } else {
    display.print("TH:");
    display.print(data_out[P_pos]);
  }

  // print R stick position
  display.setCursor(61, 0);             
  display.print("RX:");
  display.print(data_out[R_xpos]);
  display.setCursor(61, 16);             
  display.print("RY:");
  display.print(data_out[R_ypos]);

  // draw slider
  int slider_gfx_ypos = map(data_out[P_pos], 0, 255, 60, 0);
  display.drawLine(5, 0, 5, 63, WHITE);
  display.fillRect(0, slider_gfx_ypos, 10, 4, WHITE);

  // draw stick buttons
  if(!L_tog) display.drawCircle(38, 45, 10, WHITE);     // draw L button
  else display.fillCircle(38, 45, 10, WHITE);
  if(!R_tog) display.drawCircle(68, 45,  10, WHITE);    // draw R button
  else display.fillCircle(68, 45, 10, WHITE);

  // print standby status
  display.setCursor(87, 56);
  if(L_tog || R_tog) {
    display.print("WARMUP!");
  } else {
    display.print("STANDBY");
  }
}

void draw_ARM() {
  display.setTextSize(1);             
  display.setTextColor(WHITE);
    
  // print L stick position
  display.setCursor(20, 0);             
  display.print("LX:");
  display.print(data_out[L_xpos]);
  display.setCursor(20, 16);             
  if(!LINEAR_ON) {
    display.print("LY:");
    display.print(data_out[L_ypos]);
  } else {
    display.print("TH:");
    display.print(data_out[P_pos]);
  }

  // print R stick position
  display.setCursor(61, 0);             
  display.print("RX:");
  display.print(data_out[R_xpos]);
  display.setCursor(61, 16);
  display.print("RY:");
  display.print(data_out[R_ypos]);
  
  // draw slider
  int slider_gfx_ypos = map(data_out[P_pos], 0, 255, 60, 0);
  display.drawLine(5, 0, 5, 63, WHITE);
  display.fillRect(0, slider_gfx_ypos, 10, 4, WHITE);

  // draw stick buttons
  if(!L_tog) display.drawCircle(38, 45, 10, WHITE);     // draw L button
  else display.fillCircle(38, 45, 10, WHITE);
  if(!R_tog) display.drawCircle(68, 45,  10, WHITE);    // draw R button
  else display.fillCircle(68, 45, 10, WHITE);

  // print standby status
  display.setCursor(87, 56);             
  display.print("ARMED!!!");
}

void draw_PID() {
  // print mode type
  display.setCursor(0, 0);             
  display.print("PID MODE");

  // do the rest of the stuff
  display.setCursor(15, 15);             
  display.print("Kp:");
  display.print(Kp);

  display.setCursor(15, 25);             
  display.print("Ki:");
  display.print(Ki);

  display.setCursor(15, 35);             
  display.print("Kd:");
  display.print(Kd);

  switch(pid_cursor_pos) {
    case 0:
      display.fillCircle(6, 16, 2, WHITE);
      break;
    case 1:
      display.fillCircle(6, 26, 2, WHITE);
      break;
    case 2:
      display.fillCircle(6, 36, 2, WHITE);
      break;
    default:
      pid_cursor_pos = 0;
      break;
  }
}

void draw_ESC() {
  // print mode type
  display.setCursor(0, 0);             
  display.print("ESC MODE");
  
  // draw slider
  int slider_gfx_ypos = map(data_out[P_pos], 0, 255, 60, 0);
  display.drawLine(65, 0, 65, 63, WHITE);
  display.fillRect(60, slider_gfx_ypos, 10, 4, WHITE);

  // print throttle val
  display.setCursor(5, 30);             
  display.print("THROT:");
  display.print(data_out[P_pos]);
}

void draw_PAIR() {
  // print mode type
  display.setCursor(0, 0);             
  display.print("PAIR MODE");

  // do the rest of the stuff
}
