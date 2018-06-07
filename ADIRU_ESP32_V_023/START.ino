

void displayStartup (void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX START
  u8g2.drawFrame(0,0,128,64);
  u8g2.drawStr( 1,3, "START");
  u8g2.setFont(u8g2_font_6x10_tf);
  if (startfault == 0) u8g2.drawStr( 2, 18, "Sensor connected");
  if (startfault == 4) u8g2.drawStr( 2, 30, "Error");
  if (startfault == 5) u8g2.drawStr( 2, 30, "No I2C device found");
  if (startfault == 0) u8g2.drawStr( 2, 30, "Initialisation");
   } 
