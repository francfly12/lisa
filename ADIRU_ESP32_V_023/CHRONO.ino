int minutes,seconds ;// 
unsigned long actualtime, resettime =0; // 

void displayChrono (void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX CHRONO
  u8g2.drawStr( 0, 0, "Chrono");
actualtime = millis()+resettime;
  
 seconds = int((actualtime/1000)%60);
 minutes = int ((actualtime/60000)%60);

  u8g2.drawStr( 103, 38, "Sec");
  if (minutes >0) {u8g2.drawStr( 35,38, "Min");}  
  u8g2.setFont(u8g2_font_helvB24_te); // 24 px
  if (minutes >0) {u8g2.setCursor(15,25);}
  if (minutes >9) {u8g2.setCursor(1,25);}
 if (minutes>0) {u8g2.print(minutes);}// min
  
  u8g2.setCursor(80,25);
  if (seconds>9) {u8g2.setCursor(65,25);}
  u8g2.print(seconds);// 
  u8g2.setFont(u8g2_font_ncenB14_tr);
  
  }
  
