int temp;

void displayTemp (void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX TEMP
  u8g2.drawStr( 0, 0, "Temperature");
   // lissage valeur possible ici
   temp= int(temperature);
 
  u8g2.drawStr( 90, 35, "°C");  
  u8g2.setFont(u8g2_font_helvB24_te); // 24 px
  u8g2.setCursor(30,25);
   u8g2.print(temp);//  
  u8g2.setFont(u8g2_font_ncenB14_tr);
  }
