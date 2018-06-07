
// float newalt, oldalt ; // pour lissage des valeurs - attention aux floats

void displayAltitude (void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX ALTITUDE
  u8g2.drawStr( 0,0, "Alt");
  u8g2.setCursor(0,52);
  u8g2.print ("QNH");

  // affichage valeur
  
  u8g2.setFont(u8g2_font_helvB24_te); // 24 px for alt
  u8g2.setCursor(20,15);
  if (altitude >= 1000) {u8g2.setCursor(5,15);} // decale l affichage a gauche si > 1000 ft
  u8g2.print(int(newalt));
  u8g2.drawStr( 90,15, "ft"); 

  // qnh display
  int a,b;  // integer and decimal part of qnh
  u8g2.setFont(u8g2_font_ncenB14_tr); // font for qnh
  a= qnh ; // casting the float into an integer
  u8g2.setCursor(40,48);
  u8g2.print (a);
  
  u8g2.setFont(u8g2_font_ncenB10_tr);  // font for qnh decimal
  b= qnh*10 - (a*10);
  u8g2.setCursor(88,51);
  u8g2.print(b);  // print one decimal 
  // Serial.print("    decimalqnh = ");Serial.println(b); // for test

  
  }
  
