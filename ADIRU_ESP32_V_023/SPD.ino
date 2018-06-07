

void displaySpeed (void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX AIRSPEED
  // Airspeed computation performed i main loop
  //
  /* 
    //Serial.print ("valuepitot = "); Serial.println (valuepitot); // debug
    // Serial.print ("pitotpress = "); Serial.println (pitotpress); // debug
    //Serial.print ("Air density = "); Serial.println (ro); // debug
    //Serial.print ("Temperature = "); Serial.println (temperature); // debug  
    //Serial.print ("((2 * pitotpress)/ ro) = "); Serial.println ((2 * pitotpress)/ ro); // debug
    //Serial.print ("sqrt((2 * pitotpress)/ ro) = "); Serial.println (sqrt((2 * pitotpress)/ ro)); // debug
    
    Serial.print ("airspeed = "); Serial.println (airspeed); // debug
    Serial.print ("oldairspd = "); Serial.println (oldairspd); // debug
    Serial.print ("newairspd = "); Serial.println (newairspd); // debug
    Serial.print ("deltaspd= "); Serial.println (deltaspd); // debug
    Serial.print ("fsa = "); Serial.println (fsa); // debug
    Serial.print ("fsa/20 = "); Serial.println (fsa/20); // debug
    Serial.print ("airspd = "); Serial.println (airspd); // debug
    Serial.println ();
    // delay (1000);  
  */  
 
   
      u8g2.drawStr( 0, 0, "Airspeed   CAS");   
   u8g2.drawStr( 70,25, "Km/h"); 
   u8g2.setFont(u8g2_font_helvB24_te); // 24 px
   u8g2.setCursor(20,16);
   if (airspd >= 100) {u8g2.setCursor(5,16);} // decale l affichage a gauche si > 100
   u8g2.print(airspd);
  // u8g2.setFont(u8g2_font_ncenB14_tr);
  
  // graphical scale at the bottom of display
  u8g2.setDrawColor(2); // dessine en XOR - inverse -
  // u8g2.drawCircle(64,32,4);
  u8g2.drawLine (0,63,127,63); // barre horizontale bas
  u8g2.drawLine (0,52,0,67); // barre gauche
  u8g2.drawLine (127,52,127,63); // barre droite
  u8g2.drawLine (50,52,50,63); // barre 100
  u8g2.drawLine (75,52,75,63); // barre 150
  u8g2.drawLine (100,52,100,63); // barre 200
  u8g2.drawBox (0,55,airspd/2,8);  // speed box
  if (airspd > vstall) {u8g2.drawBox (0,54,vstall/2,8);}  // stall box
   if ((airspd <= vstall)&&(airspd>4)) {u8g2.drawBox (0,54,(airspd/2)-2,8);}
  // u8g2.setCursor (53,(airspd/2)); 
  // u8g2.drawDisc ((airspd/2),53,3); 
  u8g2.setDrawColor(1); // dessine en blanc (normal)
    
  }
  
