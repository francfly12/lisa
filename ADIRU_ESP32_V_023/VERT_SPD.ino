void displayVerticalspeed (void) { // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX VERTICAL SPEED
   // Air speed computation is performed in main loop
   //  
  int vbar=0; // pour calcul de la position du pointeur
  u8g2.drawStr( 0, 0, "Vert Spd");
  
   u8g2.drawStr( 10,45, "Ft/min"); 
  u8g2.setFont(u8g2_font_helvB24_te); // 24 px
  u8g2.setCursor(20,16);
  if (vspd >= 100) {u8g2.setCursor(5,16);} // decale l affichage a gauche si > 100
  if (vspd < 0) {u8g2.setCursor(0,16);} // decale l affichage a gauche si < 0
  u8g2.print(vspd);
  // calcule l extremite du pointeur
  if (vspd>1500) {vspd=1500;}// sinon ca bugge
  if (vspd>0) {vbar = 32-(2.5*(sqrt(vspd/10)));}
  if (vspd==0) {vbar = 32;}
  if (vspd<0) {vbar = 32+(2.5*(sqrt(-vspd/10)));}
  u8g2.drawDisc (127,32,8);
  u8g2.setDrawColor(2); // dessine en XOR - inverse -
  u8g2.drawLine (90,vbar,127,32); // barre horizontale bas
  u8g2.drawLine (90,vbar+1,127,33); //
  u8g2.drawLine (90,vbar-1,127,31); // 
  u8g2.drawLine (85,0,85,63); // vertical bar
  u8g2.drawLine (84,0,84,63); // vertical bar
  u8g2.drawBox (86,32,3,3); // center dot
  u8g2.drawBox (86,10,3,3); // upper dot
  u8g2.drawBox (86,0,3,3); // upper dot
  u8g2.drawBox (86,52,3,3); // lower dot
  u8g2.drawBox (86,61,3,3); // lower dot
  u8g2.setDrawColor(1); // dessine en blanc (normal)


   
  }
 
