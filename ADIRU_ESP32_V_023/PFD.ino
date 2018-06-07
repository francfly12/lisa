//   float airspeed ; // airspeed
// note : float pitch, yaw, roll,(software), Yaw, Pitch, Roll;(hardware)
//int ptch, rol; // integers, to ease computation and display
// int affalt,altpix,altpix10; // for altitude bars display
// int counter=1;  // for test alt
// int counter1=1;  // for test spd
// int spdpix=1;
//int cnt=0; // for spd test
//int airspd; // int for spd test
//int vmo = 240; // for vmo symbol, max speed , here 240 km/h
//int vstall=65; // for stall symbol, here 65 km/h
void displayPFD () { // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX PFD
  // lissage altitude
  boolean flip=true ;
 if (newalt==0) {newalt = altitude;} // C est pour le demarrage
 oldalt = newalt ;
 newalt = altitude ;
 newalt=oldalt + ((newalt-oldalt)/5);  // lissage au 1/5 eme
 // calculs pitch, roll
 ptch = (roll * 0.6); // casts pitch in an integer, changed pitch / roll due to sensor mounting
 rol = 64 * (tan(pitch/57.3)) ; // same for roll + calcul tangente + rad = deg/57,295
 
  // u8g2.drawFrame (0,0,128,64); // cadre maxi, just to check
  // u8g2.drawLine (63,0,63,63); // barre verticale
  // u8g2.drawLine (0,32,127,32); // barre horizontale

  u8g2.drawRBox (34,0,60,60,15);  // cadre+ fond attitude
  u8g2.setDrawColor(0); // dessine en noir
  // dessine en noir la partie basse, la terre, le ciel reste bleu/blanc
  u8g2.drawTriangle(0,30 +ptch+rol, 127,30+ptch-rol,64+rol,90);  // dessine un triangle noir en bas
  // a modifier si on veut que ca fonctionne en vol dos, pour le moment, ca gaze jusqu a 60 degres pitch et roll
  u8g2.setDrawColor(1); // dessine en blanc (normal)
  u8g2.drawRFrame (34,0,60,60,15);  // cadre arrondi attitude
  u8g2.setFont(u8g2_font_8x13B_mr);
  
  
  // affichage altitude ---------------------------------------------
  //
  u8g2.drawLine (128,0,128,63); // barre verticale a droite
  altpix = (((int(newalt))/2) %10); // valeur de 0 a 9 pour afficher des barres sur l echelle d altitude 
  altpix10 = (((int(newalt))/2) %100); // every 100ft, double bar
  // Serial.print ("altpix10 = "); Serial.println (altpix10);
  // 
  // afficher 5 barres + barres 100 ft
  u8g2.drawLine (123,   altpix,128,   altpix); // barre horizontale 1
  u8g2.drawLine (123,10 + altpix,128,10 + altpix); // barre horizontale 2
  u8g2.drawLine (123,20 + altpix,128,20 + altpix); // barre horizontale 3
  u8g2.drawLine (123,30 + altpix,128,30 + altpix); // barre horizontale 4
  u8g2.drawLine (123,40 + altpix,128,40 + altpix); // barre horizontale 5
  u8g2.drawLine (123,50 + altpix,128,50 + altpix); // barre horizontale 6
  u8g2.drawBox (120,-21 + altpix10,8,3); // thick bar
  u8g2.drawLine (97,-20 + altpix10,120,-20 + altpix10); // long bar
  u8g2.drawBox (120,-71 + altpix10,8,3); // alternate thick bar
  u8g2.drawLine (97,-70 + altpix10,120,-70 + altpix10); // alternate long bar
  u8g2.drawBox (120,29 + altpix10,8,3); // alternate 2 thick bar
  u8g2.drawLine (97,30 + altpix10,120, 30 + altpix10); // alternate 2 long bar
  u8g2.drawLine (127,0,127,55); // barre vertical a droite
  // u8g2.drawLine (96,0,96,55); // deuxieme barre vertical 
  u8g2.setCursor (105,26); // display alt
  if (newalt >= 1000) {u8g2.setCursor (98,26);} //
  u8g2.print (int(newalt));
  u8g2.setCursor(98,52); // display QNH 
  u8g2.print (qnh);

  // affichage Airspeed -----------------------------------------
  //  
  u8g2.drawLine (0,0,0,63); // barre verticale a gauche
  spdpix = ((int(airspd)) %10); // valeur de 0 a 9 pour afficher des barres sur l echelle d altitude
  // afficher des barres tous les 10 km/h
  // airspd = int(airspeed); // ce sera plus rapide
  // airspd already computed in main loop
  //Serial.print ("airspeed = "); Serial.println (airspeed);
  //Serial.print ("airspd = "); Serial.println (airspd);
  //Serial.print ("spdpix = "); Serial.println (spdpix);
  
  u8g2.drawLine (1,spdpix,8,spdpix); // barre horizontale 1
  u8g2.drawLine (1,spdpix+10,8,spdpix+10); // barre horizontale 2
  u8g2.drawLine (1,spdpix+20,8,spdpix+20); // barre horizontale 3
  u8g2.drawLine (1,spdpix+30,8,spdpix+30); // barre horizontale 4
  if (airspd>=10) {u8g2.drawLine (1,spdpix+40,8,spdpix+40);} // barre horizontale 5
  if (airspd>=20) {u8g2.drawLine (1,spdpix+50,8,spdpix+50);} // barre horizontale 6
  if (airspd>=30) {u8g2.drawLine (1,spdpix+60,8,spdpix+60);} // barre horizontale 7

  if (airspd<=(vstall+32)) {u8g2.drawBox (2,airspd-vstall+27,6,vstall);} // stall bar
  if (airspd >(vmo-27)) {u8g2.drawBox (2,0,6,airspd-vmo+27);} // Vmo bar
  
  u8g2.setCursor(3,26);//  airspeed 
  u8g2.print(int(airspd));  
    
  u8g2.setCursor(53,52);// display HDG ----------------------------------------
  u8g2.print(int(Yaw));  

  // Affichage symboles fixes ------------------------------------------
  u8g2.setDrawColor(2); // dessine en XOR - inverse -
  u8g2.drawCircle(64,32,4);
  u8g2.drawLine (64,22,64,42); // barre verticale centrale
  u8g2.drawLine (53,32,75,32); // barre horizontale au centre
  u8g2.drawLine (62,17,66,17); // barre horizontale haut 
  u8g2.drawLine (62,47,66,47); // barre horizontale bas
  u8g2.setDrawColor(1); // dessine en blanc (normal)
  }
