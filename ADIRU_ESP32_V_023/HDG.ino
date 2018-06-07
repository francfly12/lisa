// ------------ variables necessaires pour afficher le cap(HDG) -----------------

   int intcap;// Valeur en integer du cap 
   char bufferCAP [20],bufferN [5];
   int xPos = 0;   // position x de l affichage
int yPos = 0;   // position y de l affichage
int poscap = 0;  // position de l affichage du cap, selon 1,2 ou 3 digits
int posn,pose,posw,poss,pos3,pos6,pos12,pos15,pos21,pos24,pos30,pos33; // ce sont toutes les positions d affichage des symboles
float Pi = 3.14159;   
float cap,newcap,oldcap; 
int affdegres, affpix;  // utilise pour afficher les barres tous les dix degres
int bar =0;       // utilise pour compter l affichage des 7 barres tous les 10 degres  
int midcap ; // pour affichage a 0,5 degres pres
int bug ; // pour affichage du dot supplementaire

void displayHeading (void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX HEADING
   u8g2.drawStr( 0, 0, "Cap"); // put "hdg" if you prefer
   // note : yaw is software, Yaw is hardware
  
   
 newcap=Yaw ; // et C est tout !, le calcul et filtrages sont deja faits
// casting du float en integer
    intcap=int(newcap);//  transformer le cap en integer 

// calcul pour affichage car il est de 2 pixels pour un degre: Il faut trouver un moyen pour le decaler tous les 0,5 degres  
   midcap= int(newcap+0.5) ;
   bug = 1;//digitalWrite (ledgreen, HIGH);
   if (midcap==intcap) {bug=0;}// digitalWrite (ledgreen, LOW);}
 /*   Serial.print ("yaw = "); Serial.println (yaw);
    Serial.print ("intcap = "); Serial.println (intcap);
    Serial.print ("midcap = "); Serial.println (midcap);
     Serial.print ("bug = "); Serial.println (bug);
     */
// delay (1000);
      
  // dessin partie du centre lettres et symboles   
   u8g2.drawBox(0,62,127,2); // affichage barre bas 1 pixel 
 //  u8g2.setFont(u8g2_font_ncenB14_tr);  
// affichage des symboles   - ne sont affiches que dans certains cas. Ils sont affiches avec un decalage qui depend du jeu de caracteres
  if ((intcap < 75 ) or (intcap > 280)) {posn=2*(intcap)+52 + bug ;if (posn >360) posn=posn-720; u8g2.drawStr( posn +8,35,"N");};  // si visibilite de N, afficher
//  affichage 3 normal
  if ((intcap > -45 ) and (intcap < 105)) {pos3=2*(intcap)-8 + bug;if (pos3 >360) pos3=pos3-720; u8g2.drawStr( pos3 +8,35,"3");};  //
//  affichage 3 a gauche du N
  if ((intcap > 315 )) {pos3=2*(intcap)-8 + bug ;if (pos3 >360) pos3=pos3-720; u8g2.drawStr( pos3+8,35,"3");};  //
//  affichage 6
  if ((intcap > -15 ) and (intcap < 135)) {pos6=2*(intcap)-68 + bug;if (pos6 >360) pos6=pos6-720; u8g2.drawStr( pos6+8,35,"6");};  //
//  affichage W
  if ((intcap > 15 ) and (intcap < 165)) {posw=2*(intcap)-128 + bug;if (posw >360) posw=posw-720; u8g2.drawStr( posw +9,35,"W");};  // si visibilite de W, afficher
//  affichage 12
  if ((intcap > 45 ) and (intcap < 195)) {pos12=2*(intcap)-188 + bug;if (pos12 >360) pos12=pos12-720; u8g2.drawStr( pos12 +5,35,"12");};  //
//  affichage 15
  if ((intcap > 75 ) and (intcap < 225)) {pos15=2*(intcap)-248 + bug;if (pos15 >360) pos15=pos6-720; u8g2.drawStr( pos15 +5,35,"15");};  //
//  affichage S
  if ((intcap > 105 ) and (intcap < 225)) {poss=2*(intcap)-308 + bug;if (poss >360) poss=poss-720; u8g2.drawStr( poss +8,35,"S");};  // si visibilite de S, afficher
  
//  affichage 21
  if ((intcap > 135 ) and (intcap < 285)) {pos21=2*(intcap)-368 + bug;if (pos21 >360) pos21=pos21-720; u8g2.drawStr( pos21 +4,35,"21");};  //
//  affichage 24
  if ((intcap > 165 ) and (intcap < 315)) {pos24=2*(intcap)-428 + bug;if (pos24 >360) pos24=pos24-720; u8g2.drawStr( pos24 +4,35,"24");};  //
//  affichage E
  if ((intcap > 195 ) and (intcap < 340)) {pose=2*(intcap)-488 + bug;if (pose >360) pose=pose-720; u8g2.drawStr( pose +9,35,"E");};  // si visibilite de E, afficher
//  affichage 30
  if ((intcap > 225 ) and (intcap < 375)) {pos30=2*(intcap)-548 + bug;if (pos30 >360) pos30=pos30-720; u8g2.drawStr( pos30 +4,35,"30");};  //
//  affichage 33 a droite du N
  if (intcap < 15) {pos33=2*(intcap)+112 + bug; u8g2.drawStr( pos33-4,35,"33");};  // cas du 33 affiche a droite du cadran
//  affichage 33 normal
  if ((intcap > 255 ) and (intcap < 405)) {pos33=2*(intcap)-608 + bug;if (pos33 >360) pos33=pos33-720; u8g2.drawStr( pos33 +4,35,"33");};  //  
   

  affdegres = (intcap %10); // cap modulo 10, devient une valeur de 0 a 9
  for (int bar = -1; bar < 8 ; bar++) {     //  8 barres maxi sur l affichage
//   u8g2.drawBox(((bar)*20)+affdegres+1,1,((bar)*20)+affdegres+3,5); // affichage boite tous les 10 degres, soit 20 pixels
   affpix = (affdegres*2)+(20*bar) + bug ;// valeur de decalage affichage
   u8g2.drawBox(affpix+4,57,3,7);  // boite 8 px X 3 px
   u8g2.drawBox(affpix+15,53,1,11);  // boite 11 px X 1 px 
    }
   u8g2.drawBox(65,35,1,28);  //  barre centrale    
   
  // affichage valeur 
  // u8g2.setFont(u8g2_font_ncenB14_tr);
  //u8g2.setFont(u8g2_font_fub20_tn); // 20 px
  u8g2.setFont(u8g2_font_helvB24_te); // 24 px
  if (intcap==0) {intcap = 360 ;} // discussion d aviateurs ...
  if (intcap<=10) {u8g2.setCursor(56, 5);}
  if (intcap>=10) {u8g2.setCursor(47, 5);}
  if (intcap>=100) {u8g2.setCursor(38, 2);}
   u8g2.print(intcap);// valeur affichee au centre
   
   
   
   
   
   
   }
