
const char *menu_list =
  "Exit\n"              //1
  "PFD\n"               //2
  "Heading\n"           //3 
  "Altitude\n"          //4
  "Speed\n"             //5
  "Vertical speed\n"    //6
  "Temperature\n"       //7
  "Chrono\n"            //8
  "Compas calibration\n"//9
  "Timer reset\n"       //10
  "Full reset\0"        //11
  ;
uint8_t current_selection = 0; // selection dans le menu principal
uint8_t default_selection ; // selection dans le menu default

void displayTest (void) {  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX TEST & CAL

  u8g2.setFont(u8g2_font_6x10_tf);// default charset
  u8g2.setFontRefHeightAll();    /* this will add some extra space for the text inside the buttons */

  current_selection = u8g2.userInterfaceSelectionList ("Calibration & test", current_selection, menu_list);

  Serial.print ("current_selection "); Serial.println (current_selection);
    
  switch (current_selection) {
    case 0 :  ;
    case 1 : /* Exit*/modeaffichage = 6; writetoEEPROM();break;
    case 2 : /* PFD*/  u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 0; writetoEEPROM();break;
    case 3 : /* Hdg*/   u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 1; writetoEEPROM();break;
    case 4 : /* Alt*/  u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 2; writetoEEPROM();break;
    case 5 : /* Speed*/  u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 3; writetoEEPROM();break;
    case 6 : /* V/S */  u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 4; writetoEEPROM();break;
    case 7 : /* Temp*/  u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 5; writetoEEPROM();break;
    case 8 : /* Chrono*/  u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 6; writetoEEPROM();break;
    case 9 : /* Compas Calibr*/  u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");modeaffichage = 7; writetoEEPROM();break;
    case 10 : /* Timer reset*/    u8g2.userInterfaceMessage( "Selection:",  u8x8_GetStringLineStart(current_selection-1, menu_list ),  "",  " ok \n cancel ");resettime = -(millis());modeaffichage=6; writetoEEPROM();break;
    case 11 : /* Reset*/ setup(); break ;

    
 Serial.print ("default_selection "); Serial.println (default_selection) ;  
    //  case 2 : u8g2.userInterfaceMessage( "Selection:",u8x8_GetStringLineStart(current_selection - 1, menu1_list ),""," ok \n cancel\n");

  }

}

void writetoEEPROM() {

  EEPROM.writeByte(0, defaultaffichage); // write new value

}
/* for info :
  enum modeaffichage {
  PFD, // 0
  Heading,        // 1
  Altitude,       // 2
  Speed,          // 3
  Verticalspeed,  // 4
  Temp,           // 5
  Chrono,         // 6
  Test,           // 7
  Startup         // 8

  attachInterrupt(digitalPinToInterrupt(interruptPinR), handleInterruptR, CHANGE);  // declarations
  attachInterrupt(digitalPinToInterrupt(interruptPinL), handleInterruptL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinDATA), updateEncodeur, CHANGE); // Interrupt data
  attachInterrupt(digitalPinToInterrupt(interruptPinCLOCK), updateEncodeur, CHANGE); // Interrupt clock
*/
