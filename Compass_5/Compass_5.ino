/* Current issues:

- right now this is turning the GPS on 10 seconds before transmission and turning off
only right before transmission, however it's taking the first sample collected during
that period.  A better way would be to use a sample selected say 2 seconds before 
transmission, which would probably make things more accurate.

 */

// V5 actually seems to work!  it's doing all the transmitting I'd expect
// and transitioning between modes
// Optimizing for program memory - code commented for this reason is all labeled with 'PROGMEM'

// DANGER - pin 10 has been set as the TX for software serial, which is the light pin (because it's not used)
// at some point, the software serial library should be modified to remove the TX
// UPDATE - it doesn't seem to care that it's being used for the light actually... maybe not a problem?

// DANGER - the software serial library may cause Serial1 transmission to fail.
// This manifests as chopped packets being sent over the radio.
// The GPS should either be turned off prior to sending data (or disabling software serial?) or the transmission
// should only occur right after the reception of a GPS packet so that the line is sure to be quiet for a bit

/*
 * There are two types of mode: background mode and special mode
 * The background mode tells the system what to be doing at specific time intervals
 * in the background without any user updates necessarily.
 * The special mode is something like navigation, where an update is being shown
 * to the user.  They system can be in background and special modes simultaneously.
 * Special modes do not block the execution of background modes.
 * Only one mode of each can be active at a time.
 */

/* Change Log
 * V2: Full menu and navigation with function running
 * Starting compatibility with V2 hardware
 *
 * List of Functions:
 * battery_voltage: returns battery voltage as float
 * battery_SOC: converts float voltage to integer percentage state of charge
 * backlight: 1 turns backlight on, 0 off
 * vibrate: 1 vibrates, 0 off
 * buttons: returns byte with 4 LSBs corresponding to button states (push = 1)
 * init_lcd: initializes LCD screen settings
 * a_lcd: set the A0 pin on the LCD
 * clear_lcd: wipe everything from screen
 * full_word: print a text string
 * start_text: start sending a text string to a line on lcd
 * letter: send a single character to lcd
 * mag_setup: 1 sets up and sets to active, 0 sets up and sets to standby
 * mag: input 3 int array and updates with current mag readings
 * mag_cal_ret: retrieve mag cal coefficients from EEPROM
 * mag_scale: convert raw uncal'ed mag readings to scaled -1000 to 1000 per axis
 * GPS: turn on/off (1/0) GPS
 * GPS_receive: parse incoming GPS data
 */


#define SCL_PIN 0
#define SCL_PORT PORTD
#define SDA_PIN 1
#define SDA_PORT PORTD
#define I2C_TIMEOUT 200 // Timeout I2C after n milliseconds
#define volt_sens_turn_on 13
#define LED_backlight A1
#define LCD_CS A2
#define LCD_RST A3
#define LCD_A0 A4
#define GPS_EN 12
#define UP 6
#define DOWN 7
#define LEFT 8
#define RIGHT 9
#define VIBRATE 4
#define LIGHT 10
#define GPS_RX_PIN 11


//#include <SoftI2CMaster.h> PROGMEM
//#include <Wire.h> PROGMEM
#include <SPI.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <avr/pgmspace.h>



// ID, forward ID, backward ID, function
// IDs do not need to be the same as the index of the row
// rows will be show on display in the order they are presented here
// -1 in the forward ID column indicates an end point and that a function should be run
int menu_map[][4] = {
  {0, 0, 0, -1}, // Menu (kind of a hack - this menu array method doesn't handle the menu root well without this 'dummy' entry)
  {1, 2, 0, -1}, // Utility
  {2, -1, 1, 0}, // Cal Magnetometer
  {3, -1, 0, 1}, // Self Destruct [special mode]
  {5, 6, 0, -1}, // GPS
  {6, -1, 5, 2}, // GPS ON
  {7, -1, 5, 3}, // GPS OFF
  {4, -1, 5, 5}, // Show Coordinates Mode
  {10, -1, 5, 6}, // Set Home
  {11, -1, 5, 7}, // Home Distance Mode
  {12, -1, 5, 8}, // Single GPS Fix
  {13, -1, 5, 9}, // Transmit Location
  {8, 9, 0, -1}, // Light
  {9, -1, 8, 4}, // Toggle Light
};

byte menu_map_size = sizeof(menu_map) / sizeof(menu_map[0]);

// indices must correspond to IDs in menu map (so menu_labels[ID] = correct label)
char* menu_labels[] = {
  "--Menu--",
  "Utility",
  "Cal Magnetometer",
  "Self Destruct",
  "Show Coordinates",
  "GPS",
  "GPS ON",
  "GPS OFF",
  "Light",
  "Toggle Light",
  "Set Home",
  "Distance To Home",
  "Single GPS Fix",
  "Transmit Location"
};

byte menu_location_id = 0; // starting menu location at boot

int mag_range[6]; // storage of magnetometer calibration
int mag_values_raw[6]; // storage of most recent raw magnetometer values
int mag_values_scaled[6]; // storage of most recent mag values scaled with mag_range

byte button_states[2][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}}; // 0 is current status, 1 is last cycle's status

byte light_state = 0; // state of the flashlight LED

byte special_mode = 0; // flag for special display modes (like navigate, etc.)
// anything other than 0 overrides menu display and navigation
byte force_update_menu = 0; // setting to 1 forces a menu update on the next loop
// used when exiting a special display mode
byte run_once = 0; // when entering a special mode, run a part of it once and then don't again
// do something if this is 0, and then set to 1
// set to 0 as you exit

// GPS
byte gps_fix_active = 0; // GPS is a special background activity, and needs its own flag to know whether to run
byte first_gps_fix = 0; // sets to 1 the first time a GPS fix is made - to avoid taking action before fix is made
byte gps_in_use_special = 0; // allows special mode to prevent background modes from turning off GPS
byte gps_in_use_background = 0; // allows background modes to prevent special modes from turning off GPS
byte int_in_use_special = 0; // allows special mode to prevent background modes from turning off GPS
byte int_in_use_background = 0; // allows background modes to prevent special modes from turning off GPS

float home_coord[] = {0.0 , 0.0}; // 'home' coordinates
float current_coord[] = {0.0, 0.0}; // current location coordinates
float nav_coord[] = {0.0 , 0.0}; // coordinates currently being navigated to
String nav_name; // text of place being navigated to


char username[] = "ben-----"; // 8 characters exactly (9 bytes in the string with null)


SoftwareSerial gps_serial (GPS_RX_PIN, LIGHT); // 10 is actually the LED - hopefully this doesn't screw something up...

// The TinyGPS++ object
TinyGPSPlus gps;

// TIME
long gps_time;
long time_correction;
long current_time; // current milliseconds since midnight UTC
long last_time; // last time fix
// can use first_gps_fix to avoid acting based on values of GPS time until a GPS fix has actually been made
long current_time_raw = 0L; // raw time in milliseconds from boot
long last_time_raw = 0L; // raw time in milliseconds from boot

// SCHEDULING
long transmit_period = 30000L; // how frequently to transmit position, in milliseconds
long transmit_window = 5000L; // length of window during which to transmit position
long timeslot; // time after transmit window opens to make transmission - set every transmit cycle

byte interrupts_active = 0; // are interrupts active?  Always change this when enabling or disabling interrupts

byte background_mode = 0; // background mode to be executing
// 0: Hot Start Mode: spends the first 10 seconds with GPS on trying to get a hot fix
// 1: First Fix Mode: checks for fix every 30 seconds
// 2: Update Mode: gets fix every transmit cycle, and then transmits

void setup() {

  Serial.begin(115200); // USB serial for debugging
//  delay(5000); // allow time to open serial port for debugging
  Serial1.begin(38400); // Serial to radio
  //  gps_serial.begin(9600); // start with this disabled, and enable as necessary

  // Wire.begin(); PROGMEM
  SPI.begin();
  // i2c_init(); // for SW I2C PROGMEM

  // vibration motor
  pinMode(VIBRATE, OUTPUT);
  vibrate(0); // turn off vibration, if device was turned off while vibrating

  // backlight
  pinMode(LED_backlight, OUTPUT);
  backlight(1); // turn on backlight

  // battery voltage sense turn on
  pinMode(volt_sens_turn_on, OUTPUT);
  digitalWrite(volt_sens_turn_on, HIGH);

  //GPS
  pinMode(GPS_EN, OUTPUT);
  GPS(1); // turn on GPS
  gps_in_use_background = 1; // bg mode takes GPS

  //Light
  pinMode(LIGHT, OUTPUT);
  digitalWrite(LIGHT, HIGH);

  // LCD
  pinMode(LCD_A0, OUTPUT); // A0 pin
  digitalWrite(LCD_A0, LOW); // A0 pin
  pinMode(LCD_RST, OUTPUT); // Reset pin
  digitalWrite(LCD_RST, LOW); // Reset pin

  //BUTTONS
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(RIGHT, INPUT);
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);
  digitalWrite(LEFT, HIGH);
  digitalWrite(RIGHT, HIGH);

  delay(5);
  digitalWrite(LCD_RST, HIGH);
  delay(1);

  init_lcd();

  delay(5);

  clear_lcd();

  // mag_setup(1); // setup magnetometer PROGMEM

  mag_cal_ret(mag_range); // pull the magnetometer calibration from EEPROM

  // go to initial state on the display
  menu_display(menu_location_id, menu_map, menu_labels, menu_map_size);

  // seed random number generator with analog battery voltage
  // this will also further be randomized by the fact that people will not turn on their compasses simultaneously
  randomSeed(analogRead(A0));

}

// See here for more characters
// https://www.google.com/search?q=lcd+display+character+set+5&safe=active&espv=2&biw=1280&bih=702&source=lnms&tbm=isch&sa=X&ved=0ahUKEwjcjbiprZXNAhUYPlIKHf3ZAw4Q_AUIBigB&dpr=2#safe=active&tbm=isch&q=lcd+display+characters&imgrc=c0hQQXIgfCwsBM%3A

byte numbers[][5] = {
  {
    B00000000, B00011100, B00100010, B01000001, B00000000
  }// (
  , {
    B00000000, B01000001, B00100010, B00011100, B00000000
  } // )
  , {
    B00010100, B00001000, B00111110, B00001000, B00010100
  } // *
  , {
    B00001000, B00001000, B00111110, B00001000, B00001000
  } // +
  , {
    B00000000, B00000101, B00000110, B00000000, B00000000
  } // ,
  , {
    B00001000, B00001000, B00001000, B00001000, B00001000
  } // -
  , {
    B00000000, B00000011, B00000011, B00000000, B00000000
  } // .
  , {
    B00000010, B00000100, B00001000, B00010000, B00100000
  }// /
  , {
    B00111110, B01000101, B01001001, B01010001, B00111110
  } // 0
  , {
    B00010001, B00100001, B01111111, B00000001, B00000001
  } // 1
  , {
    B00100001, B01000011, B01000101, B01001001, B00110001
  } // 2
  , {
    B01000010, B01000001, B01010001, B01101001, B01000110
  } // 3
  , {
    B00001100, B00010100, B00100100, B01111111, B00000100
  } // 4
  , {
    B01110010, B01010001, B01010001, B01010001, B01001110
  } // 5
  , {
    B00011110, B00101001, B01001001, B01001001, B00000110
  } // 6
  , {
    B01000000, B01000111, B01001000, B01010000, B01100000
  } // 7
  , {
    B00110110, B01001001, B01001001, B01001001, B00110110
  } // 8
  , {
    B00110000, B01001001, B01001001, B01001010, B00111100
  } // 9
  , {
    B00000000, B00110110, B00110110, B00000000, B00000000
  } // :
};

const uint8_t letters[][5] = {
  {
    B00111111, B01001000, B01001000, B01001000, B00111111
  }//a
  , {
    B01111111, B01001001, B01001001, B00110110, B00000000
  }//b
  , {
    B00111110, B01000001, B01000001, B01000001, B00100010
  }//c
  , {
    B01111111, B01000001, B01000001, B00111110, B00000000
  }//d
  , {
    B01111111, B01001001, B01001001, B01001001, B01000001
  }//e
  , {
    B01111111, B01001000, B01001000, B01001000, B00000000
  }//f
  , {
    B00111110, B01000001, B01001001, B01001001, B00101110
  }//g
  , {
    B01111111, B00001000, B00001000, B00001000, B01111111
  }//h
  , {
    B01000001, B01000001, B01111111, B01000001, B01000001
  }//i
  , {
    B00000110, B00000001, B00000001, B00000001, B01111110
  }//j
  , {
    B01111111, B00011000, B00100100, B01000010, B00000001
  }//k
  , {
    B01111111, B00000001, B00000001, B00000001, B00000001
  }//l
  , {
    B01111111, B00100000, B00010000, B00100000, B01111111
  }//m
  , {
    B01111111, B00110000, B00001000, B00000110, B01111111
  }//n
  , {
    B00111110, B01000001, B01000001, B01000001, B00111110
  }//o
  , {
    B01111111, B01001000, B01001000, B01001000, B00110000
  }//p
  , {
    B00111100, B01000010, B01000010, B01000011, B00111101
  }//q
  , {
    B01111111, B01001000, B01001100, B01001010, B00110001
  }//r
  , {
    B00110001, B01001001, B01001001, B01001001, B01000110
  }//s
  , {
    B01000000, B01000000, B01111111, B01000000, B01000000
  }//t
  , {
    B01111110, B00000001, B00000001, B00000001, B01111110
  }//u
  , {
    B01111100, B00000010, B00000001, B00000010, B01111100
  }//v
  , {
    B01111111, B00000010, B00000100, B00000010, B01111111
  }//w
  , {
    B01000001, B00110110, B00001000, B00110110, B01000001
  }//x
  , {
    B01110000, B00001000, B00000111, B00001000, B01110000
  }//y
  , {
    B00010001, B00010011, B00010101, B00011001, B00010001
  }//z
  , {
    B00000000, B00000000, B00000000, B00000000, B00000000
  }//space
  , {
    B01111111, B00111110, B00011100, B00001000, B00000000
  }//right triangle
};

void loop() {

  last_time_raw = current_time_raw; // record the raw time from the last loop
  current_time_raw = millis(); // update the raw time

  background_runner(); // run actions associated with the background mode



  //  Serial.print("Time: ");
  //  Serial.println(current_time);


  byte radio_error = 0; // status of the radio serial read
  // Handler for incoming radio transmission

  while (Serial1.available()) {

    if (Serial1.read() == '[') { // If the packet has started
      unsigned long start_radio_read = millis(); // record the time the packet was received

      // Wait for all the bytes to come in, or for a timeout
      while (1) {
        if (Serial1.available() >= 24) {
          Serial.println("got 24 bytes");
          radio_error = 0;
          break;
        }
        else if ( (millis() - start_radio_read) > 100 ) {
          radio_error = 1;
          Serial.println("Timeout in radio trans.");
          break;
        }
      }

      if (radio_error == 0) { // Do this if there are enough bytes and no error
        byte radio_data[16];
        for (byte t = 0; t < 8; t++) { // Dump the username bytes for the moment
          Serial1.read(); // just dumping bytes
        }
        Serial.print("radio data: ");
        for (byte t = 0; t < 16; t++) { // read in all lat/long data
          radio_data[t] = Serial1.read();
          Serial.write(radio_data[t]);
        }
        Serial.println();

        byte lat_bytes[4];
        byte lon_bytes[4];

        // then read the lattitude and longitude bytes
        for (byte t = 0; t < 4; t++) {
          lat_bytes[-1 * t + 3] = 16 * hex_str(radio_data[2 * t]) + hex_str(radio_data[2 * t + 1]);
          lon_bytes[-1 * t + 3] = 16 * hex_str(radio_data[2 * t + 8]) + hex_str(radio_data[2 * t + 9]);
        }

        // then convert them into floats and save to the home coordinates
        memcpy(&home_coord[0], &lat_bytes, 4);
        memcpy(&home_coord[1], &lon_bytes, 4);

        Serial.println("updated home");
        Serial.println(home_coord[0]);
        Serial.println(home_coord[1]);

        // for debugging to show when packet is received
        light(1);
        delay(200);
        light(0);

      }

    }

  }

  cycle_buttons(); // cycle button_states array
  get_buttons(); // update button_states array

  // Navigation
  if (special_mode == 0) {
    byte menu_location_id_old = menu_location_id;
    menu_location_id = action(button_states, menu_location_id, menu_map, menu_map_size); // take action based on the current state of the buttons

    if ((menu_location_id_old != menu_location_id) || (force_update_menu == 1)) { // if menu location has changed, refresh display
      menu_display(menu_location_id, menu_map, menu_labels, menu_map_size); // update the display
      force_update_menu = 0; // reset the menu force update flag
    }
  }

  // this function automatically enables/disabled interrupts, but not GPS power
  // if GPS fix mode is active, run GPS fix function and get the state of the fix
  // gps_fix_active should only be used by background modes
  if (gps_fix_active == 1) {
    if (interrupts_active == 0) {
      int_in_use_background = 1; // reserve interrupts
      GPS_safe(-1, 1); // enable interrupts but not GPS power
    }
    byte gps_fix_acheived = get_GPS_fix(0); // 0 means don't turn GPS off after fix is made
    if (gps_fix_acheived == 1) {
      gps_fix_active = 0; // if fix is acheived, turn off gps fixing
      int_in_use_background = 0; // release interrupts
      GPS_safe(-1, 0); // safely disable interrupts (but not GPS power [the -1 indicating that])
    }
  }
  else { // if GPS has been set to not active
    if (interrupts_active == 1) { // but the interrupts are still enabled
      int_in_use_background = 0; // release interrupts
      GPS_safe(-1, 0); // safely disable interrupts (but not GPS power [the -1 indicating that])
    }
  }

  // Special Modes
  switch (special_mode) {
    case 1:
      coordinates_mode();
      break;
    case 2:
      home_set_mode();
      break;
    case 3:
      dist_mode();
      break;
  }


  //  acc_setup(1);
  //  delay(1000);
  //  mag(mag_values_raw);
  //  mag_scale(mag_values_raw, mag_range, mag_values_scaled);
  //
  //  //
  //  Serial.print(mag_values_scaled[0], DEC);
  //  Serial.print(", ");
  //  Serial.print(mag_values_scaled[1], DEC);
  //  Serial.print(", ");
  //  Serial.print(mag_values_scaled[2], DEC);
  //  Serial.print(", ");
  //  Serial.println(sqrt(pow(mag_values_scaled[0], 2) + pow(mag_values_scaled[1], 2) + pow(mag_values_scaled[2], 2)));

  //  delay(50);

}

// executes actions for whatever background mode we're in
void background_runner() {

  switch (background_mode) {

    case 0: // Currently in Hot Start Mode

      if ((current_time_raw >= 8000) && (last_time_raw < 8000)) { // at 8 seconds
        Serial.print(millis());
        Serial.println(", BG0: 8s");
        gps_fix_active = 1; // start listening to GPS serial
        Serial.print(millis());
        Serial.println(", BG0: starting to listen for GPS serial");
      }
      else if ((current_time_raw >= 10000) && (last_time_raw < 10000)) { // at 10 seconds
        Serial.print(millis());
        Serial.println(", BG0: 10s");
        if (gps_fix_active == 0) { // if fix has been achieved
          gps_in_use_background = 0; // release GPS power
          GPS_safe(0, -1); // turn off GPS safely
          background_mode = 2; // set background mode to Update Mode
          Serial.print(millis());
          Serial.println(", BG0: switching to BG2");
        }
        else { // if no fix has been achieved
          gps_fix_active = 0; // stop listening to GPS serial
          background_mode = 1; // set background mode to First Fix Mode
          Serial.print(millis());
          Serial.println(", BG0: switching to BG1");
        }
      }

      break;

    case 1: // Currently in First Fix Mode

      if ( ((current_time_raw + 2000) % 30000) < ((last_time_raw + 2000) % 30000) ) { // at 2 seconds before 30 second period
        gps_fix_active = 1; // start listening to GPS serial
        Serial.print(millis());
        Serial.println(", BG1: starting to listen to GPS serial");
      }
      else if ( (current_time_raw % 30000) < (last_time_raw % 30000) ) { // at 2 seconds before 30 second period
        if (gps_fix_active == 0) { // fix has been achieved
          gps_in_use_background = 0; // release GPS power
          GPS_safe(0, -1); // safely turn off GPS
          background_mode = 2; // set background mode to Update Mode
          Serial.print(millis());
          Serial.println(", BG1: switching to BG2");
        }
        else { // no fix achieved yet
          gps_fix_active = 0; // stop listening to GPS serial
          // don't change background mode - keep doing this until fix is achieved
          Serial.print(millis());
          Serial.println(", BG1: no fix yet");
        }
      }

      break;

    case 2: // Currently in Update Mode

      last_time = current_time; // record the time from the last loop
      current_time = millis() + time_correction; // update the GPS UTC time


      // at transmit window minus 5 seconds, get a GPS fix
      // at transmit window minus 5 seconds, pick random transmit timeslot
      // at transmit window + timeslot, transmit location
      // "timeslot" is the variable to set here

      // transmit minus 10 seconds (I think the + here is right...)
      if ( ((current_time + 10000) % transmit_period) < ((last_time + 10000) % transmit_period) ) {
        Serial.print(millis());
        Serial.println(", BG2: enabling GPS");
        // put into get gps fix mode
        gps_in_use_background = 1; // take GPS power
        GPS(1);
        gps_fix_active = 1;
        // set timeslot
        timeslot = random(0, transmit_window);
        //      Serial.println("Transmit window: Putting into GPS fix mode and setting timeslot");
        //      Serial.print("Timeslot: ");
        //      Serial.println(timeslot);
        //      Serial.println(millis());
      }

      // transmit plus 'timeslot' seconds
      if ( ((current_time - timeslot) % transmit_period) < ((last_time - timeslot) % transmit_period) ) {
        Serial.print(millis());
        Serial.println(", BG2: transmitting");
        gps_in_use_background = 0; // release GPS power
        GPS_safe(0, -1); // turn off GPS if a special mode isn't using it
        gps_serial.end();
        // noInterrupts(); // force interrupts off to avoid radio transmission conflict
        transmit_location();
        if (interrupts_active == 1) { // if the interrupts were on before the forced disable
          // interrupts(); // turn interrupts back on
          gps_serial.begin(9600);
        }
        //      Serial.println("Transmit window: transmitting location");
        //      Serial.println(millis());
      }

      break;

  }

}

void runner(int function_id) {
  switch (function_id) {
    case 0:
      // mag_cal(mag_range); PROGMEM
      break;
    case 1:
      quick_vibrate();
      break;
    case 2:
      GPS(1);
      break;
    case 3:
      GPS(0);
      break;
    case 4:
      light_state = light(light_state);
      break;
    case 5:
      special_mode = 1; // activate special mode 1 (show coordinates)
      break;
    case 6:
      special_mode = 2; // activate special mode 2 (set home)
      break;
    case 7:
      special_mode = 3; // activate special mode 3 (dist mode)
      nav_coord[0] = home_coord[0]; // dist mode needs to be told where to measure to
      nav_coord[1] = home_coord[1]; // the place to measure to is set by nav_coord and reported as nav_name
      nav_name = "Home";
      break;
    case 8: // make GPS get a single fix
      GPS(1);
      gps_fix_active = 1;
      break;
    case 9: // transmit location
      transmit_location();
      break;
  }

  menu_display(menu_location_id, menu_map, menu_labels, menu_map_size); // update the display

}


// this is glitchy for some reason - packets somehow get fragmented.  Buffer issue?
// send username and location over radio
// packet format:
// [ben-----<lat1><lat2><lat3><lat4><lon1><lon2><lon3><lon4>]
// start bracket, 8 character username (with dashes for unused chars), 8 bytes of lattitude float in hex, 8 bytes of longitude float in hex, end bracket
void transmit_location() {

  //  Serial1.print('[');
  //  Serial1.print(username);
  // http://forums.devshed.com/programming/567826-printing-float-hex-post2149393.html
  //  Serial1.print(*((long*)&current_coord[0]),HEX); // pointer to long interpretation of float (something like that)
  //  Serial1.print(*((long*)&current_coord[1]),HEX); // see above URL for explanation
  //  Serial1.print(']');

  String username_string = String(username);
  String lat_string = String(*((long*)&current_coord[0]), HEX);
  String lon_string = String(*((long*)&current_coord[1]), HEX);
  String transmit_string = String(   '[' + username_string + lat_string + lon_string + ']'  );

  Serial1.print(transmit_string);
  Serial.print(millis());
  Serial.print(", transmit_location: ");
  Serial.println(transmit_string);
  Serial.print(millis());
  Serial.println(", transmit_location: loc sent");

}

// converts single ASCII character to decimal value (0-15)
byte hex_str(byte input_character) {
  //  Serial.println(input_character);
  byte output;
  if ((input_character >= 97) && (input_character <= 102)) {
    output = input_character - 87;
  }
  else if ((input_character >= 48) && (input_character <= 57)) {
    output = input_character - 48;
  }
  Serial.println(output);
  return output;

}

// get a single GPS fix
byte get_GPS_fix(byte off_after_fix) {

  while (gps_serial.available() > 0) {
    gps.encode(gps_serial.read());
  }

  // if fix has been acheived
  if (gps.location.isUpdated()) {

    Serial.print(millis());
    Serial.println(", get_GPS_fix: GPS updated");

    first_gps_fix = 1;

    // get GPS coordinates
    current_coord[0] = gps.location.lat();
    current_coord[1] = gps.location.lng();

    // get time
    gps_time = long(gps.time.hour()) * 60 * 60 * 1000 + long(gps.time.minute()) * 60 * 1000 + long(gps.time.second()) * 1000 + long(gps.time.centisecond()) * 10;

    if ((gps_time - long(gps.time.age())) > 0) { // don't let time go less than 0
      gps_time = gps_time - long(gps.time.age()); // compensate for the fact that gps sample may be slightly stale
    }

    // this number can be added to millis() at any time to get the current gps time
    time_correction = gps_time - millis();

    //
    //    Serial.println("get_GPS_fix: GPS fix acheived");
    //    Serial.print("latitude: ");
    //    Serial.println(current_coord[0], 6);
    //    Serial.print("longitude: ");
    //    Serial.println(current_coord[1], 6);
    //    Serial.print("GPS fix age: ");
    //    Serial.println(gps.time.age());
    //    Serial.print("GPS time: ");
    //    Serial.println(gps_time);
    //    Serial.print("time_correction: ");
    //    Serial.println(time_correction);
    //    Serial.print("current_time: ");
    //    Serial.println(current_time);

    // Turn off GPS power if function has been commanded to turn off after fix is made
    if (off_after_fix == 1) {
      gps_in_use_background = 0;
      int_in_use_background = 0;
      GPS_safe(0, 0); // safely disable GPS power and interrupts
    }
    return 1; // notify outside code that fix has been acheived
  }
  else {
    return 0; // notify outside code that fix has not been acheived
  }

}

void coordinates_mode() {

  if (run_once == 0) {

    run_once = 1; // don't do this again unless the mode is activated again
    clear_lcd();
    full_word(2, "Coordinate Display");
    full_word(1, "Acquiring Satellites");

    gps_in_use_special = 1;
    int_in_use_special = 1;
    GPS_safe(1, 1); // enable GPS fully

  }



  // this is very slow and should not be done continuously
  // only slow when GPS is powered on and transmitting a serial stream
  // probably only need GPS updates every minute when not navigating and ~5 seconds when navigating (or more with compass)
  while (gps_serial.available() > 0) {
    gps.encode(gps_serial.read());
  }

  if (gps.location.isUpdated()) {
    String latitude = String(gps.location.lat(), 10);
    String longitude = String(gps.location.lng(), 10);
    char lat_char[10];
    char lon_char[10];
    latitude.toCharArray(lat_char, 10);
    longitude.toCharArray(lon_char, 10);
    Serial.print("LAT="); Serial.print(gps.location.lat(), 8);
    Serial.print("LNG="); Serial.println(gps.location.lng(), 8);
    clear_lcd();

    full_word(2, "--Current Location--");
    full_word(1, lat_char);
    full_word(0, lon_char);

  }

  if (any_buttons(button_states)) {
    cycle_buttons(); // cycle button array to prevent subsequent options in the loop from triggering
    special_mode = 0;
    run_once = 0;
    force_update_menu = 1;
    gps_in_use_special = 0;
    int_in_use_special = 0;
    GPS_safe(0, 0); // turn off GPS power and interrupts if unused by background mode
    Serial.println("Leaving coordinates mode.");
  }

}

void home_set_mode() {

  if (run_once == 0) {

    run_once = 1; // don't do this again unless the mode is activated again
    clear_lcd();
    full_word(2, "Setting Home");
    full_word(1, "Acquiring Satellites");

    gps_in_use_special = 1;
    int_in_use_special = 1;
    GPS_safe(1, 1); // enable GPS fully

  }


  // check for serial GPS data
  while (gps_serial.available() > 0) {
    gps.encode(gps_serial.read());
  }


  if (gps.location.isUpdated()) {

    // get GPS coordinates
    home_coord[0] = gps.location.lat();
    home_coord[1] = gps.location.lng();

    clear_lcd();
    full_word(2, "Home Set");
    delay(1000);
    special_mode = 0;
    run_once = 0;
    force_update_menu = 1;
    Serial.println("Home set successful.");
    Serial.println(home_coord[0], 8);
    Serial.println(home_coord[1], 8);
    Serial.println("Leaving home set mode.");


  }

  if (any_buttons(button_states)) {
    cycle_buttons(); // cycle button array to prevent subsequent options in the loop from triggering
    special_mode = 0;
    run_once = 0;
    force_update_menu = 1;
    gps_in_use_special = 0;
    int_in_use_special = 0;
    GPS_safe(0, 0); // turn off GPS power and interrupts if unused by background mode
    Serial.println("Leaving home set mode.");
  }

}

void dist_mode() {

  // leave distance mode if no home has been set
  if ((nav_coord[0] == 0.0) || (nav_coord[1] == 0.0)) {
    Serial.println("Leaving distance mode");
    clear_lcd();
    full_word(2, "Set Destination");
    full_word(1, "First");
    special_mode = 0;
    run_once = 0;
    force_update_menu = 1;
    delay(1000);
    return;
  }

  if (run_once == 0) {

    run_once = 1; // don't do this again unless the mode is activated again
    clear_lcd();
    full_word(2, "Distance Mode");
    full_word(1, "Acquiring Satellites");

    gps_in_use_special = 1;
    int_in_use_special = 1;
    GPS_safe(1, 1); // enable GPS fully

  }

  // check for serial GPS data
  while (gps_serial.available() > 0) {
    gps.encode(gps_serial.read());
  }


  if (gps.location.isUpdated()) {

    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    float dist_degrees = sqrt( sq(latitude - nav_coord[0]) + sq(longitude - nav_coord[1]) );
    float dist_meters = dist_degrees * 111000.0; // rough conversion from degrees lat/lon to meters

    String dist_meters_str = String(dist_meters, 1) + " meters";
    char dist_meters_char[20];
    dist_meters_str.toCharArray(dist_meters_char, 20);

    String nav_name_disp = nav_name + ":"; // just to add a colon to the end of the name
    char nav_name_char[20];
    nav_name_disp.toCharArray(nav_name_char, 20);

    clear_lcd();
    full_word(2, "Distance To");
    full_word(1, nav_name_char);
    full_word(0, dist_meters_char);

  }

  if (any_buttons(button_states)) {
    cycle_buttons(); // cycle button array to prevent subsequent options in the loop from triggering
    special_mode = 0;
    run_once = 0;
    force_update_menu = 1;
    gps_in_use_special = 0;
    int_in_use_special = 0;
    GPS_safe(0, 0); // turn off GPS power and interrupts if unused by background mode
    Serial.println("Leaving dist mode.");
  }


}

void quick_vibrate() {
  vibrate(1);
  delay(1000);
  vibrate(0);
}

// returns 1 if any buttons have changed, else 0
byte any_buttons(byte button_states[2][4]) {
  byte any_pressed = 0;
  for (byte b = 0; b < 4; b++) {
    if ((button_states[0][b] == 1) && (button_states[1][b] == 0)) {
      any_pressed = 1;
    }
  }
  return any_pressed;
}

byte action(byte button_states[2][4], byte menu_location_id, int menu_map[][4], byte menu_map_size) {

  int menu_map_index; // index in the menu map of the current location
  int parent_id;

  // get index associated with menu_location_id
  for (int t = 0; t < menu_map_size; t++) {
    if (menu_map[t][0] == menu_location_id) {
      menu_map_index = t;
    }
  }

  //  Serial.print("menu_location_id: ");
  //  Serial.println(menu_location_id);
  //  Serial.print("menu_map_index: ");
  //  Serial.println(menu_map_index);
  //  Serial.print("parent_id: ");
  //  Serial.println(parent_id);
  //  Serial.print("menu_map_size: ");
  //  Serial.println(menu_map_size);

  // record parent ID of menu_location
  parent_id = menu_map[menu_map_index][2];

  //**************  UP  ***************
  if ((button_states[0][0] == 1) && (button_states[1][0] == 0)) { // if up button has been pressed
    cycle_buttons(); // cycle button array to prevent subsequent options in the loop from triggering
    Serial.println("button 0");
    // search up in menu map for ID with same parent ID, and change menu_location_id to that
    for (int t = menu_map_index - 1; t >= 0; t--) {
      if (menu_map[t][2] == parent_id) {
        menu_location_id = menu_map[t][0];
        break;
      }
    }
  }

  //**************  DOWN  ***************
  else if ((button_states[0][1] == 1) && (button_states[1][1] == 0)) { // if down button has been pressed
    cycle_buttons(); // cycle button array to prevent subsequent options in the loop from triggering
    Serial.println("button 1");
    // search down in menu map for ID with same parent ID, and change menu_location_id to that
    for (int t = menu_map_index + 1; t < menu_map_size; t++) {
      if (menu_map[t][2] == parent_id) {
        menu_location_id = menu_map[t][0];
        break;
      }
    }
  }

  //**************  RIGHT  ***************
  else if ((button_states[0][2] == 1) && (button_states[1][2] == 0)) { // if right button has been pressed
    cycle_buttons(); // cycle button array to prevent subsequent options in the loop from triggering
    Serial.println("button 2");
    // change menu_location_id to the next_id of the current menu_location_id
    if (menu_map[menu_map_index][1] == -1) { // if menu position is an endpoint with a function to run
      runner(menu_map[menu_map_index][3]); // start the function runner with the function ID
    }
    else { // not an endpoint - no function to run
      menu_location_id = menu_map[menu_map_index][1]; // just update the menu position
    }
  }

  //**************  LEFT  ***************
  else if ((button_states[0][3] == 1) && (button_states[1][3] == 0)) { // if left button has been pressed
    cycle_buttons(); // cycle button array to prevent subsequent options in the loop from triggering
    Serial.println("button 3");
    // change menu_location_id to the parent_id of the current menu_location_id
    menu_location_id = menu_map[menu_map_index][2];
  };

  return menu_location_id;

}

void menu_display(byte menu_location_id, int menu_map[][4], char* menu_labels[], byte menu_map_size) {
  // find index of menu_location in menu_map

  int menu_map_index; // index in the menu map of the current location
  int parent_id; // ID of the parent of the current location
  int prev_id = -1; // ID of item to be shown previously on display
  int next_id = -1; // ID of item to be shown next on display

  char prev_id_text[20];
  char current_id_text[20];
  char next_id_text[20];

  clear_lcd();

  for (int t = 0; t < menu_map_size; t++) {
    if (menu_map[t][0] == menu_location_id) {
      menu_map_index = t;
    }
  }

  // record parent ID of menu_location
  parent_id = menu_map[menu_map_index][2];

  // move up menu_map, and record if there's another with same parent ID
  for (int t = menu_map_index - 1; t >= 0; t--) {
    if (menu_map[t][2] == parent_id) {
      prev_id = menu_map[t][0];
      break;
    }
  }

  // move down menu_map, and record if there's another with same parent ID
  for (int t = menu_map_index + 1; t < menu_map_size ; t++) {
    if (menu_map[t][2] == parent_id) {
      next_id = menu_map[t][0];
      break;
    }
  }

  // print labels

  if (prev_id >= 0) {
    //    Serial.println(menu_labels[prev_id]);
    sprintf(prev_id_text, "%s%s", "  ", menu_labels[prev_id]);
    full_word(2, prev_id_text);
  }
  //  Serial.println(menu_labels[menu_location_id]);
  sprintf(current_id_text, "%s%s", "> ", menu_labels[menu_location_id]); // add  a bullet triangle to this one
  full_word(1, current_id_text);
  Serial.println(menu_labels[menu_location_id]);
  if (next_id >= 0) {
    //    Serial.println(menu_labels[next_id]);
    sprintf(next_id_text, "%s%s", "  ", menu_labels[next_id]);
    full_word(0, next_id_text);
  }

  //  Serial.println();

}

// scale raw mag values based on full calibration range
void mag_scale(int mag_values_raw[], int mag_range[], int mag_values_scaled[]) {
  for (int x = 0; x <= 2; x++) { // loop through xyz axes
    mag_values_scaled[x] = map(mag_values_raw[x], mag_range[x * 2], mag_range[x * 2 + 1], -1000, 1000);
  }
}


// pulls saved magnetometer calibration from EEPROM and saves in mag_range array
void mag_cal_ret(int mag_range[]) {
  for (int t = 0; t <= 5; t++) { // loop through Xmin, Xmax, Ymin, Ymax, Zmin, Zmax
    EEPROM.get(t * 4, mag_range[t]); // set array
  }
}

/* PROGMEM
void mag_cal(int mag_range[]) {
  // for t times, get a magnetometer reading
  // record the maximum and minimum values for each axis
  // save those max/min values to an array

  clear_lcd();
  full_word(3, "calibrate now");

  int current_reading[3];

  // get magnetometer reading, and default the range array to the current value
  mag(current_reading); // PROGMEM
  mag_range[0] = current_reading[0];
  mag_range[1] = current_reading[0];
  mag_range[2] = current_reading[1];
  mag_range[3] = current_reading[1];
  mag_range[4] = current_reading[2];
  mag_range[5] = current_reading[2];

  // do a bunch of samples
  for (int t = 0; t <= 100; t++) {
    mag(current_reading); // get a magnetometer reading
    for (int l = 0; l <= 2; l++) { // loop through x/y/z axes
      if (current_reading[l] < mag_range[2 * l]) { // if reading is less than low limit
        mag_range[2 * l] = current_reading[l]; // change limit
      }
      if (current_reading[l] > mag_range[2 * l + 1]) { // if reading is greater than high limit
        mag_range[2 * l + 1] = current_reading[l]; // change limit
      }
    }

    delay(200);
  }

  // record new range to EEPROM to be available after next reboot
  for (int t = 0; t <= 5; t++) { // loop through Xmin, Xmax, Ymin, Ymax, Zmin, Zmax
    EEPROM.put(t * 4, mag_range[t]); // set array
    //    Serial.println(mag_range[t]);
  }

  clear_lcd();
  full_word(3, "calibration complete");

  delay(2000);

}
/*


// doesn't set up yet - just accelerometer debugging
void acc_setup(byte state) {
  // Accelerometer is 0x1D
  byte device = 0x1D;
  byte whoami = 0x0D;
  // 1D 0D
  // 0E 07
  Wire.beginTransmission(device);
  //  delay(50);
  Wire.write(whoami);
  //  delay(50);
  Wire.endTransmission();
  //  delay(50);

  Wire.requestFrom(device, 1);
  //  delay(50);
  //
  // timeout after 5 milliseconds of waiting
  int time = millis();
  while (Wire.available() < 1) {
    if ((millis() - time) > 5) {
      Serial.println("Timeout in acc_setup");
      return;
    }
  }
  Serial.print("Acc ID: ");
  byte result = Wire.read();
  Serial.println(result, HEX);

}

// Magnetometer setup (1 active, 0 standby)
void mag_setup(byte state) {

  Wire.beginTransmission(0x0E);
  Wire.write(0x10); // CTRL_REG1
  if (state == 1) {
    Wire.write(B00111001); // 5 Hz readout, 128 oversample, automatic.  LSB is active/standby
  }
  else if (state == 0) {
    Wire.write(B00111000); // 5 Hz readout, 128 oversample, automatic.  LSB is active/standby
  }
  Wire.endTransmission();

  Wire.beginTransmission(0x0E);
  Wire.write(0x11); // CTRL_REG2
  Wire.write(B10100000); // Automatic resets, raw output, no reset
  Wire.endTransmission();

}

/* PROGMEM
void mag(int reading[]) {
  // X-Z MSB-LSB is 0x01 to 0x06

  byte MSB;
  byte LSB;

  Wire.beginTransmission(0x0E);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.requestFrom(0x0E, 6);

  // timeout after 5 milliseconds of waiting
  int time = millis();
  while (Wire.available() < 6) {
    if ((millis() - time) > 5) {
      Serial.println("Timeout in mag");
      break;
    }
  }

  while (Wire.available() > 6) { // purge extra bytes (probably totally unnecessary)
    Wire.read();
  }

  for (byte t = 0; t <= 2; t++) {
    MSB = Wire.read();
    LSB = Wire.read();
    reading[t] = (MSB << 8) | LSB;
  }

}
*/

// WON'T PORT
void a_lcd(int state) {
  digitalWrite(LCD_A0, state);
}

// PORTED
void start_text(byte u) {

  pinMode(LCD_CS, OUTPUT); // assert SS pin
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LCD_CS, LOW); // pull SS pin low

  a_lcd(0);
  SPI.transfer(B01000000); // line
  SPI.transfer(B10110000 | u); // page
  SPI.transfer(B00010000); // column
  SPI.transfer(B00000000);
  a_lcd(1);

  SPI.endTransaction();
  pinMode(LCD_CS, INPUT); // release SS pin
}

void full_word(byte p, char mystring[]) {

  byte len = strlen(mystring);
  //  Serial.println(mystring);
  //  Serial.println(strlen(mystring));
  start_text(p); // p is which line

  for (int r = 0; r < len; r++) {
    letter(mystring[r]);
  }

}

// PORTED
void letter(char l) {

  pinMode(LCD_CS, OUTPUT); // assert SS pin
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LCD_CS, LOW); // pull SS pin low

  if (l == 32) { // if it's a space
    for (byte t = 0; t < 5; t++) {
      SPI.transfer(letters[26][t]);
    }
  }
  else if (l == 62) { // if it's a right triangle
    for (byte t = 0; t < 5; t++) {
      SPI.transfer(letters[27][t]);
    }
  }
  else if ((l >= 40) && (l <= 58)) { // numbers and some punctuation
    for (byte t = 0; t < 5; t++) {
      SPI.transfer(numbers[l - 40][t]);
    }
  }
  else if (l > 95) { // if it's lowercase
    for (byte t = 0; t < 5; t++) {
      SPI.transfer(letters[l - 97][t]);

    }
  }
  else { // if it's uppercase
    for (byte t = 0; t < 5; t++) {
      SPI.transfer(letters[l - 65][t]);
    }
  }

  SPI.transfer(B00000000); // gap between letters

  SPI.endTransaction();
  pinMode(LCD_CS, INPUT); // release SS pin

}


// PORTED
void clear_lcd() {

  pinMode(LCD_CS, OUTPUT); // assert SS pin
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LCD_CS, LOW); // pull SS pin low

  for (byte page = 0; page < 4; page++) {
    byte page2 = page | B10110000;
    a_lcd(0);
    SPI.transfer(B01000000); // line
    SPI.transfer(page2);
    SPI.transfer(B00010000); // column
    SPI.transfer(B00000000);
    a_lcd(1);
    for (int t = 0; t < 128; t++) {
      SPI.transfer(B00000000);
    }
  }

  SPI.endTransaction();
  pinMode(LCD_CS, INPUT); // release SS pin
}

// PORTED
void init_lcd() {

  pinMode(LCD_CS, OUTPUT); // assert SS pin
  SPI.beginTransaction(SPISettings(15000000, MSBFIRST, SPI_MODE0)); //20MHz is fastest speed
  digitalWrite(LCD_CS, LOW); // pull SS pin low

  // LCD bias setting
  SPI.transfer(B10100010);
  // ADC selection
  SPI.transfer(B10100000); // normal
  // common output mode selection
  // resistor ratio
  SPI.transfer(B00100010); // 000 to 111
  // volume control
  SPI.transfer(B10000001);
  SPI.transfer(B00010000); //000000 to 111111
  // power control setting
  SPI.transfer(B00101111); // 111 for using internal power supply
  // initialize dram
  // turn on
  SPI.transfer(B10101111);

  SPI.endTransaction();

  pinMode(LCD_CS, INPUT); // release SS pin

}

// cycle buttons
void cycle_buttons() {
  // cycle new button readings to old
  for (int t = 0; t <= 3; t++) {
    button_states[1][t] = button_states[0][t];
  }
}

// gets latest button states (push = 1)
void get_buttons() {

  button_states[0][0] = digitalRead(UP);
  button_states[0][1] = digitalRead(DOWN);
  button_states[0][2] = digitalRead(RIGHT);
  button_states[0][3] = digitalRead(LEFT);

}

//checks for bytes from GPRMC data, adds them to char array as they come in
//once a complete GPRMC array is stored, parses lattitude, longitude, and time
void GPS_receive() {
  // might be a smart way to use the String object to just buffer the data and truncate to the data

}

// turns off GPS and/or turns off interrupts, first checking that no one's using them
// first input 1/0 for gps power on/off
// second input 1/0 for interrupts enabled/disabled
void GPS_safe(int gps_state, int int_state) {

  if (gps_state == 1) {
    GPS(1);
    Serial.print(millis());
    Serial.println(", GPS_safe: gps on");
  }
  if ((int_state == 1) && (interrupts_active == 0)) { // enable interrupts if command and they're not already active
    gps_serial.begin(9600);
    interrupts_active = 1; // ... and remember they've been activated
    Serial.print(millis());
    Serial.println(", GPS_safe: int on");
  }
  if (gps_state == 0) {
    if ((gps_in_use_background == 0) && (gps_in_use_special == 0)) { // if no one's using GPS
      GPS(0); // turn off GPS
      Serial.print(millis());
      Serial.println(", GPS_safe: gps off");
    }
  }
  if ((int_state == 0) && (interrupts_active == 1)) { // disable interrupts if commanded and they're enabled
    if ((int_in_use_background == 0) && (int_in_use_special == 0)) { // if no one's using interrupts
      gps_serial.end();
      interrupts_active = 0; // ... and remember they've been deactivated
      Serial.print(millis());
      Serial.println(", GPS_safe: int off");
    }
  }

}

void GPS(int state) {
  if (state == 1) {
    digitalWrite(GPS_EN, LOW); // GPS on
    Serial.print(millis());
    Serial.println(", GPS on");
    //    clear_lcd();
    //    full_word(2, "GPS Power On");
    //    delay(1000);
    //    force_update_menu = 1;
  }
  else if (state == 0) {
    digitalWrite(GPS_EN, HIGH); // GPS off
    Serial.print(millis());
    Serial.println(", GPS off");
    //    clear_lcd();
    //    full_word(2, "GPS Power Off");
    //    delay(1000);
    //    force_update_menu = 1;
  }

}

// Set light (1 on, 0 off)
byte light(byte state) {

  digitalWrite(LIGHT, -1 * (state - 1) );

  state = -1 * state + 1; // invert
  return state;

}

// Set vibration (1 on, 0 off)
void vibrate(int state) {
  digitalWrite(VIBRATE, -1 * (state - 1) );
}

// Set LCD backlight (1 on, 0 off)
void backlight(int state) {
  digitalWrite(LED_backlight, -1 * (state - 1) );
}

/* PROGMEM
// Return battery voltage in volts
float battery_voltage() {

  return 3.3 * 2.0 * float(analogRead(A0)) / 1024.0;

}
*/

/* PROGMEM
// Convert battery voltage to state of charge (percentage)
int battery_SOC(float battery_voltage) {
  int SOC;
  int voltage = int(100 * battery_voltage); // voltage * 100

  if (voltage >= 420) {
    SOC = 100;
  }
  else if (voltage < 420 && voltage >= 394) {
    SOC = map(voltage, 394, 420, 80, 100);
  }
  else if (voltage < 394 && voltage >= 382) {
    SOC = map(voltage, 382, 394, 60, 80);
  }
  else if (voltage < 382 && voltage >= 374) {
    SOC = map(voltage, 374, 382, 40, 60);
  }
  else if (voltage < 374 && voltage >= 367) {
    SOC = map(voltage, 367, 374, 20, 40);
  }
  else if (voltage < 367 && voltage >= 358) {
    SOC = map(voltage, 358, 367, 10, 20);
  }
  else if (voltage < 358 && voltage >= 300) {
    SOC = map(voltage, 300, 358, 0, 10);
  }
  else if (voltage < 300) {
    SOC = 0;
  }

  //  100: 4.2
  //  80: 3.94
  //  60: 3.82
  //  40: 3.74
  //  20: 3.67
  //  10: 3.58
  //  0: 3.0

  return SOC;

}
*/




