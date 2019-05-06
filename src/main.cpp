#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_Si7021.h>

#define DEBUG
#undef DEBUG

#define HOME 30000
#define MAXCOMMAND 8
#define DHTTYPE DHT22

/////////////////////////
// Pin locations
#define RUNLED 13                 // Amber LED lights whenever motor is active, 13 for nano 
#define STEP_PIN        7         // D7
#define DIRECTION_PIN   8         // D8
#define ENABLE_PIN      9         // D9

// m0/m1/m2 sets stepping mode 000 = F, 100 = 1/2, 010 = 1/4, 110 = 1/8, 001 = 1/16, 101 = 1/32
// steps per revolution = 200, 400, 800, 1600, 6400
// must set the current limiting for microstepping to work correctly
#define myM0      4  // microstepping lines
#define myM1      5
#define myM2      6

#define MAXSPEED  100
#define SPEEDMULT 3

char inChar;
char cmd[MAXCOMMAND];
char param[MAXCOMMAND];
char line[MAXCOMMAND];
char tempString[6];

int isRunning = 0;
int speed = 2;
int eoc = 0;
int idx = 0;
long millisLastMove = 0;
int testPin = 12;
int coilPwr = true;
int tempAvailable = false;      // temperature sensor available flag

// Temperature measurement
int temperatureChannel = 0;
unsigned int wADC;

#define SCALE 1.25    //  Temperature compensation
#define OFFSET 100 /* 2 * 50 offset for TPM-36 */

AccelStepper  stepper(AccelStepper::DRIVER,STEP_PIN, DIRECTION_PIN );
Adafruit_Si7021 sensor = Adafruit_Si7021();

int GetTemp(void) {
  // supposed to be "four-digit signed (2's complement) hex number
  // but docs are inconsistent
  //
  float temp = 0;
  int itemp = 0;


  temp  = sensor.readTemperature();
  if ( isnan(temp) )  {
    temp = 0;
  } 
  itemp = (int) (temp * SCALE);
  return itemp;
}

long hexstr2long(char *line) {
  long ret = 0;
  ret = strtol(line, NULL, 16);
  return (ret);
}  


void setup() {
    Serial.begin(9600);

  // Try and initialize the si7021 temperature sensor
  //
  tempAvailable = sensor.begin();

#ifdef DEBUG
  if ( tempAvailable == true )  {
    Serial.println("Temperature sensor found...");
  } else {
    Serial.println("Temperature sensor unavailable...");
  }
#endif
  
  pinMode(RUNLED, OUTPUT); /* yellow LED */
  pinMode(testPin, OUTPUT);

  //  Set the "0" position on startup
  stepper.setCurrentPosition( HOME );        

  stepper.setSpeed( MAXSPEED );
  stepper.setMaxSpeed( MAXSPEED );
  stepper.setAcceleration( 10 );
  stepper.enableOutputs();

  memset(line, 0, MAXCOMMAND);
  millisLastMove = millis();
}

void loop() {

  int tempTemp;
  long pos;

  if (!Serial.available()) {
  
    // run the stepper if there's no pending command and if there are pending movements
    //
    if (isRunning) {
      stepper.run();
      millisLastMove = millis(); /* reset idle timer */
      digitalWrite( RUNLED, HIGH );
      
    } else { /* Check to see if idle time is up */
  
      if ((millis() - millisLastMove) > 15000) {
        // if so, turn off motor
        if (coilPwr = false)  {
          stepper.disableOutputs();
        }
      }
    }
      
    // if motion is complete
    if (stepper.distanceToGo() == 0) {  
      stepper.run();
      isRunning = 0;
      digitalWrite(RUNLED, LOW);
    }
    
  } else {

    /////////////////////////////////////////////////////
    // read the command until the terminating # character
    //
    while (Serial.available() && !eoc) {
      inChar = Serial.read();

      // Skip CR or LF
      if ( inChar == 0x0a || inChar == 0x0d) {
        continue;
      }
      
      if (inChar != '#' && inChar != ':') {
        line[idx++] = inChar;

#ifdef DEBUG  
        Serial.print( inChar, HEX );
#endif
        if (idx >= MAXCOMMAND)
          idx = MAXCOMMAND - 1;
      }
      else {
        if (inChar == '#')
          eoc = 1;
      }
    }
  } // end if (!Serial.available())

  //////////////////////////////////////
  // process the command we got
  //
  if ( eoc ) {
    digitalWrite(testPin, LOW);

    ////////////////
    // Split into command and optional parameter
    //
#ifdef DEBUG  
    Serial.print( "Line = " ); Serial.println( line );
#endif    
    memset(cmd, 0, MAXCOMMAND);
    memset(param, 0, MAXCOMMAND);
    
    int len = strlen(line);

    if (len >= 2) 
      strncpy(cmd, line, 2);

    if (len > 2)
      strncpy(param, line + 2, len - 2);

    // the stand-alone program sends :C# :GB# on startup
    // :C# is to start a temperature conversion, doesn't require any response (we don't use it)
    eoc = 0;                          // reset counters
    idx = 0;

    /////////////////////////////////
    //  Command processing
    //
    if ( strncmp( line, "GP",2) == 0 ) {    
      // Get Position
      //     
      pos = stepper.currentPosition();
      sprintf(tempString, "%04X", pos);
      Serial.print(tempString);
      Serial.print("#");
      
    } else if (strncmp(line,"GT",2) == 0) {
      // Get Temperature
      //
      tempTemp = GetTemp();
      sprintf(tempString, "%04X", tempTemp);
      Serial.print(tempString);
      Serial.print("#");     
      
    } else if (strncmp(line,"GI",2) == 0) {
      // GI command 01 if motor running, 00 if not
      //
#ifdef DEBUG  
      Serial.print("In GI");
#endif
      if ( abs(stepper.distanceToGo()) > 0 )
        Serial.print("01#");
      else
        Serial.print("00#");
                
    } else if (strncmp(line,"GB",2) == 0) {
        // GB command Get current backlight value, always 00
        //
        Serial.print("00#");
      
    } else if (strncmp(line,"PH",2) == 0 ) {
        // PH command Find motor home
        //
        stepper.setCurrentPosition( HOME );
        stepper.moveTo( 0 );
        isRunning = 1;
        digitalWrite(RUNLED, HIGH);
     
    } else if (strncmp(line,"GV",2) == 0 ) {
        // GV command Get software version, always 10
        //
#ifdef DEBUG  
        Serial.println( "In GV");
#endif        
        Serial.print("10#");

    } else if (strncmp(line,"GN",2) == 0 ) {
        // GN command Get new (target) position
        //
        pos = stepper.targetPosition();
        sprintf(tempString, "%04X", pos);
        Serial.print(tempString);
        Serial.print("#");
      
    } else if (strncmp(line,"GC",2) == 0 ) {  
        // GC command Get temerature coefficient, always 2
        //
        Serial.print("02#");
      
    } else if (strncmp(line,"GD",2) == 0 ) {
        // GD command Get motor speed
        //
        sprintf(tempString, "%02X", speed);
        Serial.print(tempString);
        Serial.print("#");
      
    } else if (strncmp(line,"SD",2) == 0 ){
        // SD command Set motor speed
        //
        speed = hexstr2long(param);
        stepper.setSpeed( MAXSPEED );
        stepper.setMaxSpeed( MAXSPEED );
      
    } else if (strncmp(line,"GH",2) == 0 ) {
        // GH command Get half step mode, always 00
        //
        Serial.print("00#");
      
    } else if (strncmp(line,"SP",2) == 0 ) {  
        // SP command Set current position
        //
        pos = hexstr2long(param);
        stepper.setCurrentPosition( pos );        

    } else if (strncmp(line,"SN",2) == 0 ) {  
        // SN command Set New position
        //
        /**
        This is using absolute positions, which is screwed up when moving -1
        
        pos = hexstr2long(param);
        stepper.moveTo( pos );          
        **/

        // trying moving relative to the current position, vs. absolute positioning
        //
        pos = hexstr2long(param);
        stepper.move( pos - stepper.currentPosition());
        
    } else if (strncmp(line,"FG",2) == 0 ) {
        // FG command Start motor command
        //
        stepper.enableOutputs();
        isRunning = 1;
        digitalWrite(RUNLED, HIGH);
              
    } else if (strncmp(line,"FQ",2) == 0 ) {
        // FQ command Stop motor command
        //
        isRunning = 0;
        stepper.moveTo( stepper.currentPosition() );
        stepper.run();
        digitalWrite(RUNLED, LOW);
        
    }

    memset(line, 0, MAXCOMMAND);      // zero out buffer
    digitalWrite(testPin, HIGH);
      
  } // end process command

}