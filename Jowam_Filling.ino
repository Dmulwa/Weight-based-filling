/*
  Water VendingMachine.ino

*/
#include "LiquidCrystal_I2C.h"// For LCD
#include <EEPROM.h>
#include <avr/wdt.h>
#include "HX711.h"


#include <SoftwareSerial.h>
#include <EdgeDebounceLite.h>    //Use EdgeDebounceLite to debounce the pin
EdgeDebounceLite debounce;       //Name it debounce

//LCD definitions
//--------------LCD DEFINITIONS-----------------
#define COLUMNS           16
#define ROWS             2

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 11;

HX711 scale;

//H/w interconnect pins:
#define PMP 16

float m_drn, bmlk, r_dn, ltrs;
//This variables should be read from EEPROM at setup()
float half_liter;
float liter;
unsigned long  m_clb;
bool state;
bool pos;
bool lock_sys;

int min_pulse_width; // the minimum pulse width to acccept
int max_pulse_width; // the maximum pulse width to accept
int debounce_speed; // ignore changes in input line state that happen faster than this
//int pulse_count; // how many pulses have been received so far in this pulse train

unsigned long pulse_duration; // how long was the last pulse
unsigned long pulse_begin; // when did the last pulse begin
unsigned long pulse_end; // if they pulse was within min and max pulse width, when did it end
unsigned long curtime; // what is the current time
int post_pulse_pause; // how long to wait after last pulse before sending pulse count
int pulse_state; // what is the current input line state (1 for high, 0 for low)
int last_state; // what was the last input line state

enum VendingMachineStates {WAIT, FILL_1L, FILL_L, FILL_Q, DRAIN, CALIBRATE};  //The five possible states of the Vending state machine
VendingMachineStates vmState = WAIT ;                                          //Start state is HOME

enum SwitchStates {IS_OPEN, IS_RISING, IS_CLOSED, IS_FALLING};  //The four possible states of the Switch state machine
SwitchStates switchState[2] = {IS_OPEN, IS_OPEN};               //Both switch's states are IS_OPEN
byte switchPin[2] = {14, 15};                                   //Switches are on pins 10 and 11

enum SwitchModes {PULLUP, PULLDOWN};              //The two possible modes for the switches
SwitchModes switchMode[2] = {PULLDOWN, PULLDOWN};     //Both switches are in PULLUP mode


uint16_t volatile  refresh = 0;
uint32_t instant_t1, instant_t0, instant_t6 , instant_t11;
bool cleaning;
bool filling;
bool calibrating;

//lcd setup

#define LCD_SPACE_SYMBOL 0x20  //space symbol from the lcd ROM, see p.9 of GDM2004D datasheet

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);


void switchMachine(byte i) {                      //The Switch state machine
  byte pinIs = debounce.pin(switchPin[i]);          //<<<<<Replace digitalRead() with debounce.pin()
  if (switchMode[i] == PULLUP) pinIs = !pinIs;      //Reverse the value read if the switch is in PULLUP mode
  switch (switchState[i]) {                         //Depending of the state
    case IS_OPEN:    {                                //State is IS_OPEN
        if (pinIs == HIGH)                                //If the pin is HIGH
          switchState[i] = IS_RISING;                       //We just changed form LOW to HIGH: State is now IS_RISING
        break;                                            //Get out of switch
      }
    case IS_RISING:  {                                //State is IS_RISING
        switchState[i] = IS_CLOSED;                       //It is not rising anymore, State is now IS_CLOSED
        break;                                            //Get out of switch
      }
    case IS_CLOSED:  {                               //State is IS_CLOSED
        if (pinIs == LOW)                                //If the pin is LOW
          switchState[i] = IS_FALLING;                     //We just changed form HIGH to LOW: State is now IS_FALLING
        break;                                           //Get out of switch
      }
    case IS_FALLING: {                               //State is IS_FALLING
        switchState[i] = IS_OPEN;                        //It is not falling anymore, State is now IS_OPEN
        break;                                           //Get out of switch
      }
  }
}

bool button0() {                    //Find out if a dollar has been inserted
  switchMachine(0);                           //Read switch 0
  if (switchState[0] == IS_FALLING)           //If it is in the state IS_FALLING
    return true;                                //A dollar has been inserted, return true
  else                                        //If not
    return false;                               //return false
}

bool button1() {                     //Find out if a quarter has been inserted
  switchMachine(1);                           //Read switch 1
  if (switchState[1] == IS_FALLING)           //If it is in the state IS_FALLING
    return true;                                //A quarter has been inserted, return true
  else                                        //If not
    return false;                               //return false
}


void calib() {
  bool calibrating = true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">> CALIBRATE - 1L <<");
  lcd.setCursor(0, 1);
  lcd.print("Remove weights");
  delay(3000);
  while (!scale.is_ready());
  scale.set_scale();
  scale.tare();

  //Calibrate scale

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">> CALIBRATE - 1L <<");
  lcd.setCursor(0, 1);
  lcd.print("START-> GREEN ");

  while (calibrating) {

    if (button0()) {
      digitalWrite(PMP, HIGH);
      //-----------LCD prob.------------
      // display
      instant_t0 = millis();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(">> CALIBRATE - 1L <<");
      lcd.setCursor(2, 1);
      lcd.print(" STOP->RED ");

      //Serial.println("Press red to stop");
    }
    if (button1()) {

      digitalWrite(PMP, LOW);

      //read scale
      long reading  = scale.get_units(5);

      //calibration factor will be the (reading)/(known weight in grams)
      m_clb = (reading) /   (1000);

      EEPROM.put(70, m_clb);
      // display

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(">> CALIBRATE - 1L <<");
      lcd.setCursor(0, 1);
      lcd.print("DONE!");
      delay(3000);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(">> CALIBRATE - 1L <<");
      lcd.setCursor(0, 1);
      lcd.print(m_clb);

      //delay
      delay(3000);
      calibrating = false;

    }


  }
}



void Fill(float vol) {
  bool filling = false;
  bool vend = true;
  unsigned long instant_t6 = millis(); // Use unsigned long for millis()
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">> FILL-");
  lcd.print(vol);
  lcd.print(" L <<");

  // Wait for scale to be ready and calibrate
  while (!scale.is_ready());
  scale.set_scale(m_clb);
  scale.tare();

  float target = 1000 * vol;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(">> FILL-");
  lcd.print(vol);
  lcd.print(" L <<");
  lcd.setCursor(0, 1);
  lcd.print("GREEN->START");

  while (vend) {
    if (button0()) {
      digitalWrite(PMP, HIGH);
      filling = true;
    }
    if (button1()) {
      digitalWrite(PMP, LOW);
      vend = false;
      filling = false;
    }

    while (filling) {
      long readVal = scale.get_units(2); // Corrected scale reading

      if (readVal >= target) {
        digitalWrite(PMP, LOW);
        instant_t6 = millis();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(">> FILL-");
        lcd.print(vol);
        lcd.print(" L <<");
        lcd.setCursor(0, 1);
        lcd.print(">> WGT- ");
        lcd.print(readVal);
        lcd.print(" g");

        while (!((millis() - instant_t6 >= 3000) && (scale.get_units(5) < 50)));

        delay(3000);
        digitalWrite(PMP, HIGH);


      }

     

      // Update display every 2 seconds
      if (millis() - instant_t6 >= 400) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(">> FILL-");
        lcd.print(vol);
        lcd.print(" L <<");
        lcd.setCursor(0, 1);
        lcd.print(">> WGT-");
        lcd.print(readVal);
        lcd.print(" g");
        instant_t6 = millis(); // Reset timer after updating display
      }

      // Stop filling if button1 is pressed
      if (button1()) {
        digitalWrite(PMP, LOW);
        filling = false;
        vend = false;
      }
    }
  }
}

void Drain() {
  uint32_t instant_t6 = 0;
  cleaning = true;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("    >>Drain: <<     ");
  lcd.setCursor(0, 2);
  lcd.print(" GREEN->START:");

  while (cleaning) {
    if (button0()) {
      digitalWrite(PMP, HIGH);
      //-----------LCD prob.------------
      // display
      //Serial.println("Press RED to stop:");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("    >>Drain: <<     ");
      lcd.setCursor(0, 1);
      lcd.print(" RED->STOP:");
    }
    if (button1()) {

      digitalWrite(PMP, LOW);

      // display
      // Serial.println("Done!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("    >>Drain: <<     ");
      lcd.setCursor(0, 1);
      lcd.print("  Done!");
      //delay
      delay(3000);
      cleaning = false;

    }


  }
}



void vendingMachine() {         //The Vending state machine

  lcd.clear();

  while (1) {


    lcd.setCursor(0, 0);
    lcd.print("    >> Menu: << ");
    lcd.setCursor(0, 1);
    lcd.print(">> HOME :");

    switch (vmState) {              //Depending on the state
      case WAIT: {
          //State is WAIT
          // attachInterrupt(digitalPinToInterrupt(coinpin), coinInterrupt, RISING);
          pos = false;
          //while(pos){

          //attachInterrupt(digitalPinToInterrupt(coinpin), coinInterrupt, RISING);

          if ((millis() - instant_t6 > 45000) ) {
            //lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("    >> Menu: << ");
            lcd.setCursor(0, 1);
            lcd.print(">> HOME :");
            instant_t6 = millis();

            //
          }
          if (button0()) {          //If a dollar has been inserted
            vmState = FILL_1L;   //State is now GIVE_CHANGE
            pos = true;
          }
          if (button1()) {        //If a quarter has been inserted
            vmState = FILL_1L ;
            pos = true;
          }       //State is now TWETYFIVE
          //Get out of switch
          break;
        }
      case FILL_1L: {              //State is TWENTYFIFE
          //Run vend function
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(" >> FILL 1-L:<< ");
          lcd.setCursor(0, 1);
          lcd.print(" ENTER->GREEN ");
          instant_t6 = millis();
          pos = true;

          while (pos) {
            //display
            if (button0() ) {
              Fill(1);
              pos = false;
              vmState = WAIT;
              lcd.clear();
            }
            if (button1()) {
              vmState = FILL_L;
              // Serial.println("next:");
              pos = false;
              lcd.clear();
            }
            if (millis() - instant_t6 > 5000) {
              vmState = WAIT;
              refresh = 1;
              lcd.clear();
              pos = false;

            }
            //State is now calibrate
          }

          break;                         //Get out of switch
        }
      case FILL_L: {
          //Run vend function
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(" >> FILL-500ml:<< ");
          lcd.setCursor(0, 1);
          lcd.print(" ENTER->GREEN ");
          instant_t6 = millis();
          pos = true;

          while (pos) {
            //display
            if (button0() ) {
              Fill(0.5);
              pos = false;
              lcd.clear();
              vmState = WAIT;
            }
            if (button1()) {
              vmState = FILL_Q;
              // Serial.println("next:");
              pos = false;
              lcd.clear();
            }
            if (millis() - instant_t6 > 5000) {
              vmState = WAIT;
              refresh = 1;
              lcd.clear();
              pos = false;

            }
            //State is now calibrate
          }

          break;                         //Get out of switch
        }
      case FILL_Q: {
          //Run vend function
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(" >> FILL-250ml:<< ");
          lcd.setCursor(0, 1);
          lcd.print(" ENTER->GREEN ");
          instant_t6 = millis();
          pos = true;

          while (pos) {
            //display
            if (button0() ) {
              Fill(0.25);
              pos = false;
              lcd.clear();
              vmState = WAIT;
            }
            if (button1()) {
              vmState = DRAIN;
              // Serial.println("next:");
              pos = false;
              lcd.clear();
            }
            if (millis() - instant_t6 > 5000) {
              vmState = WAIT;
              refresh = 1;
              lcd.clear();
              pos = false;

            }
            //State is now calibrate
          }

          break;                         //Get out of switch
        }

      case DRAIN: {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(" >> DRAIN:<< ");
          lcd.setCursor(0, 1);
          lcd.print(" ENTER->GREEN ");
          instant_t6 = millis();
          pos = true;
          while (pos) {
            //display
            if (button0() ) {
              Drain();
              vmState = WAIT;
              pos = false;
              lcd.clear();
            }
            if (button1()) {
              vmState = CALIBRATE;
              // Serial.println("next:");
              pos = false;
              lcd.clear();
            }
            if (millis() - instant_t6 > 5000) {
              vmState = WAIT;
              lcd.clear();
              pos = false;

            }
            //State is now calibrate
          }

          //               //State is now wait
          break;
        }

      case CALIBRATE: {                //State is Calibrate
          //Serial.println("CALIBRATE:");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(" >> CALIBRATE:<< ");
          lcd.setCursor(0, 1);
          lcd.print(" ENTER->GREEN ");
          instant_t6 = millis();
          pos = true;

          while (pos) {
            //display
            if (button0() ) {
              calib();
              vmState = WAIT;
              pos = false;
              lcd.clear();
            }
            if (button1()) {
              vmState = WAIT;//State is now RESET
              // Serial.println("next:");
              pos = false;
              lcd.clear();
            }
            if (millis() - instant_t6 > 5000) {
              vmState = WAIT;

              lcd.clear();
              pos = false;

            }
            //State is now calibrate
          }
          break;                          //Get out of switch
        }
    }
  }
}
//initialize scale

//HX711 scale;

void setup() {
  //initialise LCD
  lcd.begin(COLUMNS, ROWS, LCD_5x8DOTS);

  Serial.begin(9600);
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("Powered by"));
  lcd.setCursor(0, 1);
  lcd.print(F(" Hewbe"));
  //DDRB |= (1<<PB5);
  //PORTB &= ~(1<<PB5);

  pinMode(PMP, OUTPUT);
  digitalWrite(PMP, LOW);


  //pulse_count = 0;
  EEPROM.get(70, m_clb);

    lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("1"));

  //  Keyboard.begin();
  pulse_begin = 0;
  last_state = 0;
  min_pulse_width = 15;
  max_pulse_width = 50;
  debounce_speed = 15;
  post_pulse_pause = 100;
  pulse_end = 0;

  pinMode(14, INPUT_PULLUP );
  digitalWrite(14, HIGH);

  pinMode(15, INPUT_PULLUP );
  digitalWrite(15, HIGH);

  //HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(m_clb);
  scale.tare();
  
  delay(1000);
}

void loop() {
  Serial.println("Loop:");

  vendingMachine();  //Run the machine

}

ISR(TIMER2_COMPA_vect)
{
  TCCR2B = 0;        // stop timer counter
  TCNT2 = 0;         // clear timer counter
  EIFR = 1;          // clear pending inputPin interrupt
  EIMSK |= 1;        // enable inputPin interrupt
}
