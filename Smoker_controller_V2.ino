#include <PID_v1.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_SSD1327.h>
#include <esp_now.h>
#include <WiFi.h>

// Define receiver MAC address
uint8_t broadcastAddress[] = {0xC8, 0xC9, 0xA3, 0xF9, 0xED, 0xA4};

// Used for software SPI
#define CLK 18
#define MOSI 23
#define MISO 19

// Used for software or hardware SPI
#define OLED_CS 22
#define OLED_DC 4
#define MEAT_TEMP_CS 32
#define AIR_TEMP_CS 33


// Used for I2C or SPI
#define OLED_RESET -1

// Define input pins
#define air_plus 16
#define air_minus 17
#define meat_plus 5
#define meat_minus 21
#define fan_toggle 12
#define screen_wake 14


// Define output pins
#define fan_pin 13

// define global variables
bool fan_state = true;      // set fan state to on(true)
bool screen_state = true;   // set screen state to be on at boot up
bool fan_override = false;    // fan override logic value
int Reset = 0, PreError = 0;
int min_air_temp = 190, max_air_temp = 450, min_meat_temp = 120, max_meat_temp = 250;
double air_set_temp = 250, air_temp, fan_duty_cycle = 1;
double meat_set_temp = 190, meat_temp;
double Kp = 4, Ki = .3, Kd = .1;           // basic values for PID loop
unsigned long debounce_time;
unsigned long sleep_wake_time = 0;
unsigned long screen_sleep_time = 180000;   // amount of time with no activity before screen goes to sleep (milliseconds)
unsigned long current_cycle_start_time = 0, current_cycle_end_time = 60000;
unsigned long reset_time = 45000;  // fan cycle reset in milliseconds
unsigned long fan_stop_time = 45000;


// software SPI
Adafruit_SSD1327 display(128, 128, MOSI, CLK, OLED_DC, OLED_RESET, OLED_CS);
Adafruit_MAX31865 meat_thermo = Adafruit_MAX31865(MEAT_TEMP_CS, MOSI, MISO, CLK); 
Adafruit_MAX31865 air_thermo = Adafruit_MAX31865(AIR_TEMP_CS, MOSI, MISO, CLK); 

PID myPID(&air_temp, &fan_duty_cycle, &air_set_temp, Kp, Ki, Kd, DIRECT);

//**************************************************************************************************************************
//**************************************************************************************************************************

void setup()   {                
  Serial.begin(9600);
  //while (! Serial) delay(100);
  Serial.println("SSD1327 OLED test");
  
  if ( ! display.begin(0x3D) ) {
     Serial.println("Unable to initialize OLED");
     while (1) yield();
  }
  delay(1000);
  display.clearDisplay();
  display.display();

  // set up pin modes
  pinMode(air_plus, INPUT);
  pinMode(air_minus, INPUT);
  pinMode(meat_plus, INPUT);
  pinMode(meat_minus, INPUT);
  pinMode(fan_toggle, INPUT);
  pinMode(screen_wake, INPUT);
  pinMode(fan_pin, OUTPUT);
  
  // Attach interrupt pins to function
  attachInterrupt(digitalPinToInterrupt(air_plus), set_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(air_minus), set_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(meat_plus), set_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(meat_minus), set_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(fan_toggle), set_pin, RISING);
  attachInterrupt(digitalPinToInterrupt(screen_wake), set_pin, RISING);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,255);

  // start thermal sensors
  meat_thermo.begin(MAX31865_3WIRE);
  air_thermo.begin(MAX31865_3WIRE);

  debounce_time = millis();

  reset_timer();

}

//**************************************************************************************************************************
//**************************************************************************************************************************

void loop() {
  //Serial.println("main loop");
  
  // check air temperature
  air_temp = air_thermo.temperature(100.0, 430.0) * 1.8 + 32; // rtd sensor
  meat_temp = meat_thermo.temperature(100.0, 430.0) * 1.8 + 32; // rtd sensor

  /*
   * Actual temperature 33 deg F for ice water (should be 32)
   * Actual temperature 207 deg F for boiling water (should be 212)
   */
//  Serial.print("air temp = ");
//  Serial.print(air_temp);
//  Serial.println(" deg F");
//
//  Serial.print("meat temp = ");
//  Serial.print(meat_temp);
//  Serial.println(" deg F");
//  if(fan_state) {
//    Serial.println("Fan On");
//  }
//  else {
//    Serial.println("Fan off");
//  }

  if (millis() > current_cycle_end_time) {
    reset_timer();
  }
  check_fan_state();
  check_screen_state();
  
  
  
  // check timer
  // reset timer if needed
  delay(100);
  // update the display
  display_update();
}

//**************************************************************************************************************************
//**************************************************************************************************************************

void display_update() {
  display.clearDisplay();
  if (screen_state) {
  
    // draw rectangle around frame
    display.drawRect(0,0,127,127, SSD1327_WHITE);
    display.drawRect(1,1,125,125, SSD1327_WHITE);
  
    // draw countdown rectangle
    int bar_length = 127 * (current_cycle_end_time - millis()) / reset_time;
    display.fillRect(0,0, bar_length, 10, SSD1327_WHITE);
    
    // Fill in static text
    // Actual air temperature
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(5,15);
    display.print("T-Air");
    display.setTextSize(2);
    display.setCursor(5,25);
    int disp_air_temp = air_temp;
    display.print(disp_air_temp);
    display.print("F");

    // Set air temperature
    display.setTextSize(1);
    display.setCursor(5,45);
    display.print("Air-set");
    display.setTextSize(2);
    display.setCursor(5,55);
    int disp_air_set_temp = air_set_temp;
    display.print(disp_air_set_temp);
    display.print("F");

    // actual meat temperature
    display.setTextSize(1);
    display.setCursor(65,15);
    display.print("T-meat");
    display.setTextSize(2);
    display.setCursor(65,25);
    int disp_meat_temp = meat_temp;
    display.print(disp_meat_temp);
    display.print("F");

    // meat set temperature
    display.setTextSize(1);
    display.setCursor(65,45);
    display.print("Meat-set");
    display.setTextSize(2);
    display.setCursor(65,55);
    int disp_meat_set_temp = meat_set_temp;
    display.print(disp_meat_set_temp);
    display.print("F");

    // duty cycle
    display.setCursor(5,75);
    display.setTextSize(1);
    display.print("Duty Cycle");
    display.setTextSize(2);
    display.setCursor(5,85);
    int disp_duty_cycle = fan_duty_cycle * 100 / 255;
    display.print(disp_duty_cycle);
    display.print("%");

    // display fan state
    display.setTextSize(2);
    display.setCursor(5,105);
    if (fan_override) {
        // fan overridden
        display.setTextColor(SSD1327_WHITE);
        display.print(" FAN O/R ");
    }
    else if (fan_state) {
      display.setTextColor(SSD1327_BLACK, SSD1327_WHITE);
      display.print(" Fan on ");
      display.setTextColor(SSD1327_WHITE);
    }
    else {
      display.setTextColor(SSD1327_WHITE);
      display.print(" Fan off ");
    }

    // fill in updated text

  }
  // force display update
  //Serial.println("display update loop");
  display.display();
  
}


//**************************************************************************************************************************
//**************************************************************************************************************************

void reset_timer() {
  // reset the timer here
  current_cycle_start_time = millis();
  current_cycle_end_time = current_cycle_start_time + reset_time;
  // set the fan duty cycle with PID
  
  myPID.Compute();
  if (air_temp - air_set_temp > 2) {
    fan_duty_cycle = 0;
  }
  Serial.println(" ");
  Serial.println(" ");
  Serial.print("Fan duty Cycle = ");
  Serial.print(100 * fan_duty_cycle / 255);
  Serial.println("%");
  Serial.print("Air temperature: ");
  Serial.println(air_temp);
  Serial.print("Air Set temperature: ");
  Serial.println(air_set_temp);
  Serial.println(" ");
  Serial.println(" ");
  fan_stop_time = current_cycle_start_time + fan_duty_cycle * reset_time / 255;
}

//**************************************************************************************************************************
//**************************************************************************************************************************

void check_fan_state() {
  // check if the fan should be running
  if(fan_override) {
    fan_state = false;
    digitalWrite(fan_pin, LOW);

  }
  else if(millis() < fan_stop_time) {
    // fan is on
    fan_state = true;
    digitalWrite(fan_pin, HIGH);

  }
  else {
    // fan is off
    fan_state = false;
    digitalWrite(fan_pin, LOW);

  }

  
}

//**************************************************************************************************************************
//**************************************************************************************************************************

void check_screen_state() {
  unsigned long screen_time_delta = millis() - sleep_wake_time;
  if (screen_time_delta > screen_sleep_time) {
    screen_state = false;
  }
  else {
    screen_state = true;
  }
}


//**************************************************************************************************************************
//**************************************************************************************************************************

// define interrupt functions
void set_pin() {

  unsigned long debounce_delta = millis() - debounce_time;
  debounce_time = millis();
  if(debounce_delta > 300) {
    if(digitalRead(air_plus)) {
      // Increase the set air temperature by 5 degrees
      air_set_temp = air_set_temp + 5;
    }
    if(digitalRead(air_minus)) {
      
      // Decrease the set air temperature by 5 degrees
      air_set_temp = air_set_temp - 5;
    }
    if(digitalRead(meat_plus)) {
      
      // Increase the set air temperature by 5 degrees
      meat_set_temp = meat_set_temp + 5;
    }
    if(digitalRead(meat_minus)) {
      
      // decrease the set air temperature by 5 degrees
      meat_set_temp = meat_set_temp - 5;
    }
    
    if(digitalRead(fan_toggle)) {
      // toggle the fan state
      if(fan_override){ 
        fan_override = false;
        Serial.println("Fan not overriden"); 
      }
      else { 
        fan_override = true; 
        Serial.println("Fan overriden");
      }
    }
    if(digitalRead(screen_wake)) {
      // wake up the screen
      screen_state = true;
      // reset screen timer
      sleep_wake_time = millis();
    }
  }

  // Make sure set temperatures are all in their desired ranges
  if(air_set_temp > max_air_temp) { air_set_temp = max_air_temp; }
  if(air_set_temp < min_air_temp) { air_set_temp = min_air_temp; }
  if(meat_set_temp > max_meat_temp) { meat_set_temp = max_meat_temp; }
  if(meat_set_temp < min_meat_temp) { meat_set_temp = min_meat_temp; }
}
