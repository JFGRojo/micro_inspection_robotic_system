#include <FastLED.h>
#define NUM_LEDS 34
#define MODE_LED 13
#define LASER_PIN 24
#define TEMP_PIN A22
#define TEMP_PIN_2 A21
#define PRESS_PIN 23
#define RH_PIN 25
#define CONTROL_PERIOD 100000 //us
//Teensy 3.6 SPI: Data Pin: 11 || SCK Pin: 13

CRGB leds[NUM_LEDS];
int Ref=0;

// Interruption handling
IntervalTimer control_time; //Interruption variable

float temp = 0.0;
float temp_2 = 0.0;
float pressure = 0.0;
float RH = 0.0;
void setup() 
{
  pinMode(A22, INPUT); // Temp TMP35
  pinMode(23, INPUT); // Press
  pinMode(25, INPUT); // RH 
  pinMode(A21, INPUT); // Temp RH
  pinMode(MODE_LED, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);  

  FastLED.addLeds<APA102,BGR>(leds, NUM_LEDS); //If using SPI pins
  //FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN>(leds, NUM_LEDS); If using my own pins
  fill_solid( leds, 34, CRGB::Black); //Turn Off leds

        fill_rainbow( leds,34,1,5);
        FastLED.show();
        delay(1000);               // wait for a second
        fill_solid( leds, 34, CRGB::Black); //Turn Off leds
        FastLED.show();

        digitalWrite(LASER_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        digitalWrite(MODE_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(1000);               // wait for a second
        digitalWrite(LASER_PIN, LOW);    // turn the LED off by making the voltage LOW
        digitalWrite(MODE_LED, LOW);   // turn the LED on (HIGH is the voltage level)
        delay(1000);  
        digitalWrite(LASER_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        digitalWrite(MODE_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(1000);               // wait for a second
        digitalWrite(LASER_PIN, LOW);    // turn the LED off by making the voltage LOW
        digitalWrite(MODE_LED, LOW);   // turn the LED on (HIGH is the voltage level)
        delay(1000);
        
  Serial.begin(115200);
  while (!Serial)
  {
      //Wait Serial port to connect. Needed for native USB
  }
  control_time.begin(control, CONTROL_PERIOD); //Calls control function each CONTROL_PERIOD us<-- Last thing on setup
}

void control (void)
{
  read_temperature();
  read_pressure();
  read_humidity();
}

void loop() 
{
  	if (Serial.available() > 0) // Check serial buffer for avaliable bytes
  	{
      String buff = "";
    	buff = Serial.readStringUntil('\n');
    	if (buff == 'k')
    	{
        fill_rainbow( leds,34,1,5);
      	digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
        digitalWrite(MODE_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
        FastLED.show();
        delay(1000);               // wait for a second
        
        digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
        digitalWrite(MODE_LED, LOW);   // turn the LED on (HIGH is the voltage level)
        delay(1000);
        fill_solid( leds, 34, CRGB::Black); //Turn Off leds
    	}
      else if (buff == 'r')
      {
        fill_solid( leds, 34, CRGB::Red);
        Serial.println("LIGHT RED");
      }
      else if (buff == 'b')
      {
        fill_solid( leds, 34, CRGB::Blue);
        Serial.println("LIGHT BLUE");
      }
      else if (buff == 'g')
      {
        fill_solid( leds, 34, CRGB::Green);
        Serial.println("LIGHT GREEN");
      }
      else if (buff == 'w')
      {
        fill_solid( leds, 34, CRGB::White);
        Serial.println("LIGHT WHITE");
      }
      else if (buff == 'o')
      {
        fill_solid( leds, 34, CRGB::Black);
        Serial.println("LIGHT OFF");
      }
      else if (buff == 'l')
      {
      digitalWrite(LASER_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      Serial.println("LASER ON");
      }
      else if (buff == 'n')
      {
      digitalWrite(LASER_PIN, LOW);   // turn the LED on (HIGH is the voltage level)
      Serial.println("LASER OFF");
      }
      else if (buff == 's')
      {
      digitalWrite(LASER_PIN, LOW);   // turn the LED on (HIGH is the voltage level)
      Serial.println("SENSOR STATUS");
      Serial.println(temp);
      Serial.println(temp_2);
      Serial.println(pressure);
      Serial.println(RH);
      }
      else if (buff == 'd')
      {
      Serial.end();;   // turn the LED on (HIGH is the voltage level)
      Serial.println("SERIAL CLOSE");
      //break;
      }
      else if (buff == 'h')
      {
      Serial.println("===== COMMAND LIST =====");
      Serial.println("h->Help");
      Serial.println("d->Close");
      Serial.println("k->Connection Test");
      Serial.println("l->Laser on");
      Serial.println("n->Laser off");
      Serial.println("o->Light off");
      Serial.println("w->White light");
      Serial.println("r->Red light");
      Serial.println("b->Blue light");
      Serial.println("g->Green light");
      Serial.println("s->Sensor State");
      Serial.println("===== ----------- =====");
      }
      else
      {
        Ref = buff.toFloat();
        FastLED.setBrightness(Ref);
      }
      FastLED.show();
  	}
}

void read_temperature (void)
{
  temp = (int)((analogRead(A22) * 3300 / 1023.0) / 10); // Ambient_temp
}
void read_pressure (void)
{
  pressure = ((analogRead(23) / 1023.0) * 3300) / 3; // Ambient_pressure
}
void read_humidity(void)
{
  temp_2 = ((analogRead(23) / 1023.0) * 3300) / 15.1; // Ambient_pressure
  RH = ((analogRead(23) / 1023.0) * 3300) / 26.8; // Ambient_pressure
}
