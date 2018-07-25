/*****************************************************************************
* ej's o2 oled analyzer - v0.21
* http://ejlabs.net/arduino-oled-nitrox-analyzer
*
* License
* -------
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*****************************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADS1015.h>
#include <EEPROM.h>
#include <RunningAverage.h>

#define RA_SIZE 20
RunningAverage RA(RA_SIZE);

Adafruit_ADS1115 ads(0x48);

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

const int buttonPin=2; // push button
const int buzzer = 9; // buzzer
const int ledPin = 13; // led

double calibrationv;
float multiplier;

const int cal_holdTime = 2; // 2 sec button hold to calibration
const int mod_holdTime = 3; // 3 sec hold to po2 mod change
const int max_holdtime = 4; // 4 sec hold to reset max o2 result

long millis_held;    // How long the button was held (milliseconds)
long secs_held;      // How long the button was held (seconds)
long prev_secs_held; // How long the button was held in the previous check
byte previous = HIGH;
unsigned long firstTime; // how long since the button was first pressed 
int active = 0;
double result_max = 0;

/*
 Calculate MOD (Maximum Operating Depth)
*/
float max_po1 = 1.30;
const float max_po2 = 1.60;
float cal_mod (float percentage, float ppo2 = 1.4) {
  return 10 * ( (ppo2/(percentage/100)) - 1 );
}

void beep(int x=1) { // make beep for x time
  //digitalWrite(ledPin, HIGH); // led blink disable for battery save
  for(int i=0; i<x; i++) {    
      tone(buzzer, 2800, 100);
      delay(200);    
  }
  //digitalWrite(ledPin, LOW);
  noTone(buzzer);
}

void read_sensor(int adc=0) {  
  int16_t millivolts = 0;
  millivolts = ads.readADC_Differential_0_1();
  RA.addValue(millivolts);
}

void setup(void) {  

	//Serial.begin(9600);

  /* power saving stuff for battery power */
  // Disable ADC
  // ADCSRA = 0;
  // Disable the analog comparator by setting the ACD bit
  // (bit 7) of the ACSR register to one.
  // ACSR = B10000000;
  // Disable digital input buffers on all analog input pins
  // DIDR0 = DIDR0 | B00111111;

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  
  ads.setGain(GAIN_TWO);
  multiplier = 0.0625F;
  ads.begin(); // ads1115 start
  
  pinMode(buttonPin,INPUT_PULLUP);  
  
  RA.clear();
  for(int cx=0; cx<= RA_SIZE; cx++) {
     read_sensor(0);
  }
    
  calibrationv = EEPROMReadInt(0);  
  if (calibrationv < 100) {
    calibrationv = calibrate(0);
  }
  
  beep(1);
}

void EEPROMWriteInt(int p_address, int p_value)
     {
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);

     EEPROM.write(p_address, lowByte);
     EEPROM.write(p_address + 1, highByte);
     }

unsigned int EEPROMReadInt(int p_address)
     {
     byte lowByte = EEPROM.read(p_address);
     byte highByte = EEPROM.read(p_address + 1);

     return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
     }

int calibrate(int x) {
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);  
  display.setTextSize(2);
  display.print(F("Calibrate"));
  display.display();
    
  //RA.clear();
  double result;  
  for(int cx=0; cx<= RA_SIZE; cx++) {
    read_sensor(0);
  }
  result = RA.getAverage();
  result = abs(result);
  EEPROMWriteInt(x, result); // write to eeprom

  beep(1);
  delay(1000);
  active = 0;
  return result;
}

void analysing(int x, int cal) {
  double currentmv=0;
  double result;
  double mv = 0.0;

  read_sensor(0);
  currentmv = RA.getAverage();
  currentmv = abs(currentmv);
  
  result = (currentmv / cal) * 20.9;
  if (result > 99.9) result = 99.9;
  mv = currentmv * multiplier;
 
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  if (mv < 0.02 || result <= 0) {
     display.setTextSize(2);
     display.println(F("Sensor"));
     display.print(F("Error!"));
  } else {
    display.setTextSize(4);
    display.print(result,1);
    display.println(F("%"));

    if (result >= result_max) {
      result_max = result;
    }
    
    display.setTextSize(1);
    display.setCursor(0,31);
    display.setTextColor(BLACK, WHITE);    
    display.print(F("Max "));
    display.print(result_max,1);
    display.print(F("%   "));    
    //display.setCursor(75,31);
    display.print(mv,2);    
    display.print(F("mv"));
     
    if (active % 4) {
      display.setCursor(115,29);
      display.setTextColor(WHITE);
      display.print(F("."));
    }  
    
    display.setTextColor(WHITE);
    display.setCursor(0,40);
    display.print(F("pO2 "));
    display.print(max_po1,1);
    display.print(F("/"));
    display.print(max_po2,1);
    display.print(F(" MOD"));

    display.setTextSize(2);
    display.setCursor(0,50);
    display.print(cal_mod(result,max_po1),1);
    display.print(F("/"));
    display.print(cal_mod(result,max_po2),1);
    display.print(F("m "));
    
    // menu
    if (secs_held < 5 && active > 16) {
      display.setTextSize(2);
      display.setCursor(0,31);
      display.setTextColor(BLACK, WHITE);      
      if (secs_held >= cal_holdTime && secs_held < mod_holdTime) {
        display.print(F("   CAL    "));
      }
      if (secs_held >= mod_holdTime && secs_held < max_holdtime) {
        display.print(F("   PO2    "));
      }
      if (secs_held >= max_holdtime && secs_held < 10) {
        display.print(F("   MAX    "));
      }     
    }  

  }
  display.display();
}

void lock_screen(long pause = 5000) {
  beep(1);
  display.setTextSize(1);
  display.setCursor(0,31);  
  display.setTextColor(0xFFFF, 0);
  display.print(F("                "));
  display.setTextColor(BLACK, WHITE);
  display.setCursor(0,31);
  display.print(F("======= LOCK ======="));
  display.display();
  for (int i = 0; i < pause; ++i) {   
    while (digitalRead(buttonPin) == HIGH) {
      }
   }
   active = 0;
}

void po2_change() {  
  if (max_po1 == 1.3) max_po1 = 1.4;
  else if (max_po1 == 1.4) max_po1 = 1.5;
  else if (max_po1 == 1.5) max_po1 = 1.3;
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);  
  display.setTextSize(2);
  display.println(F("pO2 set"));
  display.print(max_po1);
  display.display();
  beep(1);   
  delay(1000);
  active = 0;  
}

void max_clear() {
  result_max = 0;
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0,0);  
  display.setTextSize(2);
  display.println(F("Max result"));
  display.print(F("cleared"));
  display.display();
  beep(1);   
  delay(1000);
  active = 0;
}

void loop(void) {

  int current = digitalRead(buttonPin);
 
  if (current == LOW && previous == HIGH && (millis() - firstTime) > 200) {
    firstTime = millis();
    active = 17;
  }

  millis_held = (millis() - firstTime);
  secs_held = millis_held / 1000;

  if (millis_held > 2) {
    if (current == HIGH && previous == LOW) {
      if (secs_held <= 0) {
        lock_screen();
      }
      if (secs_held >= cal_holdTime && secs_held < mod_holdTime) {        
        calibrationv = calibrate(0);
      }
      if (secs_held >= mod_holdTime && secs_held < max_holdtime) {
        po2_change();
      }
      if (secs_held >= max_holdtime && secs_held < 10) {
        max_clear();
      }
    }
  }

  previous = current;
  prev_secs_held = secs_held;
  
  analysing(0,calibrationv);
  delay(200);
    
  active++;
}

