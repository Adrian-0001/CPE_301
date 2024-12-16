#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>

#define RDA 0x80
#define TBE 0x20
#define threshold 20

const int stepsPerRevolution = 2038;  // change this to fit the number of steps per revolution

// initialize the stepper library on pins :
Stepper myStepper(stepsPerRevolution, 47,46,45,444);

// UART Pointers
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int  *)0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;
// GPIO Pointers
volatile unsigned char *portB     = (unsigned char*) 0x25;
volatile unsigned char *ddr_b     = (unsigned char*) 0x24;
volatile unsigned char *portE     = (unsigned char*) 0x2E;
volatile unsigned char *ddr_e     = (unsigned char*) 0x2D;
volatile unsigned char *pin_e     = (unsigned char*) 0x2C;
volatile unsigned char *portC     = (unsigned char*) 0x28;
volatile unsigned char *ddr_c     = (unsigned char*) 0x27;
volatile unsigned char *pin_c     = (unsigned char*) 0x26;
volatile unsigned char *portD     = (unsigned char*) 0x2B;
volatile unsigned char *ddr_d     = (unsigned char*) 0x2A;
volatile unsigned char *pin_d     = (unsigned char*) 0x29;
// Timer Pointers
volatile unsigned char *myTCCR1A  = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B  = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C  = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1  = (unsigned char *) 0x6F;
volatile unsigned char *myTIFR1   = (unsigned char *) 0x36;
volatile unsigned int  *myTCNT1   = (unsigned  int *) 0x84;
// Analog Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//global variables
double waterLevel = -1, temp = -1, hum = -1;
int prevState = 0, delayCount = 11250, prevPotState = 0;
bool needClear = true;

//state of the refrigerator
enum State{
  disabled = 0,
  idle = 1,
  running = 2,
  error = 3
};
State state = disabled;

//setup temperature sensor
DHT dht(2, DHT11);

//LCD pin used
const int rs = 10, en = 9, d4 = 8, d5 = 7, d6 = 6, d7 = 5;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//global ticks counter
void setup() 
{ 
  //set PE5 (fan) to output             
  *ddr_e |= 0x20;

  //set PC0 to (reset button) input
  *ddr_c &= ~(0x01);
  *portC &= ~(0x01);

  //set PB2 (start button) to input
  *ddr_d &= ~(0x01 << 3); 
  *portD |= 0x01 << 3;

  // set PB5,6,7 (RGB) to output
  *ddr_b |= 0x01;
  *ddr_b |= 0x20;
  *ddr_b |= 0x40;
  *ddr_b |= 0x80;

  //interrupt for the start/stop button 
  attachInterrupt(digitalPinToInterrupt(18), start, RISING);

  //Start the UART
  U0Init(9600);
  //setup the ADC
  adc_init();
  //real time clock setup

  //setup for the temperature sensor
  dht.begin();
  //setup for LCD
  lcd.begin(16,2);
  //set motor speed 
  myStepper.setSpeed(10);
}

void loop() 
{
  //control stepper motor and display if position changed
  int potState = adc_read(5);
  myStepper.step(potState-prevPotState);
  if(!(potState >= prevPotState-100 &&  potState <= prevPotState+100)){
    String potDisplay = "Potentiometer position changed at: ";
    for(int i = 0; i < potDisplay.length(); ++i){
      U0putChar(potDisplay[i]);
    }
    String date;
    for(int i = 0; i < date.length(); ++i){
      U0putChar(date[i]);
    }
    U0putChar('\n');
  }

  //display change of state
  if(state != prevState){
    prevState = state;
  }

  //change what states do 
  switch(state){
    case 1://idle
      //updates temp/hum and display them on LCD
      temp = dht.readTemperature();
      hum = dht.readHumidity();
      my_delay(125);
      delayCount++;
      if(delayCount >= 11250){
        displayTempAndHum();
        delayCount = 0;
          lcd.print("Enabled");  // Display "Disabled" when in disabled state

      }

      //update water level
      waterLevel = adc_read(0);

      //change state base on temperature
      if(temp > threshold){
        state = running;
        needClear = true;
      }

      //change state base on water level
      if(waterLevel <= 100){
        state = error;
        needClear = true;
      }
      
      //change LED to green
      *portB &= ~(0x01 << 0);
      *portB &= ~(0x01 << 5);
      *portB |= (0x01 << 6);
      *portB &= ~(0x01 << 7);

      break;
    case 2://running
      //updates and displays temp and hum to LCD
      temp = dht.readTemperature();
      hum = dht.readHumidity();
      my_delay(125);
      delayCount++;
      if(delayCount >= 11250){
        displayTempAndHum();
        delayCount = 0;
      } 

      //update water level
      waterLevel = adc_read(0);


      //start the fan
      *portE |= (0x01 << 5);

      //change state base on temperature
      if(temp <= threshold){
        state = idle;
        *portE &= ~(0x01 << 5);
        needClear = true;
      }

      //change state base on water level
      if(waterLevel < 100){
        state = error;
        *portE &= ~(0x01 << 5);
        needClear = true;
      }
      
      //change LED to blue
      *portB &= ~(0x01 << 0);
      *portB |= (0x01 << 5);
      *portB &= ~(0x01 << 6);
      *portB &= ~(0x01 << 7);
      break;
    case 3://error
      //prints error message 
      if(needClear){
        lcd.clear();
        needClear = false;
      }

      lcd.setCursor(0, 0);
      lcd.print("ERROR");
      lcd.setCursor(0, 1);
      lcd.print("WATER LEVEL LOW");

      //reset the state
      if((*pin_c & 0x01)){
        state = idle;
        needClear = true;
        temp = dht.readTemperature();
        hum = dht.readHumidity();
        displayTempAndHum();
      }

      //change LED to red
      *portB &= ~(0x01 << 0);
      *portB &= ~(0x01 << 5);
      *portB &= ~(0x01 << 6);
      *portB |= (0x01 << 7);
      break;
    case 0://disabled

      //change LED to yellow 
      *portB |= (0x01 << 0);
      *portB &= ~(0x01 << 5);
      *portB &= ~(0x01 << 6);
      *portB &= ~(0x01 << 7);
      break;
  }
  prevPotState = potState;
}

void start(){
  if(state == disabled){
    state = idle;
  }else{
    *portE &= ~(0x01 << 5);
    if(state == idle){
    }
    state = disabled;
  }
}

/*
 * UART FUNCTIONS
 */
void U0Init(unsigned int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return *myUCSR0A & RDA;
}

unsigned char U0getChar()
{
  while (!(*myUCSR0A & RDA)) ;
  return *myUDR0;
}

void U0putChar(unsigned char U0pdata)
{
  while (!(*myUCSR0A & TBE));
  *myUDR0 = U0pdata;
}


/* Analog Port functions*/
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void my_delay(unsigned int freq)
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  * myTCCR1A = 0x0;
  * myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV           
  *myTIFR1 |= 0x01;
}


void displayTempAndHum(){
  if(needClear){
    lcd.clear();
    needClear = false;
  }
  lcd.setCursor(0,0);
  lcd.print("Temp: " + (String)temp + char(223) + "C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: " + (String)hum);
}