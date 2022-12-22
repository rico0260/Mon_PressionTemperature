/*******************************************
 * 
 * Auteur : Eric H
 * Date : 20/12/2022
 * 
 *******************************************/
 #include <Arduino.h>

#include <Vcc.h>
//#include <LowPower.h>
//#include <VoltageReference.h>

// Enable debug prints to serial monitor
//#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RF24
//#define MY_RADIO_NRF5_ESB
//#define MY_RADIO_RFM69
//#define MY_RADIO_RFM95

//Options: RF24_PA_MIN, RF24_PA_LOW, (RF24_PA_HIGH), RF24_PA_MAX
//#define MY_RF24_PA_LEVEL RF24_PA_MAX

//Non car sur pile
//#define MY_REPEATER_FEATURE

//MY_RF24_CHANNEL par defaut 76
//Channels: 1 to 126 - 76 = Channel 77
#include <perso.h>

//#define MY_NODE_ID AUTO
#define MY_NODE_ID 20

//======== Ne pas utiliser cette fonction avec ce noeud ===========
//#define MY_RX_MESSAGE_BUFFER_FEATURE //for MY_RF24_IRQ_PIN
//Define this to use the IRQ pin of the RF24 module
//#define MY_RF24_IRQ_PIN (2)
//======== Ne pas utiliser cette fonction avec ce noeud ===========

#include <SPI.h>
#include <MySensors.h>  
#include <Wire.h>
#include <Adafruit_BMP085.h>

#define PILE_CHILD 0
#define TEMP_CHILD 1
#define BARO_CHILD 2
#define FORCAST_CHILD 3

// Wait times
#define LONG_WAIT 500
#define LONG_WAIT2 2000
#define SHORT_WAIT 50

#define SKETCH_NAME "Pressure Sensor"
#define SKETCH_VERSION "2.1"

unsigned long SLEEP_TIME = 60000; // Sleep time between reads (in µseconds)

Adafruit_BMP085 bmp = Adafruit_BMP085(); // Digital Pressure Sensor 

unsigned int VCC_TIME = 60; // minutes //* 60000; // temps en 60 x 1 minute
unsigned int VccCount = 100; //superieur a VCC_TIME pour retourner la premiere mesures 
float newVcc;
int newPct;
float lastVcc = -100.0;
int lastPct = -1;
// Attention changer le BOD à 1.8V voir déactivé (sinon arret à 2.7V car BOD à 2.7V)
// et voir aussi pour les alimentations des capteurs
const float VccMin = 2.0*1.0; // Minimum expected Vcc level, in Volts. Example for 2xAA Alkaline.
const float VccMax = 2.0*1.6; //1.614;  // Maximum expected Vcc level, in Volts. Example for 2xAA Alkaline.
const float VccCorrection = 3.215/3.15; // Measured Vcc by multimeter divided by reported Vcc
Vcc vcc(VccCorrection);

float lastPressure = -100.0;
float lastTemp = -100.0;
int lastForecast = -1;
int forecast;

//const char *weather[] = {"stable","sunny","cloudy","unstable","thunderstorm","unknown"};
const char *weather[] = {"stable","soleil","nuageux","instable","orageux","inconnu"};

int minutes;
float pressureSamples[7][6];
int minuteCount = -1;
bool firstRound = true;
float pressureAvg[7];
float dP_dt;
boolean metric = true;

bool first_message_sent = false;

MyMessage msgPILE(PILE_CHILD, V_VOLTAGE); // V_VOLTAGE
MyMessage msgTEMP(TEMP_CHILD, V_TEMP);
MyMessage msgPRESSION(BARO_CHILD, V_PRESSURE);
MyMessage msgPRESSIONPrefix(BARO_CHILD, V_UNIT_PREFIX);  // Custom unit message.
MyMessage msgPREVISION(FORCAST_CHILD, V_FORECAST);

int oldClkPr;

int sample(float pressure);

//void before()
//{
  // Optional method - for initialisations that needs to take place before MySensors transport has been setup (eg: SPI devices).
//}

void presentation()  
{ 
  Serial.print("===> Envoyer présentation du noeud : "); Serial.println(MY_NODE_ID);

  char sNoeud[] = STR(MY_NODE_ID);

  // Send the Sketch Version Information to the Gateway
  Serial.println("===> Présenter SketchInfo");
  Serial.print(SKETCH_NAME); Serial.print(" "); Serial.println(SKETCH_VERSION);
  sendSketchInfo(SKETCH_NAME, SKETCH_VERSION);
  wait(LONG_WAIT2);

  // Register all sensors to gw (they will be created as child devices)
  Serial.println("===> Presenting Childs");
  Serial.println("________________");
  
  //  Pile
  char sChild0[25];
  strcpy(sChild0, "myS ");
  strcat(sChild0, sNoeud);
  strcat(sChild0, " Tension pile");
  Serial.print("S_MULTIMETER: "); Serial.println(sChild0);
  present(PILE_CHILD, S_MULTIMETER, sChild0); 
  wait(LONG_WAIT2);

  // Temperature
  char sChild1[25];
  strcpy(sChild1, "myS ");
  strcat(sChild1, sNoeud);
  strcat(sChild1, " Temperature");
  Serial.print("S_TEMP: "); Serial.println(sChild1);
  present(TEMP_CHILD, S_TEMP, sChild1);
  wait(LONG_WAIT2);

  // Pression
  char sChild2[25];
  strcpy(sChild2, "myS ");
  strcat(sChild2, sNoeud);
  strcat(sChild2, " Pression");
  Serial.print("S_BARO: "); Serial.println(sChild2);
  present(BARO_CHILD, S_BARO, sChild2);
  wait(LONG_WAIT2);

  //Forcast 
  char sChild3[25];
  strcpy(sChild3, "myS ");
  strcat(sChild3, sNoeud);
  strcat(sChild3, " Forcast");
  Serial.print("S_BARO: "); Serial.println(sChild3);
  present(FORCAST_CHILD, S_BARO, sChild3);
  wait(LONG_WAIT2);

  // Get controller configuration
  metric = getControllerConfig().isMetric;
  #ifdef MY_DEBUG 
    Serial.print("Get Config: "); Serial.println(metric ? "Metric":"Imperial");
  #endif
}

void setup()
{
  //Serial.begin(115200);

  // Called once at startup, usually used to initialize sensors.
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    while (1) { }
  }

  //newVcc = readVcc();
  newVcc = vcc.Read_Volts();
  Serial.print("Batterie = "); Serial.print(newVcc); Serial.println(" mV");
  // Içi entre 2V a 3.2V pour 2 LR6 --> 0% a 100%
  lastPct = vcc.Read_Perc(VccMin, VccMax);
  Serial.print("Pourcentage batterie = "); Serial.print(lastPct); Serial.println(" %");

}

void loop() {
  
  //float pressure = bmp.readSealevelPressure(75)/100; // 75 niveau de la mer a modifier en fonction
  float pressure = bmp.readSealevelPressure(75)/100.0; // 75 niveau de la mer a modifier en fonction
  float temperature = bmp.readTemperature();
  
  forecast = sample(pressure);

  if (!metric) {
    // Convert to fahrenheit
    temperature = temperature * 9.0 / 5.0 + 32.0;
  }
  
  //Pour Home assistant
  if (!first_message_sent) {
    Serial.println("======> Sending initial value");
    //send(msgPRESSION.set(pressure, 1));
    //wait(LONG_WAIT2);    
    send(msgPRESSIONPrefix.set("hPa"));  // hPa = mbar - Set custom unit.
    wait(LONG_WAIT2);
  //   send(msgPILE.set(newVcc,3));
  //   wait(LONG_WAIT2);
  //   sendBatteryLevel(lastPct);
  //   wait(LONG_WAIT2);
  //   send(msgTEMP.set(temperature,1));
  //   wait(LONG_WAIT2);
  //   send(msgPRESSION.set(pressure,1));
  //   wait(LONG_WAIT2);
  //   send(msgPREVISION.set(weather[forecast]));
  //   wait(LONG_WAIT2);
    first_message_sent = true;
    Serial.println("======> Début du fonctionnement");
    //Serial.println("======> Start of operation");
  }
  
  #ifdef MY_DEBUG 
    Serial.print("Temperature = "); Serial.print(temperature); Serial.println(metric?" *C":" *F");
    Serial.print("Pressure = "); Serial.print(pressure); Serial.println(" hPa"); // ou milliBar
    Serial.print("weather = "); Serial.println(weather[forecast]);
    Serial.print("Pile = "); Serial.print(newVcc); Serial.println(" V");
  #endif

  if (temperature != lastTemp) {
    lastTemp = temperature;
    send(msgTEMP.set(temperature,1));
    //wait(SHORT_WAIT);
  }

  if (pressure != lastPressure) {
    lastPressure = pressure;
    send(msgPRESSION.set(pressure, 1));
    //wait(SHORT_WAIT);
  }

  if (forecast != lastForecast) {
    /*
    // DP/Dt explanation
    0 = "Stable Weather Pattern"
    1 = "Slowly rising Good Weather", "Clear/Sunny "
    2 = "Slowly falling L-Pressure ", "Cloudy/Rain "
    3 = "Quickly rising H-Press",     "Not Stable"
    4 = "Quickly falling L-Press",    "Thunderstorm"
    5 = "Unknown (More Time needed) 
    */  
    lastForecast = forecast;
    send(msgPREVISION.set(weather[forecast]));
    //wait(SHORT_WAIT);
  }
  
  //mesure de la batterie
  if (VccCount >= VCC_TIME) {
    //newVcc = readVcc();
    newVcc = vcc.Read_Volts();
    if (newVcc != lastVcc) {
      lastVcc = newVcc;
      send(msgPILE.set(lastVcc, 3));      // 2 décimales
      //wait(SHORT_WAIT);
      #ifdef MY_DEBUG
        Serial.print("Tension piles = "); Serial.print(lastVcc); Serial.println(" V");
      #endif
    }
    //pct = 100 * (lastVcc-2400) / 820; // 3280-2400=880mV  
    newPct = vcc.Read_Perc(VccMin, VccMax);
    if (newPct != lastPct) {
      lastPct = newPct;
      sendBatteryLevel(lastPct);
      //wait(SHORT_WAIT);
      #ifdef MY_DEBUG
        Serial.print("niveau batterie = "); Serial.print(lastPct); Serial.println(" %");
      #endif
    }
    VccCount = 0; // soit toute les heures
  }
  else {
    VccCount=VccCount+1;
  }

//  Serial.print("lastVcc = ");
//  Serial.print(newVcc);
//  Serial.println(" V");
  
//  delay(100);
//  // Changement de l'horloge interne : 8 MHz --> 1 MHz (voir datasheet ATMEL section 9.12.2)
//  oldClkPr = CLKPR;  // sauvegarde du registre de prescaler d'horloge CLKPR
//  CLKPR = 0x80;          // bit 7 de CLKPR à 1 et les autres à 0 pour indiquer un changement de prescaler
//  CLKPR = 0x03;          // division par 8 de l'horloge 8MHz --> 1MHz
//  //CLKPR = 0x04;          // division par 16 de l'horloge 16MHz --> 1MHz
  // on se met en sommeil
  sleep(SLEEP_TIME);
//  // Restauration de la fréquence originale à la sortie du mode veille
//  CLKPR = 0x80;          // bit 7 de CLKPR à 1 et les autres à 0 pour indiquer un changement de prescaler
//  CLKPR = oldClkPr;      // on restore l'ancien facteur prescaler
  
  //LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  
  // Enter idle state for 8 s with the rest of peripherals turned off
  // Each microcontroller comes with different number of peripherals
  // Comment off line of code where necessary

  // ATmega328P, ATmega168
  //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
  //              SPI_OFF, USART0_OFF, TWI_OFF);

  // ATmega32U4
  //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
  //		  TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);

  // ATmega2560
  //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF, 
  //		  TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF, 
  //		  USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  
}

void receive(const MyMessage &message) {

  if (message.isAck()) {
    #ifdef MY_DEBUG
      Serial.println("This is an ack from gateway");
      Serial.print("Type message "); Serial.println(message.type);
    #endif  
  }
  // else if (message.type == V_UNIT_PREFIX) {
  //   if (!first_message_sent) {
  //     Serial.println("Receiving initial value from controller");
  //     first_message_sent = true;
  //   }
  // }

}

int sample(float pressure) {
	// Algorithm found here
	// http://www.freescale.com/files/sensors/doc/app_note/AN3914.pdf
	if (minuteCount >= 180)
		minuteCount = 5;
        
 	minuteCount++;

  //From 0 to 5 min.
  if (minuteCount <= 5){
    pressureSamples[0][minuteCount] = pressure;
  }
  //From 30 to 35 min.
  if ((minuteCount >= 30) && (minuteCount <= 35)){
    pressureSamples[1][minuteCount - 30] = pressure;  
  }
  //From 55 to 60 min.
  if ((minuteCount >= 55) && (minuteCount <= 60)){
    pressureSamples[2][minuteCount - 55] = pressure;  
  }
  //From 90 to 95 min.
  if ((minuteCount >= 90) && (minuteCount <= 95)){
    pressureSamples[3][minuteCount - 90] = pressure;  
  }
  //From 115 to 119 min.
  if ((minuteCount >= 115) && (minuteCount <= 120)){
    pressureSamples[4][minuteCount - 115] = pressure;  
  }
  //From 150 to 155 min.
  if ((minuteCount >= 150) && (minuteCount <= 155)){
    pressureSamples[5][minuteCount - 150] = pressure;  
  }
  //From 175 to 180 min.
  if ((minuteCount >= 175) && (minuteCount <= 180)){
    pressureSamples[6][minuteCount - 175] = pressure;  
  }
        
	if (minuteCount == 5) {
		// Avg pressure in first 5 min, value averaged from 0 to 5 min.
		pressureAvg[0] = ((pressureSamples[0][0] + pressureSamples[0][1] 
                                  + pressureSamples[0][2] + pressureSamples[0][3]
                                  + pressureSamples[0][4] + pressureSamples[0][5]) / 6);
	} else if (minuteCount == 35) {
		// Avg pressure in 30 min, value averaged from 0 to 5 min.
		pressureAvg[1] = ((pressureSamples[1][0] + pressureSamples[1][1] 
                                  + pressureSamples[1][2] + pressureSamples[1][3]
                                  + pressureSamples[1][4] + pressureSamples[1][5]) / 6);
		float change = (pressureAvg[1] - pressureAvg[0]);
		if (firstRound) // first time initial 3 hour
			dP_dt = ((65.0 / 1023.0) * 2 * change); // note this is for t = 0.5hour
		else
			dP_dt = (((65.0 / 1023.0) * change) / 1.5); // divide by 1.5 as this is the difference in time from 0 value.
	} else if (minuteCount == 60) { 
		// Avg pressure at end of the hour, value averaged from 0 to 5 min.
		pressureAvg[2] = ((pressureSamples[2][0] + pressureSamples[2][1] 
                                  + pressureSamples[2][2] + pressureSamples[2][3]
                                  + pressureSamples[2][4] + pressureSamples[2][5]) / 6);
		float change = (pressureAvg[2] - pressureAvg[0]);
		if (firstRound) //first time initial 3 hour
			dP_dt = ((65.0 / 1023.0) * change); //note this is for t = 1 hour
		else
			dP_dt = (((65.0 / 1023.0) * change) / 2); //divide by 2 as this is the difference in time from 0 value
	} else if (minuteCount == 95) {
		// Avg pressure at end of the hour, value averaged from 0 to 5 min.
		pressureAvg[3] = ((pressureSamples[3][0] + pressureSamples[3][1] 
                                  + pressureSamples[3][2] + pressureSamples[3][3]
                                  + pressureSamples[3][4] + pressureSamples[3][5]) / 6);
		float change = (pressureAvg[3] - pressureAvg[0]);
		if (firstRound) // first time initial 3 hour
			dP_dt = (((65.0 / 1023.0) * change) / 1.5); // note this is for t = 1.5 hour
		else
			dP_dt = (((65.0 / 1023.0) * change) / 2.5); // divide by 2.5 as this is the difference in time from 0 value
	} else if (minuteCount == 120) {
		// Avg pressure at end of the hour, value averaged from 0 to 5 min.
		pressureAvg[4] = ((pressureSamples[4][0] + pressureSamples[4][1] 
                                  + pressureSamples[4][2] + pressureSamples[4][3]
                                  + pressureSamples[4][4] + pressureSamples[4][5]) / 6);
		float change = (pressureAvg[4] - pressureAvg[0]);
		if (firstRound) // first time initial 3 hour
			dP_dt = (((65.0 / 1023.0) * change) / 2); // note this is for t = 2 hour
		else
			dP_dt = (((65.0 / 1023.0) * change) / 3); // divide by 3 as this is the difference in time from 0 value
	} else if (minuteCount == 155) {
		// Avg pressure at end of the hour, value averaged from 0 to 5 min.
		pressureAvg[5] = ((pressureSamples[5][0] + pressureSamples[5][1] 
                                  + pressureSamples[5][2] + pressureSamples[5][3]
                                  + pressureSamples[5][4] + pressureSamples[5][5]) / 6);
		float change = (pressureAvg[5] - pressureAvg[0]);
		if (firstRound) // first time initial 3 hour
			dP_dt = (((65.0 / 1023.0) * change) / 2.5); // note this is for t = 2.5 hour
		else
			dP_dt = (((65.0 / 1023.0) * change) / 3.5); // divide by 3.5 as this is the difference in time from 0 value
	} else if (minuteCount == 180) {
		// Avg pressure at end of the hour, value averaged from 0 to 5 min.
		pressureAvg[6] = ((pressureSamples[6][0] + pressureSamples[6][1] 
                                  + pressureSamples[6][2] + pressureSamples[6][3]
                                  + pressureSamples[6][4] + pressureSamples[6][5]) / 6);
		float change = (pressureAvg[6] - pressureAvg[0]);
		if (firstRound) // first time initial 3 hour
			dP_dt = (((65.0 / 1023.0) * change) / 3); // note this is for t = 3 hour
		else
			dP_dt = (((65.0 / 1023.0) * change) / 4); // divide by 4 as this is the difference in time from 0 value
		pressureAvg[0] = pressureAvg[5]; // Equating the pressure at 0 to the pressure at 2 hour after 3 hours have past.
		firstRound = false; // flag to let you know that this is on the past 3 hour mark. Initialized to 0 outside main loop.
	}

	if (minuteCount < 35 && firstRound) //if time is less than 35 min on the first 3 hour interval.
		return 5; // Unknown, more time needed
	else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
		return 0; // Stable weather
	else if ((dP_dt > 0.05) && (dP_dt < 0.25))
		return 1; // Slowly rising HP stable good weather
	else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
		return 2; // Slowly falling Low Pressure System, stable rainy weather
	else if (dP_dt > 0.25)
		return 3; // Quickly rising HP, not stable weather
	else if (dP_dt < (-0.25))
		return 4; // Quickly falling LP, Thunderstorm, not stable
	else
		return 5; // Unknown
}

// long readVcc() {
//   // Read 1.1V reference against AVcc
//   // set the reference to Vcc and the measurement to the internal 1.1V reference
//   #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//     ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//   #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
//     ADMUX = _BV(MUX5) | _BV(MUX0);
//   #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
//     ADMUX = _BV(MUX3) | _BV(MUX2);
//   #else
//     ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//   #endif  
 
//   delay(2); // Wait for Vref to settle
//   ADCSRA |= _BV(ADSC); // Start conversion
//   while (bit_is_set(ADCSRA,ADSC)); // measuring
 
//   uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
//   uint8_t high = ADCH; // unlocks both
 
//   long result = (high<<8) | low;
 
//   result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
//   return result; // Vcc in millivolts
// }
