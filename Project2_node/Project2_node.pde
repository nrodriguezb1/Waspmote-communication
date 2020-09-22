/*
    ------ Waspmote Pro Code Example --------

    Explanation: This is the basic Code for Waspmote Pro

    Copyright (C) 2016 Libelium Comunicaciones Distribuidas S.L.
    http://www.libelium.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Put your libraries here (#include ...)
//For BME280 (temperature, humidity, pressure)
#include <WaspSensorEvent_v30.h>
//Send packages
#include <WaspXBee802.h>
#include <WaspFrame.h>

//Variables
//For BME280
float temp;
char temp_char[4];
float humd;
char humd_char[3];
float pres;
char pres_char[5];
//For accelerometer
uint8_t status;
int x_acc;
int y_acc;
int z_acc;
//For battery level
int bat;
float volts_bat;
//PIR
uint8_t valuePIR = 0;
pirSensorClass pir(SOCKET_1);

//Xbee configuration
//PAN (Personal Area Network) Identifier
uint8_t  panID[2] = {0x22,0x44}; 
//Define frequency channel. Range: 0x0C - 0x17 (XBee-PRO)
uint8_t  channel = 0x0F;
//Define encryption mode: 1 -> enabled, 0 -> disabled
uint8_t encryptionMode = 0;
// Define the AES 16-byte Encryption Key
//char  encryptionKey[] = "KeyCristinasAndNathalia"; 

//Send packages
//Destination MAC adress
char RX_ADDRESS[] = "0013A200417EE523"; //MAC address of the WiFi_Pro board
//Define the Waspmote ID
char WASPMOTE_ID[] = "nodeWiFiCN";
uint8_t send_error;
//Buffers to send the information
char mybufferSensors[90] = "";
char mybufferFreeFall[30] = "";
char mybufferPIR[30] = "";
char mybufferBat[30] = "";

//Variables used to count time
unsigned long remain, start, next, passed, execTime, millisInit, millisNext;
unsigned int num_ints = 0; //Number of interruptions
const unsigned int CYCLE = 30;
char formatted_time[20];
boolean myRTC = true;
char* formato2;

//////////////////Methods
void read_values();
void print_values(float temperature, float humidity, float pressure, int accel_x, int accel_y, int accel_z, int battery); //Creo que podemos quitar el parámetro del PIR de esta función. No lo estamos mostrando
void delivery_data(char mybuffer);
void checkTXFlag(uint8_t send_error);

void setup(){

  ///////////////////////////////////////////////
  // 1. INIT XBEE
  //////////////////////////////////////////////
  xbee802.ON();
  
  ///////////////////////////////////////////////
  // 2. TURN ON THE USB, SENSORS...
  //////////////////////////////////////////////
  USB.ON();

  //Turn on sensors
  //Accelerometer
  ACC.ON();
  //Events board
  Events.ON();
  
  //Wait for PIR signal stabilization
  valuePIR = pir.readPirSensor();
  while (valuePIR == 1){
    USB.println(F("...wait for PIR stabilization"));
    delay(1000);
    valuePIR = pir.readPirSensor();    
  }
  
  ///////////////////////////////////////////////
  // 3. ENABLE INTERRUPTIONS 
  //////////////////////////////////////////////
  //Enable interruptions from the board (PIR interruption)
  Events.attachInt();

  //Enable interruption from accel: free-fall
  ACC.setFF();

  //Enable interruption from RTC
  RTC.ON();
  
  ///////////////////////////////////////////////
  // 4. Enable LEDs
  //////////////////////////////////////////////
  Utils.setLED(LED0, LED_OFF);
  Utils.setLED(LED1, LED_OFF);
  
  //Print a start message
  USB.println(F("Start program"));
  USB.println(F("-------------------------------"));
   // Setting time [yy:mm:dd:dow:hh:mm:ss]
  RTC.setTime("20:02:13:05:12:00:00");
  USB.print(F("Setting time: "));
  USB.println(RTC.getTime());
  //millisInit = millis();
}


void loop(){

  ///////////////////////////////////////
  // 1. Turn on sensors
  ///////////////////////////////////////
  Events.ON();
  ACC.ON();

  ///////////////////////////////////////
  //2. Read PIR
  //////////////////////////////////////
  //Read the PIR sensor
  valuePIR = pir.readPirSensor();

  
  ///////////////////////////////////////
  // 3. Go to deep sleep mode
  ///////////////////////////////////////

  //Interruption from RTC
  if(myRTC){ //myRTC value initialized to true
    USB.println(F("enter deep sleep"));
   //Save previous value to count time
    start = RTC.second; //Start time before deep sleep for 30 secs
   //USB.print("Start time: ");
    //USB.println(start);
    PWR.deepSleep("00:00:00:30", RTC_OFFSET, RTC_ALM1_MODE5, SENSOR_ON);
   //Restart: ON devices again
    USB.ON();
    xbee802.ON(); //To send information continuously!!!!
    USB.println(F("wake up\n"));
  }
  //Interruption by other causes
  else{

   next = RTC.second;
   //Obtain the time that has passed from the previous interruption
   passed = next - start;
   millisNext = millis();
   execTime = (millisNext-millisInit)/1000;
   //If the time passed is minor than the cycle time...
   if( passed+execTime < CYCLE){
    //Obtain the remain time that have to pass
    remain = (CYCLE-passed-execTime);
    //Configure the timeSleep
        if(remain < 10){
          sprintf(formatted_time, "00:00:00:0%lu", remain);
          }else{
        sprintf(formatted_time, "00:00:00:%lu", remain);
          }
    //Sleep until 30 secs
    USB.println(F("enter deep sleep"));
    PWR.deepSleep(formatted_time, RTC_OFFSET, RTC_ALM1_MODE5, SENSOR_ON);
    //Restart: ON devices again
    USB.ON();
    xbee802.ON(); //To send information continuously!!!!
    USB.println(F("wake up\n"));
   }else{
       USB.println(F("enter deep sleep"));
      PWR.deepSleep("00:00:00:01", RTC_OFFSET, RTC_ALM1_MODE5, SENSOR_ON);
      
   }//If it has passed more than 30 secs, we do nothing, RTC INT must have been raised

  }
  millisInit = millis();

  ///////////////////////////////////////
  // 4. Check Interruption Flags
  ///////////////////////////////////////
  //Check interruption from sensor board
  if(intFlag & SENS_INT){
    //Interruption not from the RTC
    myRTC = false;
    
    //Disable interruptions from the board
    Events.detachInt();

    //Load the interruption flag
    Events.loadInt();

    //In the case the interruption came from PIR
    if(pir.getInt()){
      USB.println(F("-----------------------------"));
      USB.println(F("  Interruption from PIR"));
      USB.println(F("-----------------------------"));
    }
    
    //Send warning to the edge board
    char* formato = "field1=PIRNode";
    sprintf(mybufferPIR, formato);
    USB.println(mybufferPIR);
    //Send data to RX_ADRESS
    send_error = xbee802.send(RX_ADDRESS, mybufferPIR);
    // check TX flag
    if(send_error == 0){
      USB.println(F("send ok"));    
    }else{
      USB.println(F("send error"));
    }
    
    //Switch on LED
    Utils.blinkRedLED(2000, 1);
    
    //Stabilisation: Read the sensor level
    valuePIR = pir.readPirSensor();
    
    while (valuePIR == 1){
      USB.println(F("...wait for PIR stabilization"));
      delay(1000);
      valuePIR = pir.readPirSensor();
    }
    
    // Clean the interruption flag
    intFlag &= ~(SENS_INT);
    
    // Enable interruptions from the board
    Events.attachInt();
  }
  //check free-fall interruption
  if(intFlag & ACC_INT){

    //Interruption not from the RTC
    myRTC = false;
    
    //Enable accelerometer
    ACC.ON();
    //Disable interruption from accelerometer
    ACC.unsetFF();
    
    //Print info
    USB.println(F("-----------------------------"));
    USB.println(F("  Free-fall interruption"));
    USB.println(F("-----------------------------"));
    
    //Send warning to the edge board
    char* formato = "field2=FreefallNode";
    sprintf(mybufferFreeFall, formato);
    USB.println(mybufferFreeFall);
    //Send data to RX_ADRESS
    send_error = xbee802.send(RX_ADDRESS, mybufferFreeFall);
    // check TX flag
    if(send_error == 0){
      USB.println(F("send ok"));    
    }else{
      USB.println(F("send error"));
    }
    
    //Clean interruption flag
    intFlag &= ~(ACC_INT);
    //Clean interruption pin
    PWR.clearInterruptionPin();
    
    //Enable interruption from the accelerometer again
    ACC.setFF();

    //Switch on LED
    Utils.blinkGreenLED(2000, 1);
  }

  //Check RTC interruption
  if(intFlag & RTC_INT){
    //Interruption from the RTC
    myRTC = true;
    
    //Enable RTC
    RTC.ON();

    //Print info
    USB.println(F("---------------------------------------"));
    USB.println(F("RTC interruption: sending information..."));
    USB.println(F("---------------------------------------"));
    
    //Send data to the edge board through 802.15.4
    send_sensors_data();
    //Clear interruption flag
    intFlag &=~(RTC_INT);
  }
}

  /////////////////////////////////////////////////////
  //  METHOD TO READ SENSOR VALUES
  /////////////////////////////////////////////////////
  void read_values(){
    //Read BME280 Values
    temp = Events.getTemperature();
    humd = Events.getHumidity();  
    pres = Events.getPressure()/100;//hPa

    //Read accel values
    //status = ACC.check();
    x_acc = ACC.getX();
    y_acc = ACC.getY();
    z_acc = ACC.getZ();

    //Read battery level
    bat = PWR.getBatteryLevel();
    //volts_bat = PWR.getBatteryVolts();
  }

  /////////////////////////////////////////////////////
  //  METHOD TO PRINT VALUES
  /////////////////////////////////////////////////////
  void print_values(float temperature, float humidity, float pressure, 
  int accel_x, int accel_y, int accel_z, int battery){
    
    USB.println(F("-----------------------------"));
    USB.println(F("       READ SENSORS"));
    USB.println(F("-----------------------------"));
    //BME280
    USB.print("Temperature: ");
    USB.printFloat(temperature, 1); //1 decimal
    USB.println(F(" Celsius"));
    USB.print("Humidity: ");
    USB.printFloat(humidity, 1); 
    USB.println(F(" %")); 
    USB.print("Pressure: ");
    USB.printFloat(pressure, 1); 
    USB.println(F(" Pa"));
    //Accelerometer
    USB.print("X_accel: ");
    USB.print(accel_x, DEC);
    USB.println(F(""));
    USB.print("Y_accel: ");
    USB.print(accel_y, DEC);
    USB.println(F(""));
    USB.print("Z_accel: ");
    USB.print(accel_z, DEC);
    USB.println(F(""));
    //Battery level
    USB.print(F("Battery Level: "));
    USB.print(battery,DEC);
    USB.println(F(" %"));
    //Print alert about the battery level
    if(battery<20){
      USB.println(F("Level of battery less than 20%"));
    }else{
      USB.println(F("Level of battery greater than 20%"));
    }
    
    USB.println("-----------------------------");  
  }

  /////////////////////////////////////////////////////
  //  METHOD TO SEND SENSORS DATA TO TE EDGE BOARD
  /////////////////////////////////////////////////////
  void send_sensors_data(){
    //1. Read values
    read_values();
    
    //2. Print values through serial port
    print_values(temp, humd, pres, x_acc, y_acc, z_acc, bat);
    
    //3. Send data to the edge board
    //Convert to a valid format -from float to char-
    dtostrf(temp, 1, 1, temp_char);
    dtostrf(humd, 1, 1, humd_char);
    dtostrf(pres, 1, 1, pres_char);
    
    //Configure data
    char* formato1 = "field1=%s&field2=%s&field3=%s&field4=%d&field5=%d&field6=%d&field7=%d";
    sprintf(mybufferSensors, formato1, temp_char, humd_char, pres_char, bat, x_acc, y_acc, z_acc);
    //Send data to RX_ADRESS
    send_error = xbee802.send(RX_ADDRESS, mybufferSensors);
    USB.println(mybufferSensors);
    // check TX flag
    checkTXFlag(send_error);
    //Send alert about the battery state
    if(bat<20){
      char* formato2 = "field3=LowBatNode";
      sprintf(mybufferBat, formato2);
      //Send data to RX_ADRESS
      send_error = xbee802.send(RX_ADDRESS, mybufferBat);
      USB.println(mybufferBat);
      // check TX flag
      checkTXFlag(send_error);
    }
  }
  
  /////////////////////////////////////////////////////
  //  METHOD TO PRINT THE RESULT OF THE SENDING
  /////////////////////////////////////////////////////
  void checkTXFlag(uint8_t send_error){
    
    if(send_error == 0){
      USB.println(F("send ok"));    
    }else{
      USB.println(F("send error"));
    }
  }








