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


//For BME280 (temperature, humidity, pressure)
#include <WaspSensorEvent_v30.h>
//Send packages
#include <WaspXBee802.h>
#include <WaspFrame.h>

//WIFI libraries
#include <WaspWIFI_PRO.h>
//MQTT libraries
#include <Countdown.h>
#include <FP.h>
#include <MQTTFormat.h>
#include <MQTTLogging.h>
#include <MQTTPacket.h>
#include <MQTTPublish.h>
#include <MQTTSubscribe.h>
#include <MQTTUnsubscribe.h>


// choose socket (SELECT USER'S SOCKET)
///////////////////////////////////////
uint8_t socket = SOCKET1;
///////////////////////////////////////

// choose TCP server settings
///////////////////////////////////////
char HOST[]        = "mqtt.te.upm.es"; //MQTT Broker
char REMOTE_PORT[] = "1883";  //MQTT
char LOCAL_PORT[]  = "3000";
///////////////////////////////////////

uint8_t wifi_error;
uint8_t wifi_status;
unsigned long wifi_previous;
uint16_t socket_handle = 0;

//Variables
//For BME280
float temp = 0.0;
char temp_char[4];
float humd = 0.0;
char humd_char[3];
float pres = 0.0;
char pres_char[5];
//For accelerometer
uint8_t status;
int x_acc;
int y_acc;
int z_acc;

//For battery level
int bat;
//float volts_bat;
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
// Define the AES 16-byte Encryption Key if needed
//char  encryptionKey[] = "KeyCristinasAndNathalia"; 

//Send packages

//Define the Waspmote ID
char WASPMOTE_ID[] = "nodeWiFiCN";
uint8_t send_error;
uint8_t error;
char mybufferSensors[90] = "";
char mybufferFreeFall[30] = "";
char mybufferPIR[30] = "";
char mybufferBat[30] = "";
char mybufferSensorsNode[90] = "";
char mybufferWarningNode[30] = "";

char* formato2;

//Receive messages from the node
char messagePayload[100] = "";
int messageLength, messageLength2, meLength, meLength2;

//Topics
char topicEdgeSensors[45] = "g7/channels/982358/publish/0SH6KDPRI6RV76VQ";
char topicNodeSensors[45] = "g7/channels/980415/publish/GV0KLGHRAVKQGTRX";
char topicAlarms[45] = "g7/channels/877265/publish/NLWCGZX7Q5ACBQC6";

//////////////////Methods
void read_values();
void print_values(float temperature, float humidity, float pressure, int accel_x, int accel_y, int accel_z, int battery);
void read_sensors_data();
void receiving_packets_node(int time_receiving_packets);
void send_MQTTdata(char send_to_topic[], char send_payload[]);
void switchOFF_wifi();

void setup(){

  ///////////////////////////////////////////////
  // Store Waspmote identifier in EEPROM memory
  //////////////////////////////////////////////
  frame.setID(WASPMOTE_ID);

  ///////////////////////////////////////////////
  // 1. INIT XBEE
  //////////////////////////////////////////////
  xbee802.ON();
  
  ///////////////////////////////////////////////
  // 2. TURN ON THE USB, SENSORS...
  //////////////////////////////////////////////
  USB.ON();

  //Turn on sensors
  ACC.ON();
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
  RTC.setAlarm1(0,0,0,30,RTC_OFFSET,RTC_ALM1_MODE2);
  ///////////////////////////////////////////////
  // 4. Enable LEDs
  //////////////////////////////////////////////
  Utils.setLED(LED0, LED_OFF);
  Utils.setLED(LED1, LED_OFF);
  
  //Print a start message
  USB.println(F("Start program"));
  USB.println(F("-------------------------------"));
}


void loop(){

  ///////////////////////////////////////
  // 1. Turn on sensors
  ///////////////////////////////////////
  Events.ON();
  ACC.ON();
 // xbee802.ON(); //To send information continuously!!!!
  ///////////////////////////////////////
  //2. Go to sleep
  //////////////////////////////////////
  USB.println(F("enter sleep"));
  PWR.sleep(WTD_2S, SENSOR_ON);
  USB.ON();
  xbee802.ON(); //To send information continuously!!!!
  USB.println(F("wake up\n"));
  

  ///////////////////////////////////////
  // 3. Time to interruptions
  ///////////////////////////////////////

  //Check RTC interruption
  if(intFlag & RTC_INT){
    
      RTC.ON();
      
      //Print info
      USB.println(F("---------------------------------------"));
      USB.println(F("RTC interruption: sending information..."));
      USB.println(F("---------------------------------------"));
        //Clear interruption flag
    intFlag &=~(RTC_INT);
    //1. Read sensor values
    read_sensors_data();
    //2. Send values through MQTT
    openWifi();

    if(messageLength > 0){
      send_MQTTdata(topicNodeSensors, mybufferSensorsNode);
      close_socket();
      messageLength = 0;
    }
    send_MQTTdata(topicEdgeSensors, mybufferSensors);
    close_socket();
    switchOFF_wifi();


    //Setting alarm again
    USB.println("RTC interruption established");
    RTC.setAlarm1(0,0,0,30,RTC_OFFSET,RTC_ALM1_MODE2);
  }
  //Check interruption from sensor board
  if(intFlag & SENS_INT){
    
    //Disable interruptions from the board
    Events.detachInt();

    //Load the interruption flag
    Events.loadInt();

    //In the case the interruption came from PIR
    if(pir.getInt()){
      USB.println(F("-----------------------------"));
      USB.println(F("  Interruption from PIR"));
      USB.println(F("-----------------------------"));
          //Switch on LED
    Utils.blinkRedLED(2000, 1);

    char* formato = "field4=PIREdge";
    sprintf(mybufferPIR, formato);

    USB.println(mybufferPIR);
    //Send data through MQTT
    openWifi();
    //Its data
    send_MQTTdata(topicAlarms, mybufferPIR);
    close_socket();
    switchOFF_wifi();
    //Stabilisation: Read the sensor level
    valuePIR = pir.readPirSensor();
    
    while (valuePIR == 1){
      USB.println(F("...wait for PIR stabilization"));
      delay(1000);
      valuePIR = pir.readPirSensor();
    }
    }
    

    
    // Clean the interruption flag
    intFlag &= ~(SENS_INT);
    
    // Enable interruptions from the board
    Events.attachInt();

    USB.println("RTC interruption established");
    RTC.setAlarm1(0,0,0,6,RTC_OFFSET,RTC_ALM1_MODE2);
  }
  
  //check free-fall interruption
  if(intFlag & ACC_INT){

    ACC.ON();
    //Disable interruption from accelerometer
    ACC.unsetFF();
    //Clean interruption flag
    intFlag &= ~(ACC_INT);
    //Print info
    USB.println(F("-----------------------------"));
    USB.println(F("  Free-fall interruption"));
    USB.println(F("-----------------------------"));
        //Switch on LED
    Utils.blinkGreenLED(2000, 1);
    
    char* formato = "field5=FreefallEdge";
    sprintf(mybufferFreeFall, formato);
    USB.println(mybufferFreeFall);

    //Send data to through MQTT

    openWifi();
    //Its data
    send_MQTTdata(topicAlarms, mybufferFreeFall);
    close_socket();
    switchOFF_wifi();

    
    //Enable interruption from the accelerometer again
    ACC.setFF();
    USB.println("RTC interruption established");
    RTC.setAlarm1(0,0,0,6,RTC_OFFSET,RTC_ALM1_MODE2);
  }
//Clean interruption pin
    PWR.clearInterruptionPin();
    

  ///////////////////////////////////////////////
  // 4. Time to receive messages from the node
  //////////////////////////////////////////////
  //The information received (if there is one)
  //is saved in messagePayload and messageLength
 
  
  xbee802.ON(); 
  USB.println(F("Receiving packets from the node"));
  receiving_packets_node(7000); 
  //Swtich on led if the edge receives an alarm
  //Free fall warning
  if(messageLength2 == 19){
    //Switch on LED
    Utils.blinkGreenLED(2000, 1);
    /*This code is to send MQTT message in case there is a FF interruption in Node. As the connection establishment takes a lot of time sometimes, the RTC is lost
    openWifi();
    //Its data
    send_MQTTdata(topicAlarms, mybufferWarningNode);
    close_socket();
    switchOFF_wifi();
    */
    messageLength2 = 0;
  }
  //PIR warning
  else if(messageLength2 == 14){
    //Switch on LED
    Utils.blinkRedLED(2000, 1);
    messageLength2 = 0;
    /*This code is to send MQTT message in case there is a FF interruption in Node. As the connection establishment takes a lot of time sometimes, the RTC is lost
    openWifi();
    //Its data
    send_MQTTdata(topicAlarms, mybufferWarningNode);
    close_socket();
    switchOFF_wifi();
    */
  }else if(messageLength2 == 17){
 /*This code is to send MQTT message in case there is a FF interruption in Node. As the connection establishment takes a lot of time sometimes, the RTC is lost
    openWifi();
    //Its data
    send_MQTTdata(topicAlarms, mybufferWarningNode);
    close_socket();
    switchOFF_wifi();
    */
    messageLength2 = 0;
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
    USB.printFloat(temperature, 1);
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
    USB.println(F("-----------------------------"));  
  }

  void read_sensors_data(){
    //1. Read values
    read_values();
    //2. Print values through serial port
    print_values(temp, humd, pres, x_acc, y_acc, z_acc, bat);
    //3. Convert to a valid format -from float to char-
    dtostrf(temp, 1, 1, temp_char);
    dtostrf(humd, 1, 1, humd_char);
    dtostrf(pres, 1, 1, pres_char);
 
    //Configure data
      char* formato1 = "field1=%s&field2=%s&field3=%s&field4=%d&field5=%d&field6=%d&field7=%d";
    sprintf(mybufferSensors, formato1, temp_char, humd_char, pres_char, bat, x_acc, y_acc, z_acc);

    if(bat<20){
      char* formato2 = "field6=LowBatEdge";
      sprintf(mybufferBat, formato2);
      //Send alarm to MQTT when battery is under 20%
      openWifi();
      //Its data
      send_MQTTdata(topicAlarms, mybufferBat);
      close_socket();
    }
  }

  void receiving_packets_node(int time_receiving_packets){

    //Initiate time to receive during x seconds
    error = xbee802.receivePacketTimeout(time_receiving_packets); //In ms

    //check answer  
    if(error == 0){
      //Save data received
      USB.print(F("Data: ")); 
      for(uint8_t i= 0; i<xbee802._length; i++){  //100
        messagePayload[i] = xbee802._payload[i];
        USB.print(messagePayload[i]);
      }
     messagePayload[xbee802._length] = '\0';
      //Separate data
      if(xbee802._length > 20){
        messageLength = xbee802._length;
        //It's NOT a warning
        strcpy(mybufferSensorsNode, messagePayload);
      }else{
        //It's the warning
        messageLength2 = xbee802._length;
        strcpy(mybufferWarningNode, messagePayload);
      }
    }else{
      // Print error message:
      /*
       * '7' : Buffer full. Not enough memory space
       * '6' : Error escaping character within payload bytes
       * '5' : Error escaping character in checksum byte
       * '4' : Checksum is not correct    
       * '3' : Checksum byte is not available 
       * '2' : Frame Type is not valid
       * '1' : Timeout when receiving answer   
      */
      USB.print(F("Error receiving a packet:"));
      USB.println(error,DEC);     
    }
  }

    void openWifi(){
    //////////////////////////////////////////////////
    // 1. Switch ON
    //////////////////////////////////////////////////
    //To set a period to the WiFi connection
    for(int i=0;i<20;i++){
      wifi_error = WIFI_PRO.ON(socket);
    }
  
    if ( wifi_error == 0 ){
      USB.println(F("1. WiFi switched ON"));
    }else{
      USB.println(F("1. WiFi did not initialize correctly"));
    }
  
    //////////////////////////////////////////////////
    // 2. Check if connected
    //////////////////////////////////////////////////
    // get actual time
    wifi_previous = millis();
    // check connectivity
    wifi_status =  WIFI_PRO.isConnected();
    // check if module is connected
    if ( wifi_status == true ){
      USB.print(F("2. WiFi is connected OK"));
      USB.print(F(" Time(ms):"));
      USB.println(millis() - wifi_previous);
      // get IP address
      wifi_error = WIFI_PRO.getIP();
      if (wifi_error == 0){
        USB.print(F("IP address: "));
        USB.println( WIFI_PRO._ip );
      }else{
        USB.println(F("getIP error"));
      }
    }else{
      USB.print(F("2. WiFi is connected ERROR"));
      USB.print(F(" Time(ms):"));
      USB.println(millis() - wifi_previous);
    }
  }
  
  void send_MQTTdata(char send_to_topic[], char send_payload[]){

    //////////////////////////////////////////////////
    // 3. TCP
    /////////////////////////////////////////////////
    // Check if module is connected
    if (wifi_status == true){
      ////////////////////////////////////////////////
      // 3.1. Open TCP socket
      ////////////////////////////////////////////////
      wifi_error = WIFI_PRO.setTCPclient( HOST, REMOTE_PORT, LOCAL_PORT);
  
      // check response
      if (wifi_error == 0){
        // get socket handle (from 0 to 9)
        socket_handle = WIFI_PRO._socket_handle;
  
        USB.print(F("3.1. Open TCP socket OK in handle: "));
        USB.println(socket_handle, DEC);
      }else{
        USB.println(F("3.1. Error calling 'setTCPclient' function"));
        WIFI_PRO.printErrorCode();
        wifi_status = false;
      }
    }
    if (wifi_status == true){
      /// Publish MQTT
      MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
      MQTTString topicString = MQTTString_initializer;
      unsigned char buf[200];
      int buflen = sizeof(buf);
      unsigned char payload[100];

  
      data.clientID.cstring = (char*)"mt1";
      data.keepAliveInterval = 30;
      data.cleansession = 1;
      int len = MQTTSerialize_connect(buf, buflen, &data); /* 1 */

     
      // Topic and message
      topicString.cstring = (char *)send_to_topic;
      snprintf((char*) payload, 100, "%s", send_payload);

      int payloadlen = strlen((const char*)payload);
      USB.print("Longitud del payload: ");
      USB.println(payloadlen);
    

     len += MQTTSerialize_publish(buf + len, buflen - len, 0, 0, 0, 0, topicString, payload, payloadlen); /* 2 */


      len += MQTTSerialize_disconnect(buf + len, buflen - len); /* 3 */

      
      ////////////////////////////////////////////////
      // 3.2. send data
      ////////////////////////////////////////////////
      wifi_error = WIFI_PRO.send(socket_handle, buf, len);
  
      // check response
      if (wifi_error == 0){
        USB.println(F("3.2. Send data OK"));
      }else{
        USB.println(F("3.2. Error calling 'send' function"));
        WIFI_PRO.printErrorCode();
      }
    }
  }


  void close_socket(){
        ////////////////////////////////////////////////
    // 3.4. close socket
    ////////////////////////////////////////////////
    wifi_error = WIFI_PRO.closeSocket(socket_handle);
  
    // check response
    if (wifi_error == 0){
      USB.println(F("3.4. Close socket OK"));
    }else{
      USB.println(F("3.4. Error calling 'closeSocket' function"));
      WIFI_PRO.printErrorCode();
    }
  }
  void switchOFF_wifi(){
        //////////////////////////////////////////////////
    // 4. Switches Wifi OFF
    //////////////////////////////////////////////////
    USB.println(F("WiFi switched OFF\n\n"));
    WIFI_PRO.OFF(socket);
 
  }



