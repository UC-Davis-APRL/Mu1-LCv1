#include <Arduino.h>

#include <QNEthernet.h>

#include <SPI.h>
#include <Lib124S08.h>

//==================Ethernet Config==================//

IPAddress subnet(255,255,255,0);  // Standard subnet mask

IPAddress ip(192,168,88,247);     // This MCU's IP
unsigned int localPort = 1683;    // Listen on this port

IPAddress remote(192,168,88,251); // IP of the remote server
unsigned int remotePort = 1683;   // Send to remote port

// An EthernetUDP instance to let us send and receive packets over UDP
using namespace qindesign::network;
EthernetUDP Udp;

//==================Packet Structure==================//

#define SENSOR_COUNT 4 // Number of sensor readings per packet

// Currently, all data values are 32 bit signed integers - but change this in the future
// Structure:
// Timestamp: 4 bytes
// Each ADC reading(sensor value) is 4 bytes
// PacketID: 4 bytes
const int dataPacketSize = SENSOR_COUNT*4 +4 +4; // Length of one data packet, in bytes

uint8_t outgoingDataPacketBuffers[dataPacketSize]; // Holds the current outgoing data packet

const int packetID = 7; // Identifies the packet type for COSMOS in each data packet sent

//==================ADC Config==================//

// Defines the switching order of the ADC's MUX in pairs, corresponding to the pins each load cell is connected to
// ie. First we set the MUX to the pins in [0] to read LC1, then set it to [2] to read LC2...
// Formatted to match INPMUX register, see TI ADS124S08 Figure 9.6.1.3 - Bits 7:4 is positive input pin, Bits 3:0 is negative input pin
const uint8_t muxSwitchOrder[4] = {ADS_P_AIN0 | ADS_N_AIN1, 
                                   ADS_P_AIN2 | ADS_N_AIN3, 
                                   ADS_P_AIN4 | ADS_N_AIN5, 
                                   ADS_P_AIN6 | ADS_N_AIN7};

// Function to reset the microcontroller by setting reset bit in reset control register
void doReboot() {
  SCB_AIRCR = 0x05FA0004;
}

// Define to enable debug messages over serial(compile time)
#define DEBUG_MODE_SERIAL

void setup() {
  #if defined(DEBUG_MODE_SERIAL)
  
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }
  Serial.println("Connected...");
  delay(2000);
  #endif // DEBUG_MODE_SERIAL
  
  memset(outgoingDataPacketBuffers, 0, dataPacketSize); // danger

  // Check for Ethernet hardware present
  if (!Ethernet.begin()) {
    #if defined(DEBUG_MODE_SERIAL)
    
    printf("Failed to start Ethernet\r\n");
    
    #endif // DEBUG_MODE_SERIAL
    
    doReboot();
  }

  // Listen for link changes
  Ethernet.onLinkState([](bool state) {
    #if defined(DEBUG_MODE_SERIAL)
    
    printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF");
    
    #endif // DEBUG_MODE_SERIAL
  });

  // Start UDP with listener on specified port
  Udp.beginWithReuse(localPort);


  // Run ADC pin config
  InitGPIO();
  // Begin ADC SPI
  SPI.begin();

  // Init ADC
  adcStartupRoutine();
  // Ensure ADC is in a halted state befor continuing config
  stopConversions();

  // Set pga enabled, gain 128 TODO: Check if conversion needs to be active to set pga register
  writeSingleRegister(REG_ADDR_PGA, (ADS_PGA_ENABLED|ADS_GAIN_128));
  // Set drate
  writeSingleRegister(REG_ADDR_DATARATE, ADS_DR_2000);

  // Disable reference buffers
  writeSingleRegister(REG_ADDR_REF, (ADS_REFP_BYP_DISABLE | ADS_REFN_BYP_DISABLE));

  #if defined(DEBUG_MODE_SERIAL)
  
  Serial.print("pga pre-config readback: ");
  Serial.println(readSingleRegister(REG_ADDR_PGA));
  Serial.print("datarate pre-config readback: ");
  Serial.println(readSingleRegister(REG_ADDR_DATARATE));
  
  #endif // DEBUG_MODE_SERIAL
  

  delay(10); //allow adc time to settle/config
  startConversions(); // Must be running when doing cal
  delay(10); //allow adc time to settle/config
  sendCommand(OPCODE_SFOCAL); // Do self offset cal
  delay(10); //allow adc time to settle/config
  writeSingleRegister(REG_ADDR_INPMUX, muxSwitchOrder[0]); // Set intial mux
  delay(10); //allow adc time to settle/config
  // stopConversions(); // needed?
  enableDRDYinterrupt(true); // Attach the drdy interrupt

  #if defined(DEBUG_MODE_SERIAL)
  
  Serial.print("pga POST-config readback: ");
  Serial.println(readSingleRegister(REG_ADDR_PGA));
  Serial.print("datarate POST-config readback: ");
  Serial.println(readSingleRegister(REG_ADDR_DATARATE));
  
  #endif // DEBUG_MODE_SERIAL

  // TODO: Add a check to make sure that ADC settings stuck, if not, handle it or reboot

}

int lastLoop = 0; // variable to store the time of the last loop for debugging - TODO: refactor

void loop() {

  // if(waitForDRDYHtoL(10000)){
  //   Serial.print("Time: ");
  //   Serial.println(millis());
  //   Serial.print("Reading: ");
  //   Serial.println(readConvertedData(NULL, COMMAND));
  // }


  // Shift in the current mcu time in ms as the first 4 bytes of the packet buffer
  for (int i = 0; i<4; i++)
  {
    outgoingDataPacketBuffers[(dataPacketSize-4)+i] = (millis() >> (8*(3-i))) & 0xFF;
  }

  // Read SENSOR_COUNT number of readings, then add them to the packet buffer
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    long tempData; // temporary adc data variable
    uint8_t startByte = (i*4) + 4; // starting byte offset for this reading, in bytes(intial offset of 4B for the timestamp)
    
    // Get the ADC reading for this sensor
    if(waitForDRDYHtoL(100)){ // wait for ADC for indicate data is ready to be read, with a timeout of Xms
      tempData = readConvertedWhileMux(muxSwitchOrder[i]); // read the data, while writing the next sensor's pin config in the MUX
    }
    else{ // if timeout when waiting for ADC, reboot TODO: Refactor init code, so we can just attempt ADC reinit instead of rebooting MCU too
      doReboot();
    }
    
    // tempData = random(100); // test data junk for testing without ADC

    // Fill in this reading(len:4B), using the correct offset we calced earlier
    for (int j = 0; j<4; j++)
    {
      outgoingDataPacketBuffers[startByte+j] = (tempData >> (8*(3-j))) & 0xFF;
    }
  }

  // Once the packet is filled with sensor readings, attach an ID to the end, so the reciever knows the packet type
  for (int j = 0; j<4; j++)
  {
    outgoingDataPacketBuffers[j] = (packetID >> (8*(3-j))) & 0xFF;
  }

  // Send sensor data packet
  Udp.send(remote,remotePort,outgoingDataPacketBuffers,dataPacketSize);

  // Serial.println(millis()-lastLoop); // debug stuff TODO: refactor with above looptime thing
  // lastLoop = millis();
}