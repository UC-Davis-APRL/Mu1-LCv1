#include <Arduino.h>

#include <QNEthernet.h>

#include <SPI.h>
#include <Lib124S08.h>

// Define to enable debug messages over serial and freeze on crashes
// #define DEBUG_MODE_SERIAL

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

#define SENSOR_COUNT 4 // Number of sensor readings per set

#define BATCH_SIZE 10 // Number of reading sets per packet

// Currently, all data values are 32 bit signed integers - but change this in the future
// Structure:
// PacketID: 4 bytes
// Each ADC reading(sensor value) is 4 bytes, batched as sets of 50 samples, see beblow
// Timestamp: 4 bytes

const long packetID = 7; // Identifies the packet type for COSMOS in each data packet sent

struct __attribute__ ((packed)) packetBuffer  {
  long packetTag;
  long load1[BATCH_SIZE];
  long load2[BATCH_SIZE];
  long load3[BATCH_SIZE];
  long load4[BATCH_SIZE];
  long packetTime;
};

packetBuffer outgoingBuffer;

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
  writeSingleRegister(REG_ADDR_DATARATE, ADS_DR_4000);

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

int lastLoop = 0; // Millis since last iteration

void loop() {
  
  // new batching throw together:
  outgoingBuffer.packetTime = millis();

  for (int sample = 0; sample < BATCH_SIZE; sample++) {
    for (int sensor = 0; sensor < SENSOR_COUNT; sensor++)
    {
      long tempData;

      #ifdef DEBUG_MODE_SERIAL
      long readTime = millis();
      #endif // DEBUG_MODE_SERIAL

      if(waitForDRDYHtoL(100)) {
        tempData = readConvertedWhileMux(muxSwitchOrder[(sensor+1) % SENSOR_COUNT]); // read the data, while writing the next sensor's pin config in the MUX

        #ifdef DEBUG_MODE_SERIAL
        Serial.print("Time:");
        Serial.print(millis());
        Serial.print("/Sensor:");
        Serial.print(sensor);
        Serial.print("/Data:");
        Serial.print(tempData);
        Serial.print("/Readtime:");
        Serial.println(millis() - readTime);
        #endif // DEBUG_MODE_SERIAL

        switch (sensor)
        {
        case 0:
          outgoingBuffer.load1[sample] = tempData;
          break;
        case 1:
          outgoingBuffer.load2[sample] = tempData;
          break;
        case 2:
          outgoingBuffer.load3[sample] = tempData;
          break;
        case 3:
          outgoingBuffer.load4[sample] = tempData;
          break;
        default:
          #ifdef DEBUG_MODE_SERIAL
          Serial.print("sensor switch error execution stopped: sample: ");
          Serial.print(sample);
          Serial.print(" sensor: ");
          Serial.println(sensor);
          while (true) // stop execution
          {
          }
          #endif // DEBUG_MODE_SERIAL
          doReboot(); // should never reach here, a unrecoverable error must have occurred
          break;
        }
      }
      else { // if timeout when waiting for ADC, reboot TODO: Refactor init code, so we can just attempt ADC reinit instead of rebooting MCU too
        #ifdef DEBUG_MODE_SERIAL
        Serial.println("ADC not responsive execution stopped: sample: ");
        Serial.print(sample);
        Serial.print(" sensor: ");
        Serial.println(sensor);
        while (true) // stop execution
        {
        }
        #endif // DEBUG_MODE_SERIAL
        doReboot();
      }
    }
  }

  outgoingBuffer.packetTag = packetID;

  // Send the complete buffer via UDP
  Udp.send(remote, remotePort, (uint8_t*)&outgoingBuffer, sizeof(outgoingBuffer));
}