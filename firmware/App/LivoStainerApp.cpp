#include "Stm32SerialCompat.h"
/**
Project: Livo Stainer
File: StainerLivo.ino
Description: Firmware for controlling device

Author: Livo, Varalakshmi
Date Created: 2026-01-19
Version: 0.1
*/

#include "LivoCommunication.h"
#include "PacketProcessor.h"
#include "Constants.h"
#include "LidarReader.h"
#include "Execution2019Handler.h"
#include "MagazineLED.h"
#include "NozzleDHT.h"
#include "LoopStats.h"

// Native port: prototypes the Arduino preprocessor generated for the .ino.
void setupI2C();
void loopI2C();
void loopTimeFunction();

// Definitions for global variables declared in Constants.h
bool displayDebug = false;
bool ackAction = true;
bool printEncoderChange = false;
bool printloopTime = false;
long loopTime = micros();
long prevtotalTime = 0;
volatile bool autoUpdateEncoder = false; 
bool printIR = false;
bool printLimit = false;
bool startMagzinechecks = true;

#ifdef Master
  #ifdef Stainer_Gantry_PCB
    TwoWire Wire1(Acc1_SDA, Acc1_SCL);
    TwoWire Wire2(Acc2_SDA, Acc2_SCL);
    TwoWire Wire3(I2C3_SDA, I2C3_SCL);
    #if MAGAZINE_UART_TEST
      LivoHardwareSerial Magazine1Serial(MAG1_UART_RX, MAG1_UART_TX);
      LivoHardwareSerial Magazine2Serial(MAG2_UART_RX, MAG2_UART_TX);
      I2CInstance i2c1(Magazine1Serial);
      I2CInstance i2c2(Magazine2Serial);
    #else
      I2CInstance i2c1(Wire3);
      I2CInstance i2c2(Wire3);
    #endif
    AccelerometerInstance acc1(Wire1);
    AccelerometerInstance acc2(Wire2);
    LidarReader lidarReader(Wire3);
  #endif
  #ifdef Stainer_Master_PCB
    TwoWire Wire1(I2C1_SDA, I2C1_SCL);
    TwoWire Wire3(I2C3_SDA, I2C3_SCL);
    I2CInstance i2c1(Wire1);
    I2CInstance i2c2(Wire1);
    I2CInstance i2c3(Wire3);
    I2CInstance i2c4(Wire3);
    AccelerometerInstance acc1(Wire3);
  #endif
#endif

#ifdef Slave
  #if MAGAZINE_UART_TEST && defined(Magazine_PCB_V2)
    LivoHardwareSerial MagazineSerial(MAG_UART_RX, MAG_UART_TX);
    I2CInstanceSlave i2c(MagazineSerial);
  #else
    I2CInstanceSlave i2c(Wire);
  #endif
#endif

LivoCommunication livoCommunication;
PacketProcessor packetProcessor(livoCommunication);

void setup()
{
	// analogReadResolution(12);
  livoCommunication.init();
  // Sensor initialization can emit SFAULT through mirroredSerial. Its UART
  // must be running first; writing an unopened STM32 UART can block startup.
  execution2019Handler.beginControllerLink();
  setupI2C();           // I2C + MPU6050 must come up before any GHOME/LOAD that uses magazine I2C
  #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
    return;
  #endif
  #ifdef Magazine_PCB_V2
    magazineLED.setup();
  #endif
  packetProcessor.setup();
  #ifdef Nozzle_Mount_PCB
    setupNozzleDHT();
  #endif
}

void setupI2C() {
  #ifdef Master
    #ifdef Stainer_Gantry_PCB
      #if MAGAZINE_UART_TEST
        i2c1.setupUART();
        i2c2.setupUART();
        // Magazine UART setup does not initialize the separate I2C3 bus.
        // LiDAR, Hall boards, and I2CSCAN still use PC9/PA8.
        Wire3.begin();
      #else
        i2c1.setup(I2C3_SDA, I2C3_SCL);
        i2c2.setup(I2C3_SDA, I2C3_SCL);
      #endif
      acc1.setup(Acc1_SDA, Acc1_SCL, 0x68);
      #if !MAGAZINE_UART_TEST
        acc2.setup(Acc2_SDA, Acc2_SCL, 0x69);
      #endif
      lidarReader.setup();
    #endif
    #ifdef Stainer_Master_PCB
      i2c1.setup(I2C1_SDA, I2C1_SCL);
      i2c2.setup(I2C1_SDA, I2C1_SCL);
      i2c3.setup(I2C3_SDA, I2C3_SCL);
      i2c4.setup(I2C3_SDA, I2C3_SCL);
      acc1.setup(I2C3_SDA, I2C3_SCL, 0x68);
    #endif
  #endif

  #ifdef Slave
    #if MAGAZINE_UART_TEST && defined(Magazine_PCB_V2)
      i2c.setupUART(DEVICE_ID);
    #else
      i2c.setup(I2C1_SDA, I2C1_SCL, DEVICE_ID);
    #endif
  #endif
}

void loop()
{
  LoopStats::onLoop();
  #if defined(MAGAZINE_BOOTLOADER_BUILD) && defined(Magazine_PCB_V2)
    loopI2C();
    return;
  #endif
  #if defined(Master) && defined(Stainer_Master_PCB)
    // Master: prefix-based router (G; N; M;) — replaces the default Serial reader
    execution2019Handler.serviceMasterCommandRouter();
  #else
    livoCommunication.serialEvent_();
  #endif

  packetProcessor.loop();
  #ifdef Nozzle_Mount_PCB
    serviceNozzleDHT();
  #endif
  loopTimeFunction();
  loopI2C();

  #ifdef Magazine_PCB_V2
    static int magazineLedIrValues[24] = {};
    static unsigned long lastMagazineLedSampleMs = 0;
    bool magazineLedValuesReady = false;
    const unsigned long now = millis();

    // Keep insertion feedback local and near-instant. LED frames are still sent
    // only when pixels change and while the I2C slave is idle.
    if (now - lastMagazineLedSampleMs >= 20) {
      lastMagazineLedSampleMs = now;
      magazineLedValuesReady = i2c.captureMagazineLedSnapshot(magazineLedIrValues);
    }

    magazineLED.loop(magazineLedIrValues, magazineLedValuesReady);
    if (magazineLED.hasPendingShow() && i2c.isMagazineLedBusIdle()) {
      magazineLED.show();
    }
  #endif
}

void loopI2C() {
  #ifdef Master
    #ifdef Stainer_Gantry_PCB
      i2c1.loop();
      i2c2.loop();
      acc1.loop();
      #if !MAGAZINE_UART_TEST
        acc2.loop();
      #endif
    #endif
    #ifdef Stainer_Master_PCB
      i2c1.loop();
      i2c2.loop();
      i2c3.loop();
      i2c4.loop();
      acc1.loop();
    #endif
  #endif
  
  #ifdef Slave
    i2c.loop();
  #endif
}

void loopTimeFunction() {
  if(printloopTime){
      long totalTime = micros() - loopTime;
      if(abs(prevtotalTime-totalTime) > totalTime * 0.8)
      {
        Serial.print("T:");
        Serial.print(totalTime);
        Serial.println("");
        prevtotalTime=totalTime;
      }
      loopTime = micros();
  }
}

// Native port: entry points called by Platform main().
void appSetup() { setup(); }
void appLoop() { loop(); }
