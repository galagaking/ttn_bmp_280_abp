/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example will send Temperature and Air Pressure
 * using frequency and encryption settings matching those of
 * the The Things Network. Application will 'sleep' 7x8 seconds (56 seconds)
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************
 *code adapted by F. Beks for TTN Workshops in the Eindhoven IoT community     *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "i2c.h"

#include "i2c_BMP280.h"
BMP280 bmp280;

#include <Arduino.h>

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 55;

bool BMP_Available=false;

#define LedPin 2     // pin 13 LED is not used, because it is connected to the SPI port

// LoRaWAN NwkSKey, network session key, AppSKey, application session key, end-device address
static const PROGMEM u1_t NWKSKEY[16] = { 0x21, 0xEE, //fill in your NWSKey };
static const PROGMEM u1_t APPSKEY[16] = { 0xFE, 0xC6, //fill in your Appskey };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x; //fill in your devaddr  // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
static osjob_t initjob;

// Pin mapping is hardware specific.
// Pin mapping Doug Larue PCB
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {4, 5, 7},
};

void onEvent (ev_t ev) {
  int i,j;
    Serial.print(os_getTime());
    Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      //os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.
        if (i>10){
          i=10;     // maximum number of BLINKs
        }
          for(j=0;j<i;j++)
          {
            digitalWrite(LedPin,HIGH);
            delay(200);
            digitalWrite(LedPin,LOW);
            delay(400);
          }
      }
      // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  byte buffer[2];
  float temperature,pascal;
  uint16_t t_value, p_value, s_value;
  if (BMP_Available)
  {
    bmp280.awaitMeasurement();
    bmp280.getTemperature(temperature);
    bmp280.getPressure(pascal);
    bmp280.triggerMeasurement();
    pascal=pascal/100;
    Serial.print(F(" Pressure: "));
    Serial.print(pascal);
    Serial.print(F(" Pa; T: "));
    Serial.print(temperature);
    Serial.println(F(" C"));

    // getting sensor values
  
    temperature = constrain(temperature,-24,39);  //temp in range -24 to 40 (64 steps)
    pascal=constrain(pascal,970,1033);    //pressure in range 970 to 1034 (64 steps)*/
        t_value=int16_t((temperature*(100/6.25)+2400/6.25)); //0.0625 degree steps with offset
                                                      // no negative values
        Serial.print(F("decoded TEMP: "));
        Serial.print(t_value,HEX);
        p_value=int16_t((pascal-970)/1); //1 mbar steps, offset 970.
        Serial.print(F(" decoded Pascal: "));
        Serial.print(p_value,HEX);
        s_value=(p_value<<10) + t_value;  // putting the bits in the right place
        Serial.print(F(" decoded sent: "));
        Serial.println(s_value,HEX);
        buffer[0]=s_value&0xFF; //lower byte
        buffer[1]=s_value>>8;   //higher byte
  }
  else
  //no BMP connected, sent 12 34 dummy data
  {
        buffer[0]=0x12;
        buffer[1]=0x34;
  }
    // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*) buffer, 2 , 0);
    Serial.println(F("Sending: "));
  }
}
void setup()
  {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("Workshop series, 1 of 4: Starting TTN Node"));
  Serial.println( "Compiled: " __DATE__ ", " __TIME__);
  Serial.println(F("LMIC settings:"));
#ifdef DISABLE_PING
  Serial.println(F("- ping disabled"));
#else
  Serial.println(F("- ping enabled"));
#endif
#ifdef DISABLE_BEACONS
  Serial.println(F("- beacon disabled"));
#else
  Serial.println(F("- beacon enabled"));
#endif
#ifdef DISABLE_JOIN
  Serial.println(F("- join disabled"));
#else
  Serial.println(F("- join enabled"));
#endif
#ifdef LMIC_FAILURE_TO
  Serial.println(F("- LMIC_FAILURE TO enabled"));
#else
  Serial.println(F("- LMIC_FAILURE_TO disabled"));
#endif
#ifdef LMIC_PRINTF_TO
  Serial.println(F("- LMIC_PRINTF TO enabled"));
#else
  Serial.println(F("- LMIC_PRINTF_TO disabled"));
#endif
  Serial.print(F("Probe BMP280: "));
  BMP_Available = bmp280.initialize();
  if (BMP_Available)
  {
    Serial.println(F("found"));
    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
  }
  else
  {
    Serial.println(F("missing"));
  }


    //LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    Serial.print(F("Device Address: "));
    Serial.println(DEVADDR,HEX);
    if (DEVADDR==0)
    {
      Serial.println(F("Invalid Node Address, check your TTN device ID"));
      while(1);
    }
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);
   do_send(&sendjob);    // Sent sensor values
}

void loop() {
  os_runloop_once();
}

