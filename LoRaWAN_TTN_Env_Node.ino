/*  
 * ------------------------------------------------------------------------
 * "PH2LB LICENSE" (Revision 1) : (based on "THE BEER-WARE LICENSE" Rev 42) 
 * <lex@ph2lb.nl> wrote this file. As long as you retain this notice
 * you can do modify it as you please. It's Free for non commercial usage 
 * and education and if we meet some day, and you think this stuff is 
 * worth it, you can buy me a beer in return
 * Lex Bolkesteijn 
 * ------------------------------------------------------------------------ 
 * Filename : LoRaWAN_TTN_Env_Node.ino  
 * Version  : 1.0 (BETA)
 * ------------------------------------------------------------------------
 * Description : A low power BME280 based datalogger for the ThingsNetwork.
 *  with deepsleep support and variable interval
 * ------------------------------------------------------------------------
 * Revision : 
 *  - 2017-jul-17 1.0 first "beta"
 * ------------------------------------------------------------------------
 * Hardware used : 
 *  - Arduino Pro-Mini 3.3V 
 *  - RFM95W
 *  - BME280 sensor.
 * ------------------------------------------------------------------------
 * Software used : 
 *  - LMIC https://github.com/matthijskooijman/arduino-lmic 
 *  - LowPower library https://github.com/rocketscream/Low-Power
 *  - MiniCore loader  https://forum.arduino.cc/index.php?topic=412070 https://github.com/MCUdude/MiniCore 
 *    To use a brownout detection of 1.8V.
 *  - special adcvcc library from Charles (see : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059/32?u=lex_ph2lb )
 *  - BME280 library https://github.com/finitespace/BME280
 *  
 * For licenses of the used libraries, check the links above.
 * ------------------------------------------------------------------------ 
 * Aditional note : 
 * 
 * I use a HTTP integration on the TTN with the decoder below.
 * ------------------------------------------------------------------------ 
 * TODO LIST : 
 *  - add more sourcode comment
 *  
 *  
 **************************************************************************************** 
 * TheThingsNetwork Payload functions : 
    function Decoder(bytes, port) 
    {
      var retValue =   { 
        bytes: bytes
      };
      
      retValue.batt = bytes[0] / 10.0;
      if (retValue.batt === 0)
         delete retValue.batt; 
     
      if (bytes.length >= 2)
      {
        retValue.humidity = bytes[1];
        if (retValue.humidity === 0)
          delete retValue.humidity; 
      } 
      if (bytes.length >= 3)
      {
        retValue.temperature = (((bytes[2] << 8) | bytes[3]) / 10.0) - 40.0;
      } 
      if (bytes.length >= 5)
      { 
        retValue.pressure = ((bytes[4] << 8) | bytes[5]); 
        if (retValue.pressure === 0)
          delete retValue.pressure; 
      }
       
      return retValue; 
    }
 */

 
/*
*****************************************************************************************
* INCLUDE FILES
*****************************************************************************************
*/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LowPower.h>
#include <Wire.h> 
#include  "adcvcc.h"  
#include  "BME280I2C.h"

/*
 * Defines
 */ 

#define debugSerial Serial 
#define SHOW_DEBUGINFO  
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); } 

#define FASTINTERVAL    60    // 60 seconds (for testing)
#define NORMALINTERVAL  900   // 15 minutes (normal)


// Restrict to channel0   if uncommented; otherwise all channels  (allways use SF7)
// #define CHANNEL0
 
//// Choice on of the LMIC pinnings below.

//// Pin mapping for TTN Enschede board
//#define LMIC_NSS    10
//#define LMIC_RXTX   6
//#define LMIC_RST    LMIC_UNUSED_PIN
//#define LMIC_DIO0   4
//#define LMIC_DIO1   5
//#define LMIC_DIO2   7

// Pin mapping CH2I (check out : https://www.thethingsnetwork.org/forum/t/full-arduino-mini-lorawan-and-1-3ua-sleep-mode/8059 ) 
#define LMIC_NSS    10
#define LMIC_RXTX   LMIC_UNUSED_PIN
#define LMIC_RST    LMIC_UNUSED_PIN
#define LMIC_DIO0   2
#define LMIC_DIO1   7
#define LMIC_DIO2   8


const lmic_pinmap lmic_pins = {
    .nss = LMIC_NSS,
    .rxtx = LMIC_RXTX,   
    .rst = LMIC_RST,
    .dio = {LMIC_DIO0, LMIC_DIO1, LMIC_DIO2},  
}; 

// UPDATE WITH YOURE TTN KEYS AND ADDR
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // LoRaWAN NwkSKey, network session key 
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00A6, 0x00 }; // LoRaWAN AppSKey, application session key 
static const u4_t DEVADDR = 0x00000000 ; // LoRaWAN end-device address (DevAddr)

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
 
static osjob_t sendjob; 

// global enviromental parameters
static float temp = 0.0;
static float pressure = 0.0;
static float humidity = 0.0;    

int interval = NORMALINTERVAL;  

byte LMIC_transmitted = 0;
byte LMIC_event_Timeout = 0;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
// const unsigned TX_INTERVAL = 60; 

BME280I2C bme;                   // Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

/* ======================================================================
Function: ADC_vect
Purpose : IRQ Handler for ADC 
Input   : - 
Output  : - 
Comments: used for measuring 8 samples low power mode, ADC is then in 
          free running mode for 8 samples
====================================================================== */
ISR(ADC_vect)  
{
  // Increment ADC counter
  _adc_irq_cnt++;
}


void updateEnvParameters()
{
    delay(1000); 
    temp = bme.temp(true);
    pressure = bme.pres(1);    // 1 = hPa (milliBar)  
    humidity =  bme.hum(); 
} 

void onEvent (ev_t ev) 
{
    debugPrint(os_getTime());
    debugPrint(": ");
    debugPrintLn(ev);
    switch(ev) 
    {
        case EV_SCAN_TIMEOUT:
            //debugPrintLn(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            //debugPrintLn(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            //debugPrintLn(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            //debugPrintLn(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            //debugPrintLn(F("EV_JOINING"));
            break;
        case EV_JOINED:
            //debugPrintLn(F("EV_JOINED"));
            break;
        case EV_RFU1:
            //debugPrintLn(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            //debugPrintLn(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            //debugPrintLn(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            debugPrintLn(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
              debugPrintLn(F("R ACK")); // Received ack
            if (LMIC.dataLen) 
            {
              debugPrintLn(F("R "));
              debugPrintLn(LMIC.dataLen);
              debugPrintLn(F(" bytes")); // of payload
            }            
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            LMIC_transmitted = 1; 
            break;
        case EV_LOST_TSYNC:
            //debugPrintLn(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            //debugPrintLn(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            //debugPrintLn(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            //debugPrintLn(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            //debugPrintLn(F("EV_LINK_ALIVE"));
            break;
         default:
            //debugPrintLn(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) 
    {
        debugPrintLn(F("OP_TXRXPEND")); //P_TXRXPEND, not sending
    } 
    else 
    {
        // Prepare upstream data transmission at the next possible time.
 
        // Here the sensor information should be retrieved 
        //  Pressure: 300...1100 hPa
        //  Temperature: -40…85°C  
        updateEnvParameters();
        updateEnvParameters();
 
        int batt = (int)(readVcc() / 100);  // readVCC returns  mVolt need just 100mVolt steps
        byte batvalue = (byte)batt; // no problem putting it into a int. 
       
#ifdef SHOW_DEBUGINFO
        debugPrint(F("T="));
        debugPrintLn(temp); 
        
        debugPrint(F("P="));
        debugPrintLn(pressure); 
        
        debugPrint(F("H="));
         debugPrintLn(humidity); 
      
        debugPrint(F("B="));
        debugPrintLn(batt);
        debugPrint(F("BV="));
        debugPrintLn(batvalue);  
#endif
        int t = (int)((temp + 40.0) * 10.0); 
        // t = t + 40; => t [-40..+85] => [0..125] => t = t * 10; => t [0..125] => [0..1250]
        int p = (int)(pressure);  // p [300..1100]
        int h = (int)(humidity);

        unsigned char mydata[6];
        mydata[0] = batvalue;      
        mydata[1] = h & 0xFF; 
        mydata[2] = t >> 8;
        mydata[3] = t & 0xFF;
        mydata[4] = p >> 8;
        mydata[5] = p & 0xFF; 
        
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        debugPrintLn(F("PQ")); //Packet queued
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setup() {
    Serial.begin(115200);
    debugPrintLn(F("Starting"));
    
    if (!bme.begin()) 
    {  
        debugPrintLn(F("No valid bme280 sensor!"));
        delay(1000);  // allow serial to send.
        while (1) 
        {
           LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);   
        }
    } 
  
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

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


    // For single channel gateways: Restrict to channel 0 when defined above
#ifdef CHANNEL0
    LMIC_disableChannel(1);
    LMIC_disableChannel(2);
    LMIC_disableChannel(3);
    LMIC_disableChannel(4);
    LMIC_disableChannel(5);
    LMIC_disableChannel(6);
    LMIC_disableChannel(7);
    LMIC_disableChannel(8);
#endif

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
    debugPrintLn(F("S")); // Setup complete!"
    delay(1000);  // allow serial to send.
}

void loop() 
{ 
    // Start job
    do_send(&sendjob);
    // Wait for response of the queued message (check if message is send correctly)
    os_runloop_once();
    // Continue until message is transmitted correctly
    //debugPrintLn("\tWaiting for transmittion\n"); 
    debugPrintLn(F("W\n"));
    while(LMIC_transmitted != 1) 
    {
        os_runloop_once();
        // Add timeout counter when nothing happens:
        LMIC_event_Timeout++;
        delay(1000);
        if (LMIC_event_Timeout >= 60) 
        {
            // Timeout when there's no "EV_TXCOMPLETE" event after 60 seconds
            debugPrintLn(F("\tETimeout, msg not tx\n"));
            break;
        } 
    }

    LMIC_transmitted = 0;
    LMIC_event_Timeout = 0;
    //debugPrintLn(F("Going to sleep."));
    delay(1000);  // allow serial to send.
    
    for (int i = 0; i < interval; i++)
    {  
          i +=7 ; // no normal 1 second run but 8 second loops m.      
          // Enter power down state for 8 s with ADC and BOD module disabled
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);   
    }  
    //debugPrintLn("---");
}
