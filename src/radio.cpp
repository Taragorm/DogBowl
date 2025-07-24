#include "radio.h"
#include <Arduino.h>
#include <RFM69.h>
#include "defs.h"
#include <psiiot.h>>

//RFM69 radio_(Pins::RADIO_CS, Pins::RADIO_IRQ, /*isRFM69HW*/ IS_RFM69HW_HCW );

#if HAVE_RADIO
    #ifdef ENABLE_ATC
        #include <RFM69_ATC.h>
        RFM69_ATC radio_(Pins::RADIO_CS, Pins::RADIO_IRQ, /*isRFM69HW*/ IS_RFM69HW_HCW );
    #else
        #include <RFM69.h>
        RFM69 radio_(Pins::RADIO_CS, Pins::RADIO_IRQ, /*isRFM69HW*/ IS_RFM69HW_HCW  );
    #endif
#endif

static bool radioOk_;
typedef psiiot::Buffer<40> Buffer_t;
static Buffer_t radioBuffer_;
//---------------------------------------------------
void initRadio()
{
    bool ok = radio_.initialize(FREQUENCY, NODEID, NETWORKID);
    if(!ok)
        Serial.println("Radio fail");

#if SERIAL_TRACE
    else
        Serial.printf("\nRadio Init ok=%d freq=%d, node=%d, netid=%d\n ", ok, FREQUENCY, NODEID, NETWORKID);
#endif

#ifdef IS_RFM69HW_HCW
    radio_.setHighPower(); //must include this only for RFM69HW/HCW!
#endif

#ifdef ENCRYPTKEY
 #if SERIAL_TRACE
    Serial.printf("Encrypt key=[%s]\n", ENCRYPTKEY);
 #endif
    radio_.encrypt(ENCRYPTKEY);
#endif

    //Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
    //For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
    //For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
    //Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
    radio_.enableAutoPower(ATC_RSSI);
#endif

    radio_.setMode(RF69_MODE_RX);
    //_buffer.encodeID
}
//---------------------------------------------------
void sendRadio(float wt, float volts, int8_t state)
{
    radioBuffer_.reset();
    radioBuffer_.putID(IOT_ID_CODE);

    //
    // load stuff
    radioBuffer_.put('W', wt);       // weight in grams
    radioBuffer_.put('V', volts);    // battery volts
    radioBuffer_.put('A', state);    // alarm state

    //
    // finalised & send
    radioBuffer_.finalise();
    radio_.send(
                GW_NODEID, 
                radioBuffer_.buffer(),
                radioBuffer_.writeSpaceUsed() 
                );
}
//---------------------------------------------------
