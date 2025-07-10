// Visual Micro is in vMicro>General>Tutorial Mode
//
/*
    Name:       Dogbowl.ino
    Created:	25/05/2025 14:40:00
    Author:     FREYA\Simon
*/
#include <stdint.h>
#include "defs.h"
#include <MilliTimer.h>
#include <hx711.h>
#include <eeprom.h>
#include <psiiot.h>
#include <clocks.h>
#include <regutils.h>
#include <PitSleep.h>
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
//#include <hc12crc.h>


const uint16_t MAJICK1 = 0xDEAD;
const uint16_t MAJICK2 = 0xBEEF;
const uint16_t SLEEPFLASHSEC = 30;
const uint16_t FLASHPERSAMPLE = 10;

const int EEPROM_ADDR = 0;

struct NVM
{
    uint16_t majick1;
    long zero;
    double scale;
    uint16_t majick2;
} nvm_;

//psiiot::MilliTimer timer(1000,true);
HX711 loadcell_(Pins::MX711_DOUT, Pins::MX711_SCK);

const int BUFFSIZE = 32;
char buffer_[BUFFSIZE+1];
char* buffptr_ = buffer_;

PitSleep<1> _sleeper;
bool asleep_;

const int FLASHMS = 10;
/// Timer for LED flash
MilliTimer _flashTimer(FLASHMS, false);
MilliTimer _secondTimer(1000, true);

const uint8_t TARECOUNT = 4;
uint8_t _tareCounter;
uint8_t _tareMode;
uint8_t _poll;
uint8_t _lastLeds;
const uint16_t ASLEEPTICKS = 20;    ///< how many ticks before we check scales/batt
const uint16_t AWAKETICKS  = 60;    ///< sec before we sleep after no activity
uint16_t awaketicks_;
uint16_t flashCounter_;
bool battLow_;
bool scaleLow_;
bool noSleep_;

#if HAVE_RADIO
    #ifdef ENABLE_ATC
        RFM69_ATC radio_(Pins::RadioCS, Pins::RadioIrq, true);
    #else
        RFM69 radio_(Pins::RadioCS, Pins::RadioIrq, true );
    #endif
#endif


static double  pollScale(bool print=true);
static double getBattVolts(bool print=true);
static void pollSerial();
static void doCommand();
static void asleep();
static void awake();
static void doSleep();
static void doWake();
static void zero();
static void status();
static void kick();

const uint8_t RED =1;
const uint8_t GREEN =2;
const uint8_t YELLOW =3;
const uint8_t BLUE =4;
const uint8_t MAGENTA =5;
const uint8_t CYAN =6;
const uint8_t WHITE =3;

//------------------------------------------------------------------------------------------
static void leds(uint8_t c)
{
    _lastLeds = c;
    c = ~c; // flip the bits
    digitalWriteFast(Pins::R, (c&1) != 0); // writeFast assumes 0/1 only
    digitalWriteFast(Pins::G, (c&2) != 0);
    digitalWriteFast(Pins::B, (c&4) != 0);

}
//------------------------------------------------------------------------------------------
static uint8_t leds()
{
    return _lastLeds;
}
//------------------------------------------------------------------------------------------
void showPIT()
{
    Serial.printf(
        "\r\nPITCTRLA= %04x  CLKSEL=%04x\r\n",
        RTC_PITCTRLA,
        RTC_CLKSEL
        );
}
//------------------------------------------------------------------------------------------
void resetFactors()
{
    nvm_.majick1    = MAJICK1;
    nvm_.zero     = 0;
    nvm_.scale      = 1.0;
    nvm_.majick2    = MAJICK2;
    EEPROM.put(EEPROM_ADDR, nvm_);    
}    
//------------------------------------------------------------------------------------------
void apply()
{
    loadcell_.set_scale(nvm_.scale);
    loadcell_.set_zero(nvm_.zero);    
}    
//------------------------------------------------------------------------------------------
// The setup() function runs once each time the micro-controller starts
void setup()
{

    analogReference(INTERNAL4V34);
    pinMode(Pins::R, OUTPUT);
    pinMode(Pins::G, OUTPUT);
    pinMode(Pins::B, OUTPUT);
    pinMode(Pins::TARE, INPUT_PULLUP);
#if HAVE_RATE
    pinMode(Pins::MX711_RATE, OUTPUT);
#endif
    //Serial.swap();
    Serial.begin(115200);
    delay(250);
    Serial.print("\f*START*\r\n");
    
    for(int i=0; i<9; ++i)
    {
        leds(i);
        delay(500);
    }

    /*
    loadcell_.begin();
    EEPROM.get(EEPROM_ADDR, nvm_);
    if(nvm_.majick1 != MAJICK1 || nvm_.majick2 != MAJICK2)
    {
        resetFactors();
    }
    apply();
    */

#if HAVE_RADIO
    bool ok = radio_.initialize(FREQUENCY, NODEID, NETWORKID);

    #if SERIAL_TRACE
    if(!ok)
        Serial.println("Radio fail");
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
#endif // HAVE_RADIO

/*
    RtcControl::clockLP32k();
    _sleeper.setup();
    kick();
    status();

    //showPIT();
    
    if( digitalReadFast(Pins::RX) == 0 )
        doSleep(); // start in sleep if no serial connected
 */

 }
//------------------------------------------------------------------------------------------
// Add the main program code into the continuous loop() function
void loop()
{
    leds(7);
    Serial.print('X');
/*    
    if(asleep_)
    {
        asleep();
    }
    else
    {
        awake();
    }
*/
}
//------------------------------------------------------------------------------------------
/**
 * While asleep, we use the PIT timer to sleep the CPU for an extended
 * period of time.
 *
 * Evey second, we scan the tare input, and if present for a couple of seconds, swap to
 * "awake" mode.
 *
 * Every SAMPLESEC we measure the battery volts, and the scale weight - if the value is low,
 * we flash the appropriate LED colour(s).
 *
 */
void asleep()
{
    if(digitalReadFast(Pins::TARE)==0)
    {
        doWake();
        return;
    }

    if(--awaketicks_==0)
    {
        if(flashCounter_==0)
        {
            flashCounter_ = FLASHPERSAMPLE;
            loadcell_.power_up();
            delay(400);
            long wt = pollScale(false);
            loadcell_.power_down();

            scaleLow_ = (wt<0);

            auto v = getBattVolts(false);
            battLow_ = (v < 2.4);
            
        }
        else
            --flashCounter_;
                    
        // stuff to do when sleeping
        if(scaleLow_)
        {
            leds(BLUE);
            delay(FLASHMS);
            leds(0);
            
            if(battLow_)
                delay(FLASHMS*2);
        }

        if(battLow_)
        {
            leds(RED);
            delay(FLASHMS);
            leds(0);
        }

        awaketicks_ = ASLEEPTICKS;

        //showPIT();
    }

    //delay(1000);
    _sleeper.sleep();
}
//------------------------------------------------------------------------------------------
void kick()
{
    awaketicks_ = AWAKETICKS;
}
//------------------------------------------------------------------------------------------
void awake()
{
    pollSerial();
    if(asleep_)
        return;

    if(_secondTimer.isExpired())
    {
        if(--awaketicks_==0)
        {
            if(noSleep_)
            {
                kick();
            }
            else
            {                
                // done
                doSleep();
                return;
            }            
        }

        if(_poll)
        {
            pollScale();
        }

        switch(_tareMode)
        {
            case 0:
                // Normal state
                if(digitalReadFast(Pins::TARE)==0)
                {
                    // pressed
                    if(++_tareCounter>=TARECOUNT)
                    {
                        kick();
                        ++_tareMode;
                        _tareCounter=0;
                        leds(BLUE);
                    }
                    else
                    {
                        leds(GREEN);
                    }
                }
                else
                {
                    _tareCounter=0;
                    leds(GREEN);
                }
                break;

            case 1:
                // Pretare
                if(++_tareCounter>=TARECOUNT)
                {
                    kick();
                    ++_tareMode;
                    leds(BLUE);
                }
                else
                {
                    leds(BLUE);
                }
                break;

            case 2:
                leds(MAGENTA);
                kick();
                zero();
                _tareMode = 0;
                break;
        }

        _flashTimer.reset();
    }


    if(leds() && _flashTimer.isExpired())
        leds(0);
}
//------------------------------------------------------------------------------------------
void doSleep()
{
    leds(0);
    loadcell_.power_down();
    asleep_ = true;
    awaketicks_ = ASLEEPTICKS;
    flashCounter_ = 0;
    showPIT();
}
//------------------------------------------------------------------------------------------
void doWake()
{
    asleep_ = false;
    _tareCounter = 0;
    _tareMode = 0;
    noSleep_ = false;
    kick();
}
//------------------------------------------------------------------------------------------
double pollScale(bool print)
{
    loadcell_.power_up();
    if (loadcell_.wait_ready_timeout(1000))
    {
        auto reading = loadcell_.gross(1);
        if(print)
        {
            Serial.printf("Weight: %g\r\n", reading);
        }
        return reading;
    }
    else
    {
        if(print)
            Serial.println("HX711 not found.");
    }
    loadcell_.power_down();
    return 0;
}
//------------------------------------------------------------------------------------------
void pollSerial()
{
    while(Serial.available())
    {
        kick();

        char ch = Serial.read();

        if(ch=='\n')
        {
            *buffptr_ = 0;
            doCommand();
            buffptr_ = buffer_;
            return;
        }
        else if(ch=='\r')
            continue;

        *buffptr_++ = ch;
        if(buffptr_ >= (buffer_+BUFFSIZE))
        {
            Serial.println("OVERFLOW!");
            buffptr_ = buffer_;
        }
    }
}
//------------------------------------------------------------------------------------------
double getBattVolts(bool print)
{
    auto vbatt = analogRead(Pins::VBATT)*4.3/1023;
    if(print)
    {
        Serial.printf("VBatt =%fV\r\n", vbatt );
    }
    return vbatt;
}
//------------------------------------------------------------------------------------------
static void help()
{
    Serial.println(
        "c <wt> Calibrate\n"
        "p Toggle poll\n"
        "R reset\r\n"
        "s Status\r\n"
        "w Stay awake\r\n"
        "Z Zero scale\n"        
        "? Status\r\n"
    );
}
//------------------------------------------------------------------------------------------
void zero()
{
    loadcell_.power_up();
    loadcell_.zero();    
    Serial.printf("Zero= %ld\r\n", loadcell_.get_zero());
    loadcell_.power_down();
    nvm_.zero = loadcell_.get_zero();
    EEPROM.put(EEPROM_ADDR, nvm_);
}
//------------------------------------------------------------------------------------------
void doCommand()
{
    switch(buffer_[0])
    {
        case 'c':
            {
                double f= atof(buffer_+1);
                if(f>0)
                {
                    loadcell_.power_up();
                    delay(400);
                    auto rd = loadcell_.gross();
                    loadcell_.power_down();
                    auto scale = f / rd;
                    if(isnan(scale))
                    {
                        Serial.println("FACTOR BAD");
                    }
                    else
                    {
                        loadcell_.set_scale(scale);
                        nvm_.scale = scale;
                        EEPROM.put(EEPROM_ADDR, nvm_);
                        Serial.printf("Set Scale = %g\r\n", scale);
                    }                                       
                }   
                else
                {
                    Serial.printf("MUST SUPPLY FACTOR %g [%s]\r\n", f, buffer_);
                    break;                    
                }                                     
            }                
            break;
            
        case 'p':
            _poll ^= 1;
            Serial.printf("Poll mode=%d\r\n", _poll);
            break;

        case 'R':
            Serial.println("Resetting factors..");
            resetFactors();
            apply();
            status();
            break;
            
        case 's':
        Serial.println("Zzzz..");
            delay(500);
            doSleep();
            break;

        case 'w':
            noSleep_ = !noSleep_;
            Serial.printf("NoSleep = %d\r\n", noSleep_);
            break;

        case 'Z':
            zero();
            break;

        case '?':
            status();
            break;

        default:
            Serial.printf("Unk Cmd=0x%02x (%C)\r\n", (uint8_t)buffer_[0], buffer_[0]);
            help();
            break;
    }
}
//------------------------------------------------------------------------------------------
void status()
{
    Serial.printf(
        "offs=%ld\r\nscale=%g\r\n",
        nvm_.zero,
        nvm_.scale
        );
    getBattVolts();
    pollScale();
    loadcell_.power_up();
    Serial.printf(
        "abs=%ld gross=%ld\r\n",
        loadcell_.absolute(), loadcell_.grossRaw()
        );
    loadcell_.power_down();
}
//------------------------------------------------------------------------------------------

