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
#include <radio.h>

#define TEST_SCALE 0

const bool POWERSW_ON = true;
const bool POWERSW_OFF = false;

const uint16_t MAJICK1 = 0xDEAD;
const uint16_t MAJICK2 = 0xBEEB;
const uint16_t SLEEPFLASHSEC = 30;
const uint16_t FLASHPERSAMPLE = 10;

const int EEPROM_ADDR = 0;

struct NVM
{
    uint16_t majick1;
    long zero;
    long tare;
    double scale;
    double alarm;
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
uint8_t tareCounter_;
uint8_t tareMode_;
uint8_t poll_;
uint8_t lastLeds_;
const uint16_t ASLEEPTICKS = 20;    ///< how many ticks before we check scales/batt
const uint16_t AWAKETICKS  = 60;    ///< sec before we sleep after no activity
uint16_t awaketicks_;
uint16_t flashCounter_;
bool battLow_;
bool scaleLow_;
bool noSleep_;

const uint16_t MINSENDRATE = 90;
uint16_t sendCounter_;
double lastSendWeight_;
const double WEIGHT_HYSTERESIS = 10.0;

static double pollScale(bool print=true, bool controlPower=true);
static double getBattVolts(bool print=true);
static void pollSerial();
static void doCommand();
static void asleep();
static void awake();
static void doSleep();
static void doWake();
static void zero();
static void tare();
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
const struct PinState
{
    uint8_t pin;
    uint8_t sleep;
    uint8_t wake;
    
} _states[] = 
{
  //{ Pins::MX711_DOUT, INPUT_PULLUP, INPUT_PULLUP }, // really!
  { Pins::MX711_SCK,  INPUT_PULLUP, OUTPUT }, 
  //{ Pins::MISO,       INPUT_PULLUP, INPUT_PULLUP },
  { Pins::MOSI,       INPUT_PULLUP, OUTPUT },
  { Pins::SCK,        INPUT_PULLUP, OUTPUT },
  { 255 }
};
//------------------------------------------------------------------------------------------
void pinState(bool st)
{
    //Serial.printf("Set pins=%d\n", st );
    for(auto wp = _states; wp->pin != 255; ++wp)
        pinMode(wp->pin, st ? wp->wake : wp->sleep);
}
//------------------------------------------------------------------------------------------
static void leds(uint8_t c)
{
    lastLeds_ = c;
    c = ~c; // flip the bits
    digitalWriteFast(Pins::R, (c&1) != 0); // writeFast assumes 0/1 only
    digitalWriteFast(Pins::G, (c&2) != 0);
    digitalWriteFast(Pins::B, (c&4) != 0);

}
//------------------------------------------------------------------------------------------
static uint8_t leds()
{
    return lastLeds_;
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
    Serial.println("RESETTING NVM");
    nvm_.majick1    = MAJICK1;
    nvm_.zero       = 0;
    nvm_.scale      = 1.0;
    nvm_.tare       = 0;
    nvm_.alarm      = 100.0;
    nvm_.majick2    = MAJICK2;
    EEPROM.put(EEPROM_ADDR, nvm_);    
}    
//------------------------------------------------------------------------------------------
void apply()
{
    loadcell_.set_scale(nvm_.scale);
    loadcell_.set_zero(nvm_.zero);    
    loadcell_.set_tare(nvm_.tare);    
}    
//------------------------------------------------------------------------------------------
// The setup() function runs once each time the micro-controller starts
void setup()
{
    SPI.swap();
    Serial.begin(115200);
    delay(10);
    Serial.print("\f*START*\r\n");

    pinState(0);

    analogReference(INTERNAL);
    leds(0);
    pinMode(Pins::R, OUTPUT);
    pinMode(Pins::G, OUTPUT);
    pinMode(Pins::B, OUTPUT);
    pinMode(Pins::TARE, INPUT_PULLUP);
#if HAVE_RATE
    pinMode(Pins::MX711_RATE, OUTPUT);
    digitalWriteFast(Pins::MX711_RATE, 1); // 0=10Hz, 1=80Hz
#endif

    //
    // Make sure the radio is properly reset
    pinMode(Pins::POWER_SW, OUTPUT);
    digitalWriteFast(Pins::POWER_SW, POWERSW_OFF);
    delay(500);
    digitalWriteFast(Pins::POWER_SW, POWERSW_ON);
    delay(500);
    pinState(1);


    //Serial.swap();
    
    for(int i=0; i<9; ++i)
    {
        leds(i);
        delay(100);
    }

    loadcell_.begin();

    EEPROM.get(EEPROM_ADDR, nvm_);
    if(nvm_.majick1 != MAJICK1 || nvm_.majick2 != MAJICK2)
    {
        resetFactors();
    }
    apply();

#if HAVE_RADIO
    initRadio();
#endif // HAVE_RADIO


    RtcControl::clockLP32k();
    _sleeper.setup();

    kick();
    status();

    //showPIT();
    
    //if( digitalReadFast(Pins::RX) == 0 )
    //    doSleep(); // start in sleep if no serial connected

    //doWake();

#if TEST_SCALE
    digitalWriteFast(Pins::POWER_SW, 1);
    loadcell_.power_up();
#endif

    wdt_enable(WDT_PERIOD_8KCLK_gc); // 8s - not same codes as basic arduino

 }
//------------------------------------------------------------------------------------------
// Add the main program code into the continuous loop() function
void loop()
{

#if TEST_SCALE    
    //pollScale(true, false);
    long a = loadcell_.read();
    Serial.printf("Abs=%ld\r\n", a);
    delay(500);
#else
    if(asleep_)
    {
        //Serial.print('$'); delay(5);
        asleep();
    }
    else
    {
        //Serial.print('@');
        awake();
    }
    wdt_reset();     
#endif
}
//------------------------------------------------------------------------------------------
void pollMaybeSend(bool force, bool log=false)
{
#if USE_POWER_SW
    digitalWriteFast(Pins::POWER_SW, POWERSW_ON);
    pinState(1);
    loadcell_.begin();
    delay(10);
#endif

    double wt = pollScale(log);
    scaleLow_ = (wt< nvm_.alarm);

    auto v = getBattVolts(true);
    battLow_ = (v < 2.4);
    
    uint8_t st = (battLow_<<1)|scaleLow_;

    Serial.printf(">> wt=%g alm=%d BV=%g\n", wt, st, v);

    //
    // Now consider radio actions
    if( force
        || abs(wt-lastSendWeight_) > WEIGHT_HYSTERESIS // we cal in grams
        )
    {
        lastSendWeight_ = wt;
        sendRadio(wt, v, st);
    }

#if USE_POWER_SW
    delay(10);

    digitalWriteFast(Pins::POWER_SW, POWERSW_OFF);
    pinState(0);
#endif
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
    //Serial.printf("%d", digitalReadFast(Pins::TARE));
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
            //
            // Now consider radio actions
            if(sendCounter_==0
                )
            {
                pollMaybeSend(true);
                sendCounter_ = MINSENDRATE-1;
            }
            else
            {
                pollMaybeSend(false);
                --sendCounter_;
            }

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
    //Serial.printf("Sleeping, powersw=%d\r\n", digitalReadFast(Pins::POWER_SW));
    //delay(100);
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

        if(poll_)
        {
            pollMaybeSend( 
                    /*force=*/false, 
                    /*log=*/ true
                    );
        }

        switch(tareMode_)
        {
            case 0:
                // Normal state
                if(digitalReadFast(Pins::TARE)==0)
                {
                    // pressed
                    if(++tareCounter_>=TARECOUNT)
                    {
                        kick();
                        ++tareMode_;
                        tareCounter_=0;
                        leds(BLUE);
                    }
                    else
                    {
                        leds(GREEN);
                    }
                }
                else
                {
                    tareCounter_=0;
                    leds(GREEN);
                }
                break;

            case 1:
                // Pretare
                if(++tareCounter_>=TARECOUNT)
                {
                    kick();
                    ++tareMode_;
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
                tare();
                tareMode_ = 0;
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
    digitalWriteFast(Pins::POWER_SW, POWERSW_OFF);
    pinState(0);
    leds(0);
    asleep_ = true;
    awaketicks_ = ASLEEPTICKS;
    flashCounter_ = 0;
    showPIT();
}
//------------------------------------------------------------------------------------------
void doWake()
{
    digitalWriteFast(Pins::POWER_SW, POWERSW_ON);
    delay(5);
    asleep_ = false;
    tareCounter_ = 0;
    tareMode_ = 0;
    noSleep_ = false;
    kick();
}
//------------------------------------------------------------------------------------------
double pollScale(bool print, bool controlPower = true)
{
    if(controlPower)
        loadcell_.power_up();

    if (loadcell_.wait_ready_timeout(1000))
    {
        loadcell_.poll();
        auto reading = loadcell_.nett();
        if(print)
        {
            Serial.println();
            Serial.println("-- Status --");
            Serial.printf("Abs:    %ld\n", loadcell_.absolute());
            Serial.printf("Zero:   %ld\n", loadcell_.get_zero() );
            Serial.printf("Gross:  %gg  (%ld)\n", loadcell_.gross(), loadcell_.grossRaw() );
            Serial.printf("Tare:   %ld\n", loadcell_.get_tare() );
            Serial.printf("Nett:   %gg  (%ld)\n", loadcell_.nett(), loadcell_.nettRaw());
        }
        return reading;
    }
    else
    {
        Serial.println("HX711 not found.");
    }

    if(controlPower)
        loadcell_.power_down();

    return -1;
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
    ADC0.CTRLA |= ADC_ENABLE_bm;
    auto vbatt = analogRead(Pins::VBATT)*3.3/1023;
    ADC0.CTRLA &= ~ADC_ENABLE_bm;
    if(print)
    {
        Serial.printf("VBatt =%fV\r\n", vbatt ); delay(10);
    }
    return vbatt;
}
//------------------------------------------------------------------------------------------
static void help()
{
    Serial.println(
        "\n"
        "a <wt> Set alarm level (scaled units)\n"
        "c <wt> Calibrate (scaled units)\n"
        "P Toggle Auto poll\n"
        "p poll\n"
        "R reset\r\n"
        "S Sleep\r\n"
        "t Tare\r\n"
        "w Stay awake\r\n"
        "Z Zero scale\n"        
        "? Status\r\n"
    );
}
//------------------------------------------------------------------------------------------
void zero()
{
    Serial.println("Zeroing...");
    loadcell_.zero();    
    Serial.printf("Zero= %ld\r\n", loadcell_.get_zero());
    nvm_.zero = loadcell_.get_zero();
    nvm_.tare = loadcell_.get_tare();
    EEPROM.put(EEPROM_ADDR, nvm_);
}
//------------------------------------------------------------------------------------------
void tare()
{
    Serial.println("Tareing...");
    loadcell_.tare();    
    Serial.printf(
                    "Zero= %ld Tare=%ld\r\n", 
                    loadcell_.get_zero(), 
                    loadcell_.get_tare()
                    );

    nvm_.tare = loadcell_.get_tare();
    EEPROM.put(EEPROM_ADDR, nvm_);
}
//------------------------------------------------------------------------------------------
void doCommand()
{
    switch(buffer_[0])
    {
        case 'a':
            {
                double f= atof(buffer_+1);
                if(!isnan(f))
                {
                    nvm_.alarm = f;
                    EEPROM.put(EEPROM_ADDR, nvm_);
                    Serial.printf("Set Alarm = %g\r\n", f);
                }   
                else
                {
                    Serial.printf("MUST supply real %g [%s]\r\n", f, buffer_);
                    break;                    
                }                                     
            }                
            break;

        case 'c':
            {
                double f= atof(buffer_+1);
                if(f>0 && !isnan(f))
                {
                    loadcell_.poll(32);
                    auto rd = loadcell_.grossRaw();
                    Serial.printf("grossRaw=%ld\n", rd);
                    auto scale = f / rd;
                    if(isnan(scale))
                    {
                        Serial.printf("FACTOR BAD [%s] -> %f", buffer_+1, f);
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
            
        case 'P':
            poll_ ^= 1;
            Serial.printf("Auto Poll mode=%d\r\n", poll_);
            break;

        case 'R':
            Serial.println("Resetting factors..");
            resetFactors();
            apply();
            status();
            break;
            
        case 'S':
        Serial.println("Zzzz..");
            delay(500);
            doSleep();
            break;

        case 't':
            tare();
            break;

        case 'w':
            noSleep_ = !noSleep_;
            Serial.printf("NoSleep = %d\r\n", noSleep_);
            break;

        case 'Z':
            zero();
            break;

        case 'p':
            pollMaybeSend(true,true);
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
        "zero =%ld\n"
        "tare =%ld\n"
        "tare =%ld\n"
        "scale=%g\n"
        "alarm=%f\n",
        nvm_.zero,
        nvm_.tare,
        nvm_.scale,
        nvm_.alarm
        );
    getBattVolts();
    pollScale();
    loadcell_.power_up();
    Serial.printf(
        "abs  =%ld\n"
        "gross=%g\n"
        "nett =%g\n",
        loadcell_.absolute(), loadcell_.gross(), loadcell_.nett()
        );
    loadcell_.power_down();
}
//------------------------------------------------------------------------------------------
/**
 * PIT Interrupt vector
 */
ISR(RTC_PIT_vect)
{
    if( (RTC.PITINTCTRL & 1) && (RTC.PITINTFLAGS & 1) )
    {
    }
    RTC.PITINTFLAGS = 1;
    //Serial.print(RTC.PITINTFLAGS ? "$" : ".");
}
//------------------------------------------------------------------------------------------
