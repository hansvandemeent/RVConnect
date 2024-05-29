#pragma region Version
  /*  Application: RVConnect
  0.0.0 29-05-2024  Initial version
  */
#pragma endregion Version

#include <thingsml_http.h>
#include <Sodaq_R4X.h> 
#include <Sodaq_wdt.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Sodaq_LSM303AGR.h>
#include <RTCZero.h>

#include "LedColor.h"
#include "TracePrint.h"
#include "MyTime.h"
#include "ublox.h"
#include SETTINGSFILE 

#define MIN_TIME_BETWEEN_TRANSMITS 30 // Seconds: 30
#define TIME_TO_STAY_AWAKE 10 // Seconds: 10
#define SLEEP_TIME_WHEN_RETRY 60 // Seconds: 60

#define STATUS_CLEARED      0b00000000
#define GPS_FIX             0b00000001
#define MOTION_DETECTED     0b00000010
#define SENSOR_ERROR        0b00010000

#define MAX_RTC_EPOCH_OFFSET 25
#define CONSOLE_STREAM   SerialUSB

#ifndef NBIOT_BANDMASK
#define NBIOT_BANDMASK BAND_MASK_UNCHANGED
#endif

#define BUFFER_SIZE 1024
#define STARTUP_DELAY 5000

static Sodaq_R4X r4x;
static Sodaq_LSM303AGR accelerometer;
static Sodaq_SARA_R4XX_OnOff saraR4xxOnOff;
TracePrint trace;
RTCZero rtc;
Time time;

SenMLPack device(DEVICE_URN);
SenMLIntRecord status(SENML_NAME_COUNTER, SENML_UNIT_BIT);
SenMLFloatRecord temperature(SENML_NAME_TEMPERATURE, SENML_UNIT_DEGREES_CELSIUS);
SenMLIntRecord humidity(SENML_NAME_HUMIDITY, SENML_UNIT_RELATIVE_HUMIDITY);
SenMLIntRecord latitude(SENML_NAME_LATTITUDE, SENML_UNIT_DEGREES_LATITUDE);
SenMLIntRecord longitude(SENML_NAME_LONGITUDE, SENML_UNIT_DEGREES_LONGITUDE);
SenMLFloatRecord battery1(SENML_NAME_BATTERY_VOLTAGE, SENML_UNIT_VOLT);
//SenMLFloatRecord battery2(SENML_NAME_BATTERY_VOLTAGE, SENML_UNIT_VOLT);
SenMLFloatRecord system_battery(SENML_NAME_BATTERY_VOLTAGE, SENML_UNIT_VOLT);

static bool isReady;
static bool isRtcInitialized;
static bool isSleeping;
static int64_t rtcEpochDelta; // set in setNow() and used in getGpsFixAndTransmit() for correcting time in loop
char buffer[BUFFER_SIZE] = {0};

volatile bool updateOnTheMoveTimestampFlag;
unsigned long previousMillis = 0;
const long interval = 60000;

enum SYSTEMSTATUS {
    SYSTEM_NONE,
    SYSTEM_INIT,
    SYSTEM_ACTIVE,
    SYSTEM_SEND_OK,
    SYSTEM_SEND_ERROR,
    SYSTEM_ERROR,
    SYSTEM_GPS_FIX,
    SYSTEM_GPS_NO_FIX,
    SYSTEM_MOTION_DETECTED
};
SYSTEMSTATUS systemStatus;

enum EVENT {
    idleEvent,
    motionEvent,
    timerEvent,
    errorEvent
};
static EVENT event;

uint8_t reportedStatus;
uint64_t lastTransmit;

//void setAccelerometerTempSensorActive(bool on);
void initOnTheMove();
void setGpsActive(bool on);
//uint8_t getBatteryVoltage();

void setSystemStatus(SYSTEMSTATUS s);

#pragma region RTC

    uint32_t getNow() {
    return rtc.getEpoch();
    }

    void setNow(uint32_t newEpoch) {
        uint32_t currentEpoch = getNow();
        rtcEpochDelta = newEpoch - currentEpoch;
        rtc.setEpoch(newEpoch);
        isRtcInitialized = true;
    }

    void rtcAlarmHandler() {
        isSleeping = false; // Waked up after (deep) sleep
        //timerFlag = true;
        event = timerEvent;
    }

    void initRtc() {
        rtc.begin();
        // Set default time and date. Update when GPS fix
        rtc.setTime(0, 0, 0);
        rtc.setDate(1, 1, 23);
        rtc.enableAlarm(RTCZero::MATCH_HHMMSS); 
        rtc.attachInterrupt(rtcAlarmHandler);
    }

    void offsetAlarmTime(uint16_t offset) {
        TRACE("->");
        uint8_t offsetHours;
        uint8_t offsetMinutes;
        uint8_t offsetSeconds;
        uint8_t hours = rtc.getHours();
        uint8_t minutes = rtc.getMinutes();
        uint8_t seconds = rtc.getSeconds();
        offsetHours = offset / 3600;
        offsetMinutes = (offset - (offsetHours * 3600)) / 60;
        offsetSeconds = offset - (offsetHours * 3600) - (offsetMinutes * 60);
        hours += offsetHours;
        minutes += offsetMinutes;
        seconds += offsetSeconds;
        if (seconds > 59) {
            seconds -= 60;
            minutes ++;
        }
        if (minutes > 59) {
            minutes -= 60;
            hours ++;
        }
        if (hours > 23) {
            hours -= 24;
        }
        TRACE("Offset: %02i:%02i:%02i Alarm at: %02i:%02i:%02i", offsetHours, offsetMinutes, offsetSeconds, hours, minutes, seconds);
        rtc.setAlarmTime(hours, minutes, seconds);
        TRACE("<-");
    }

#pragma endregion RTC

#pragma region POWER

    #define BATTERY_ANALOG_1 PIN_A0
    #define BATTERY_ANALOG_2 PIN_A1
    #define ADC_AREF 3.3f
    #define BATVOLT_R1 4.7f
    #define BATVOLT_R2 10.0f
    #define BATVOLT_R3 10.0f
    #define BATVOLT_R4 1.0f

    struct Battery_Values {
        float battery1;
        float battery2;
        float system_battery;
    };
    static Battery_Values batteryValues;

    void initSleep() {
        // Set the sleep mode
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    }

    void systemSleep(bool deepSleep) {
        TRACE("->");

        setSystemStatus(SYSTEM_NONE);
        setGpsActive(false); // explicitly disable after resetting the pins           
        
        isSleeping = true; // Flag while sleeping

        if (deepSleep) {
            TRACE("*** Deepsleep active ***");
            trace.reset();
            //SerialUSB.flush();
            //SerialUSB.end();
            //USBDevice.detach();
            //USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB

            noInterrupts();
            SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
            interrupts();
            sodaq_wdt_reset();
            __WFI(); // SAMD sleep
            // Enable systick interrupt
            SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
            interrupts();
        }
        else {
            #ifndef TRACEOFF 
                while (isSleeping) {
                    TRACE("...");
                    sodaq_wdt_safe_delay(10000);
                };
            #endif
        }
    }

    void initBOD33() {
        SYSCTRL->BOD33.bit.LEVEL = 0x07;    // ~1.7 Volt
        SYSCTRL->BOD33.bit.ACTION = 1;      // Go to Reset
        SYSCTRL->BOD33.bit.ENABLE = 1;      // Enabled
        SYSCTRL->BOD33.bit.HYST = 1;        // Hysteresis on
        while (!SYSCTRL->PCLKSR.bit.B33SRDY) {
            /* Wait for synchronization */
        }
    }

    void getBatteryVoltages() {
        TRACE("->");

        float voltage;

        if (SIMULATE_SENSORS) {
            batteryValues.battery1 = 12.6;
            batteryValues.battery2 = 12.2;
        }
        else {
            voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R3 + BATVOLT_R4) / BATVOLT_R4 * (float)analogRead(BATTERY_ANALOG_1));
            batteryValues.battery1 = voltage /1000;
            voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R3 + BATVOLT_R4) / BATVOLT_R4 * (float)analogRead(BATTERY_ANALOG_2));
            batteryValues.battery2 = voltage /1000;
        }

        voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BAT_VOLT));
        batteryValues.system_battery =  voltage  / 1000.0;

        TRACE("B1: %f B2: %f BS: %f", batteryValues.battery1, batteryValues.battery2, batteryValues.system_battery);
        
        TRACE("<-");
    }

#pragma endregion POWER

#pragma region GPS

    #define GPS_TIME_VALIDITY 0b00000011 // date and time (but not fully resolved)
    #define GPS_FIX_FLAGS 0b00000001 // just gnssFixOK
    #define GPS_COMM_CHECK_TIMEOUT 3 // seconds

    struct GpsRecord {
        int32_t latitude;
        int32_t longitude;
        uint8_t satelitesCount;
        uint32_t horizontalAccuracy;
        uint32_t verticalAccuracy;
    };
    union SensorData {
        uint8_t raw[20];

        struct {
            uint32_t time;
            int32_t latitude;
            int32_t longitude;
            uint32_t horizontalAccuracy;
            uint32_t verticalAccuracy;
        } __attribute__((packed));

        void init() {
            memset(this->raw, 0, sizeof(this->raw));
        }
    };
    UBlox ublox; 
    GpsRecord gpsRecord;
    SensorData sensorData;
    uint8_t timeOut = GPS_TIMEOUT;           // GPS fix timeout
    bool isPendingGpsRecordNew;     // this is set to true only when pendingReportDataRecord is written by the delegate
    uint16_t navPvtCounter = 0;
    static bool isGpsInitialized;
    bool gpsFix;

    void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt) {
        sodaq_wdt_reset();

        if (!isGpsInitialized) {
            TRACE("<- delegateNavPvt exiting because GPS is not initialized.");
            return;
        }

        ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
            NavPvt->year, NavPvt->month, NavPvt->day,
            NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
            NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

        if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
            uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

            // check if there is an actual offset before setting the RTC
            if (abs((int64_t)getNow() - (int64_t)epoch) > MAX_RTC_EPOCH_OFFSET) {
                setNow(epoch);
            }
        }

        // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
        if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
            gpsRecord.latitude = NavPvt->lat;
            gpsRecord.longitude = NavPvt->lon;
            gpsRecord.satelitesCount = NavPvt->numSV;
            gpsRecord.horizontalAccuracy = NavPvt->hAcc;
            gpsRecord.verticalAccuracy = NavPvt->vAcc;
            isPendingGpsRecordNew = true;
        }
    }

    void setGpsActive(bool on) {
        TRACE("-> %s", on ? "true":"false");
        sodaq_wdt_reset();

        if (on) {
            digitalWrite(GPS_ENABLE, HIGH);

            ublox.enable();
            ublox.flush();

            sodaq_wdt_safe_delay(100);

            PortConfigurationDDC pcd;

            uint8_t maxRetries = 6;
            int8_t retriesLeft;

            retriesLeft = maxRetries;
            while (!ublox.getPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
                TRACE("Retrying ublox.getPortConfigurationDDC(&pcd)...");
                sodaq_wdt_safe_delay(15);
            }
            if (retriesLeft == -1) {
                TRACE("ublox.getPortConfigurationDDC(&pcd) failed!");
                TRACE("<- Failed");
                return;   
            }

            pcd.outProtoMask = 1; // Disable NMEA
            retriesLeft = maxRetries;
            while (!ublox.setPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
                TRACE("Retrying ublox.setPortConfigurationDDC(&pcd)...");
                sodaq_wdt_safe_delay(15);
            }
            if (retriesLeft == -1) {
                TRACE("ublox.setPortConfigurationDDC(&pcd) failed!");
                TRACE("<- Failed");
                return;
            }

            ublox.CfgMsg(UBX_NAV_PVT, 1); // Navigation Position Velocity TimeSolution
            ublox.funcNavPvt = delegateNavPvt;
        }
        else {
            ublox.disable();
            digitalWrite(GPS_ENABLE, LOW);
        }
        TRACE("<- OK");
    }

/*
    void getSensorData(SensorData& sensorData) {
        TRACE("->");
        sensorData.time = getNow();
        sensorData.latitude = gpsRecord.latitude;
        sensorData.longitude = gpsRecord.longitude;
        sensorData.horizontalAccuracy = gpsRecord.horizontalAccuracy;
        sensorData.verticalAccuracy = gpsRecord.verticalAccuracy;
        TRACE("<-");
    }
*/

    void initGps()
    {
        TRACE("->");

        if (!SIMULATE_GPS) {
            setSystemStatus(SYSTEM_INIT);
            sodaq_wdt_safe_delay(1000); 
            pinMode(GPS_ENABLE, OUTPUT);
            pinMode(GPS_TIMEPULSE, INPUT);
            digitalWrite(GPS_ENABLE, HIGH);
            ublox.enable();
            ublox.flush();

            uint32_t startTime = getNow();
            bool found = false;
            while (!found && (getNow() - startTime <= GPS_COMM_CHECK_TIMEOUT)) {
                sodaq_wdt_reset();
                found = ublox.exists();
            }

            if (found) {
                setGpsActive(true); // properly turn on before returning
                isGpsInitialized = true;
                sodaq_wdt_safe_delay(1000); 
                TRACE("<- GPS Found");
                return;
            }

            // turn off before returning in case of failure
            setGpsActive(false);
            setSystemStatus(SYSTEM_ERROR);
            
            isGpsInitialized = false;
            TRACE("<- GPS Not Found");
            return;
        }
        TRACE("<-");
    }

    bool getLocation()
    {
        TRACE("->");
        if (SIMULATE_GPS) {
            sensorData.latitude = 521900000 + (rand() % 10000); 
            sensorData.longitude = 50510000 + (rand() % 10000); 
            TRACE("Lat: %i Lon: %i", sensorData.latitude, sensorData.longitude);
        }
        else {   
            
            setSystemStatus(SYSTEM_ACTIVE);
            sodaq_wdt_safe_delay(1000); 
            initGps();
            if (!isGpsInitialized) {
                TRACE("<- GPS is not initialized, exiting...");
                return false;
            }

            setSystemStatus(SYSTEM_ACTIVE);
            gpsFix = false;
            setGpsActive(true);

            navPvtCounter = 0;
            gpsRecord.satelitesCount = 0; 
            uint32_t startTime = getNow();
            #ifndef GPSOFF
                // Try to find a fix
                while ((getNow() - startTime <= timeOut) && (gpsRecord.satelitesCount < 3)) {
                    sodaq_wdt_reset();
                    uint16_t bytes = ublox.available();
                    if (bytes) {
                        rtcEpochDelta = 0;
                        isPendingGpsRecordNew = false;
                        ublox.GetPeriodic(bytes); // calls the delegate method for passing results
                        startTime += rtcEpochDelta; // just in case the clock was changed (by the delegate in ublox.GetPeriodic)
                        if (isPendingGpsRecordNew) {
                            gpsFix = true;
                        }
                    }
                }
            #endif

            //getSensorData(sensorData);  

            setGpsActive(false); // turn off gps as soon as it is not needed

            if (gpsFix) {
                TRACE("GPS Fix");
                reportedStatus &= ~GPS_FIX;
                setSystemStatus(SYSTEM_GPS_FIX);
            }
            else {
                TRACE("No GPS Fix");   
                reportedStatus &= ~GPS_FIX;
                setSystemStatus(SYSTEM_GPS_NO_FIX);
            }
            
            
        }
        TRACE("<-");
        return true;
    }

#pragma endregion GPS

#pragma region ACCELEROMETER

    void accelerometerInt1Handler()
    {
        //TRACE("->");
        if (digitalRead(ACCEL_INT1)) {
            isSleeping = false;
            //motionFlag = true;
            event = motionEvent;
            //0.0.13
            updateOnTheMoveTimestampFlag = true;
        }
        //TRACE("<-");
    }

    void initAccelerometer() {
        TRACE("->");
        
        accelerometer.disableMagnetometer();
        pinMode(ACCEL_INT1, INPUT);
        attachInterrupt(ACCEL_INT1, accelerometerInt1Handler, CHANGE);

        GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
            GCLK_CLKCTRL_GEN_GCLK1 |
            GCLK_CLKCTRL_CLKEN;

        accelerometer.enableAccelerometer();
        sodaq_wdt_safe_delay(100);

        accelerometer.enableInterrupt1(
            Sodaq_LSM303AGR::XHigh | Sodaq_LSM303AGR::XLow | Sodaq_LSM303AGR::YHigh | Sodaq_LSM303AGR::YLow | Sodaq_LSM303AGR::ZHigh | Sodaq_LSM303AGR::ZLow,
            - 1.0 ,
            1.0,
            Sodaq_LSM303AGR::MovementRecognition);

        TRACE("<-");
    }

/*
    int8_t getBoardTemperature() {
        TRACE("->");

        setAccelerometerTempSensorActive(true);
        int8_t temp = accelerometer.getTemperature();
        //0.0.13
        //setAccelerometerTempSensorActive(false);

        TRACE("<- Board temperature = %i", temp);
        return temp;
    }
*/
#pragma endregion ACCELEROMETER

#pragma region DHT
    
    DHT_Unified dht(2, DHT22);

    struct DHT_Values {
        float temperature = 0;  
        int humidity = 0; 
    };
    static DHT_Values dhtValues;

    bool getDHT_Values(DHT_Values &dhtValues) {
    
        sensors_event_t event;  
        dht.temperature().getEvent(&event);

        if (isnan(event.temperature)) {
            reportedStatus |= SENSOR_ERROR;
            return false;
        }
        else {
            dhtValues.temperature = event.temperature;  
        }
        dht.humidity().getEvent(&event);
        if (isnan(event.relative_humidity)) {
            reportedStatus |= SENSOR_ERROR;
            return false;
        }
        else {
            dhtValues.humidity = (int)event.relative_humidity;
        }
        
        return true;   
    }

    int initDHT(DHT_Values &dhtValues) {
        TRACE("->");

        dht.begin();
        sensor_t sensor;
        sensor.min_delay = 2000;

        TRACE("<- DHT OK");
        return true;
    }

    bool getTemperatureHumidity() {
        TRACE("->");
        
        if (!getDHT_Values(dhtValues)) {
            return false;
        }

        TRACE("T: %f H: %i", dhtValues.temperature, dhtValues.humidity);

        TRACE("<-");
        return true;
    }

#pragma endregion DHT

#pragma region KPN

    bool initModem() {
        TRACE("->");

        #define MODEM_STREAM     Serial1
        MODEM_STREAM.begin(r4x.getDefaultBaudrate());
        r4x.setDiag(CONSOLE_STREAM);
        r4x.init(&saraR4xxOnOff, MODEM_STREAM);

        TRACE("<-");
        return true;
    }

    bool connectKPN() {
        TRACE("->");

        if (SIMULATE_SEND) {
            isReady = true;
        }
        else {
            r4x.on();
            isReady = r4x.connect(APN, CURRENT_URAT, CURRENT_MNO_PROFILE, CURRENT_OPERATOR, BAND_MASK_UNCHANGED, NBIOT_BANDMASK);
         }
           
        TRACE(isReady ? "Network connected" : "Network connection failed");  
        TRACE("<-");
        return isReady;
    }

    void buildSenML() {
        TRACE("->");  
        
        memset(buffer, '\0', sizeof(buffer)); 

        if (event == motionEvent) {
            latitude.set((int)gpsRecord.latitude);
            longitude.set((int)gpsRecord.longitude);
        }

        if (event == timerEvent) { 
            status.set(reportedStatus);
            latitude.set((int)gpsRecord.latitude);
            longitude.set((int)gpsRecord.longitude);
            temperature.set((float)dhtValues.temperature); 
            humidity.set((int)dhtValues.humidity);
            battery1.set((float)batteryValues.battery1); 
            //battery2.set((float)batteryValues.battery2); 
            system_battery.set((float)batteryValues.system_battery); 
        }

        TRACE("<-");
    }

    bool uploadKPN(){
        TRACE("->");

        connectKPN();

        if (SIMULATE_SEND) {

        }
        else {
            if (isReady) {
                setSystemStatus(SYSTEM_ACTIVE);
                
                int len = ThingsML::httpPost(buffer, BUFFER_SIZE, DEVICE_KEY, HTTP_HOST, HTTP_PATH, device);
                
                uint8_t socketId = r4x.socketCreate(0, TCP);
                bool result = r4x.socketConnect(socketId, HTTP_IP, HTTP_PORT);
                TRACE("socketConnect %s", result ? "OK": "Failed");
                if (result == false) {
                    setSystemStatus(SYSTEM_SEND_ERROR); 
                    return result;
                }

                size_t size = r4x.socketWrite(socketId, (uint8_t *) buffer, len);
                TRACE("socketWrite: %s, %i bytes", buffer, size);

                r4x.socketWaitForRead(socketId);
                TRACE("Receiving message...");

                int receiveLength = r4x.socketRead(socketId, (uint8_t *) buffer, BUFFER_SIZE);
                TRACE("Message response length: %i", receiveLength);

                result = r4x.socketClose(socketId);
                TRACE("socketClose %s", result ? "OK": "Failed");
                if (result == false) {
                    setSystemStatus(SYSTEM_SEND_ERROR); 
                    return result;
                }
        
                if (receiveLength > 0) {
                    buffer[receiveLength] = 0; // Null terminate the string
                    TRACE("Message response: %s", buffer);
                }

                lastTransmit = rtc.getEpoch();
                setSystemStatus(SYSTEM_SEND_OK);
                TRACE("Message sending finished.");
            }
        }
        r4x.off();
        TRACE("<-");
        return true;
    }

#pragma endregion KPN

void setSystemStatus(SYSTEMSTATUS s) {

    LedColor ledColor = getLedColor();

    switch (s) {
        case SYSTEM_NONE: 
            setLedColor(NONE);
        break;
        case SYSTEM_INIT: 
            setLedColor(YELLOW);
        break;
        case SYSTEM_ACTIVE:
            setLedColor(WHITE);
        break;
        case SYSTEM_SEND_OK:
            setLedColor(GREEN);
        break;
        case SYSTEM_SEND_ERROR:
            for(int c= 0; c < 5; c++) {
                setLedColor(GREEN);
                sodaq_wdt_safe_delay(500); 
                setLedColor(RED);
                sodaq_wdt_safe_delay(500);   
            }
            setLedColor(ledColor);
        break;
        case SYSTEM_ERROR:
            setLedColor(RED);
            sodaq_wdt_safe_delay(5000); 
            setLedColor(ledColor);
        break;
        case SYSTEM_GPS_FIX:
            setLedColor(BLUE);
            sodaq_wdt_safe_delay(5000);  
            setLedColor(ledColor);
        break;
        case SYSTEM_GPS_NO_FIX:
            
            for(int c= 0; c < 5; c++) {
                setLedColor(YELLOW);
                sodaq_wdt_safe_delay(500); 
                setLedColor(RED);
                sodaq_wdt_safe_delay(500);   
            }
            setLedColor(ledColor);
        break;
        case SYSTEM_MOTION_DETECTED:
            setLedColor(MAGENTA); 
            sodaq_wdt_safe_delay(5000);  
            setLedColor(ledColor);
        break;
        default:
        break;
    }
}

void sendMessage() {
    TRACE("->");
    if (event == motionEvent) {
        setSystemStatus(SYSTEM_MOTION_DETECTED);
        // filter motions within x minutes
        if (rtc.getEpoch() > (lastTransmit + (MIN_TIME_BETWEEN_TRANSMITS))) {
            TRACE("MotionEvent after timeout");

            getLocation();  
            //getBatteryVoltages();
            //getTemperatureHumidity();

            buildSenML(); 

            if (uploadKPN()) {
                // Send after motion has stopped to get last location
                offsetAlarmTime(SLEEP_TIME_AFTER_MOTION_DETECTED);
            }
            else {
                offsetAlarmTime(SLEEP_TIME_WHEN_RETRY);
            }
        }
        else {
            TRACE("MotionEvent to soon");
        }  
    }

    if (event == timerEvent) {
        TRACE("TimerEvent");
        setSystemStatus(SYSTEM_ACTIVE);

        getLocation();  
        getBatteryVoltages();
        getTemperatureHumidity();

        buildSenML(); 

        if (uploadKPN()) {
            offsetAlarmTime(SLEEP_TIME_WHEN_OK); // Set to long sleeptime
        }
        else {
            offsetAlarmTime(SLEEP_TIME_WHEN_RETRY);
        }
    }
    setSystemStatus(SYSTEM_NONE);
    TRACE("<-");
}

void setup() {
    pinMode(LED_BLUE, OUTPUT);
    setSystemStatus(SYSTEM_INIT);
    sodaq_wdt_disable();
    initBOD33();
    sodaq_wdt_reset();

    #ifndef TRACEOFF
        CONSOLE_STREAM.begin(9600);
        while (!CONSOLE_STREAM && millis() < 10000);
        trace.setTraceStream(CONSOLE_STREAM);
    #endif
    TRACE("->");
    TRACE("Settingsfile: %s", SETTINGSFILE);
    Wire.begin();
    initSleep(); 
    initRtc(); 
    initGps(); 
    initDHT(dhtValues);
    initModem(); 
    initAccelerometer(); // 3,5mA

    device.add(status);
    device.add(temperature);
    device.add(humidity);
    device.add(latitude);
    device.add(longitude);
    device.add(battery1);
    //device.add(battery2);
    device.add(system_battery);

    event = idleEvent;
    lastTransmit = rtc.getEpoch();
    offsetAlarmTime(SLEEP_TIME_WHEN_OK); 
    setSystemStatus(SYSTEM_NONE); 
    event = timerEvent;

    TRACE("<-");
}

void loop() {

    // Start here after wake up
    if (event == motionEvent) {
        if (rtc.getEpoch() > (lastTransmit + (MIN_TIME_BETWEEN_TRANSMITS))) {
            sendMessage();
        }
        else {
            TRACE("MotionEvent to soon");
        } 
    } 
    if (event == timerEvent) {
        sendMessage();   
    }

    event = idleEvent;
    reportedStatus = STATUS_CLEARED;
    interrupts();
    
    // Go to sleep after X seconds
    if (rtc.getEpoch() > (lastTransmit + (TIME_TO_STAY_AWAKE))) {
        systemSleep(DEEPSLEEP);
    }
}

