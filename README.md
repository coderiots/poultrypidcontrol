#include <Arduino.h>

// dht sensor
#include <DHTesp.h>
DHTesp dht;
#define dht_pin 32

// mq sensor
#include <MQUnifiedsensor.h>
#define vres 3.3
#define mq_pin 35					//pin yg digunakan oleh MQ-135
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6
MQUnifiedsensor MQ135("ESP-32", vres, ADC_Bit_Resolution, mq_pin, "MQ-135");

// save data like eeprom
#include <Preferences.h>
Preferences preferences;

// wifi
#include <ThingSpeak.h>
#include <WiFi.h>

String apikey = "70AOQN27GDRSHQK6";
char ssid[] = "Silas";
char pass[] = "silas123";
char server[] = "api.thingspeak.com";
WiFiClient espClient;
unsigned long myChannelNumber = 2207195;
const char * myWriteAPIKey = "70AOQN27GDRSHQK6";

// esp32 ntp
#include <WiFiUdp.h>
#include <NTPClient.h>				//library ntp
#iniclude <RTClib.h>				//library RTC						
#define WIB 25200					//konfigurasi waktu yang digunakan(WIB)
WiFiUDP espudp;
NTPClient moment(espudp);			//menginisialisasi ntp client sebagai momen
int day_temp;						//inisialisasi pembacaan temperature berdasarkan hari
int days;							//inisialisasi hari
#define RTC 14						//pin rtc yang digunakan

// dimmer
#include <RBDdimmer.h>					//libraray dimmer
#define heat_pin 4						//pin yang digunakan oleh dimmer 	
#define zerocross 5						//pin yang digunakan oleh zerocross
dimmerLamp heat(heat_pin, zerocross);

// pid
#include "PID.h"						//library pid 
PIDController pid;						//medefinisikan pembacaan pid sebagai PIDcontroller
float kp = 5; // sensor error reading	//tuning pid
float ki = 2; // output power
float kd = 4;

// exhaust
#define exhaust_pin 18		//pin exshaust yang digunakan

//fan
#define fan_pin 26			//pin fan yang digunakan

// wdt
#include <esp_task_wdt.h>

// lcd
#include <LiquidCrystal_I2C.h>				//library LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);			//konfigurasi LCD yang digunakan(0x27, 20 x 4)

void set_fan(float suhu) 					//void perintah untuk fan
{
    if (!isnan(suhu)) {
        if (suhu < pid.getpoint()) digitalWrite(fan_pin, 0);
        else digitalWrite(fan_pin, 1);
    }
}

void set_heat(float suhu) {					//void pengaturan suhu
    if(!isnan(suhu)) { 
        int pid_output = pid.compute(int(suhu));
        Serial.println("temp: " + String(suhu));
        Serial.println("pid output: " + String(pid_output));
        heat.setPower(pid_output);
    }
    else Serial.print("nan");
}

void set_exhaust(float NH4, float suhu) {		//void perintah untuk exshaust
    if(NH4 >= 10.0 && suhu > pid.getpoint()) digitalWrite(exhaust_pin, 0);
    //else if(suhu < pid.getpoint()) digitalWrite(exhaust_pin, 0);    

    else digitalWrite(exhaust_pin, 1);
}

void set_lcd(float temp, float hum, float am) {		//void perintah tampilan LCD
    lcd.setCursor(0, 0);
    lcd.print("t: " + String(temp) + "C | h: " + String(hum) + "%");

    lcd.setCursor(0, 1);
    lcd.print("NH4: " + String(am) + "ppm");
    
    lcd.setCursor(0, 2);
    lcd.print(" kp     ki     kd");
    
    lcd.setCursor(0, 3);
    lcd.print(String(kp) + " " + String(ki) + " " + String(kd));
}

void mq_calibrate() {			//void perintah mq 135
    Serial.print("Calibrating MQ135 please wait.");
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
        MQ135.update();
        calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
        Serial.print(".");
    }
    MQ135.setR0(calcR0/10);
    Serial.println("  done!");
}

void check_day() {				// void perintah penyesuaian suhu dengan waktu 
    int day = moment.getDay();
    if(day != day_temp) {
        day_temp = day;
        days++;
    }

    if(days >= 0 && days < 2) pid.setpoint(32);
    else if(days >= 3 && days < 4) pid.setpoint(31);
    else if(days >= 5 && days < 7) pid.setpoint(30);
     else if(days >= 8 && days <10) pid.setpoint(29);
}

void startTaskOnCore(TaskFunction_t task, const char taskname[], BaseType_t cpu_number) {
    xTaskCreatePinnedToCore(
        task,
        taskname,
        configMINIMAL_STACK_SIZE * 6, NULL, 2 | portPRIVILEGE_BIT,
        NULL,
        cpu_number
    );
}

void first_core(void *args) {				//void sistem pid, suhu, plx dan ThingSpeak
    heat.begin(NORMAL_MODE, ON);
    heat.setPower(0);

    lcd.init();
    lcd.backlight();

    MQ135.setRegressionMethod(1);			//kalibrasi mq 135
    MQ135.setA(102.2);
    MQ135.setB(-2.473);
    MQ135.init(); 
    mq_calibrate();

    pid.begin();
    pid.setpoint(32);
    pid.tune(kp, ki, kd);
    pid.limit(0, 100);

    // pinMode(exhaust_pin, OUTPUT);
    // digitalWrite(exhaust_pin, 0);

    dht.setup(dht_pin, DHTesp::DHT22);
    pinMode(fan_pin,OUTPUT);
    digitalWrite(fan_pin, 0);

    WiFi.begin(ssid, pass);
    ThingSpeak.begin(espClient);
    
    moment.begin();							//membaca pembacaan momen
    moment.setTimeOffset(WIB);				//konfigurasi waktu yang digunakan 
RTC.begin();								//membaca RTC

    Serial.println("CLEARDATA");					//mengirim data ke PLX Daq
    Serial.println("LABEL,Temperature,Humidity");	//konfigurasi untuk menampilkan data kedalam tabel PLX Daq

    while(1) {
        TempAndHumidity dht_data = dht.getTempAndHumidity();
        MQ135.update();							//pembacaan sensor mq135 terakhir
        moment.update();						//pembacaan moment terakhir 
        check_day();							//mengecek data hari yang tersimpan 

        float temperature = dht_data.temperature;
        float humidity = dht_data.humidity;
        float NH4 = MQ135.readSensor();

ThingSpeak.writeField(myChannelNumber, 1, temperature, myWriteAPIKey);	//mengirim data pembacaan sensor  ke channel server ThingSpeak dalam field 1
ThingSpeak.writeField(myChannelNumber, 2, humidity, myWriteAPIKey);	//mengirim data pembacaan sensor ke channel server ThingSpeak dalam field 2
ThingSpeak.writeField(myChannelNumber, 3, NH4 , myWriteAPIKey);	//mengirim data pembacaan sensor ke channel server ThingSpeak dalam field 1

        set_lcd(temperature, humidity, NH4);
        set_fan(temperature);
        set_heat(temperature);
        set_exhaust(NH4, temperature);
        Serial.print("DATA,TIME,");	//menampilkan data pembacaan sensor ke dalam tabel PLX Daq
         Serial.print(temperature);
         Serial.print(",");
         Serial.print(humidity);
         Serial.print(",");
         Serial.print(NH4);
         Serial.println("");
        vTaskDelay(1000);
    }
}

void setup() {
    Serial.begin(9600);
    
    startTaskOnCore(&first_core, "main_task", PRO_CPU_NUM);

    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, pass);
    pinMode(fan_pin,OUTPUT);      
    pinMode(exhaust_pin,OUTPUT);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    Serial.println(" Connected!");

    // https://randomnerdtutorials.com/esp32-save-data-permanently-preferences/
    preferences.begin("chicken", false); 

    // watchdog 30 minutes
    esp_task_wdt_init(1800, true);
}

// let being void //
void loop() {}
