#include <Arduino.h>
/*
ESP32-CAM Remote Control 
*/
#include "esp_wifi.h"
#include "esp_camera.h"
#include <WiFi.h>
#include "analogWrite.h" // download library from https://github.com/ERROPiX/ESP32_AnalogWrite
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "App.hpp"
#include "camera_pins.h"
#include "tank_pins.h"

#ifdef DEBUG
#define DEBUGOUT(msg) Serial.print(msg)
#define DEBUGNUM(num, base) Serial.print(num, base)
#else
#define DEBUGOUT(msg)
#define DEBUGNUM(num, base)
#endif

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

#define INPINCNT sizeof(inPinNo)/sizeof(inPinNo[0])
uint8_t inPinNo[] = {MotInLPin, MotInRPin};
int16_t inPulseTime[INPINCNT]; //length of the throttle pulse [us], -1 = invalid
unsigned long inPulseStart[INPINCNT]; //begin of pulse micros()
uint8_t inPinVal[INPINCNT];

int16_t servoPulseMin=1000;
int16_t servoPulseMax=2000;
int16_t servoDeadZone=50;
int16_t servoTrim[INPINCNT]; //pulse time for neutral position

void startCameraServer();

void initMotors() 
{
  ledcSetup(MotPWM0, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcSetup(MotPWM1, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcSetup(MotPWM2, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcSetup(MotPWM3, 2000, 8); // 2000 hz PWM, 8-bit resolution
  ledcAttachPin(MotPin0, MotPWM0); 
  ledcAttachPin(MotPin1, MotPWM1); 
  ledcAttachPin(MotPin2, MotPWM2); 
  ledcAttachPin(MotPin3, MotPWM3); 
}

void initFlashLight() 
{
  ledcSetup(FlashPWM, 5000, 8);
  ledcAttachPin(FlashPin, FlashPWM);  //pin4 is LED
}

void initServo() 
{
  ledcSetup(ServoPWM, 50, 16); // 50 hz PWM, 16-bit resolution, range from 3250 to 6500.
  ledcAttachPin(ServoPin, ServoPWM); 
}

void initInputs()
{
	pinMode(MotInLPin, INPUT);
	pinMode(MotInRPin, INPUT);
}

void setup() 
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // prevent brownouts by silencing them
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  // Remote Control Car
  initMotors();
  initServo();
	initFlashLight();

	for (uint8_t pinIdx=0; pinIdx<INPINCNT; pinIdx++) {
		pinMode(inPinNo[pinIdx], INPUT);
		inPinVal[pinIdx] = digitalRead(inPinNo[pinIdx]);
		inPulseTime[pinIdx] = -1;
		inPulseStart[pinIdx] = 0;
		servoTrim[pinIdx] = (servoPulseMax + servoPulseMin)/2;
	}
  
  Serial.println("ssid: " + (String)ssid);
  
  WiFi.begin(ssid, password);
  delay(500);

  long int StartTime=millis();
  while (WiFi.status() != WL_CONNECTED) 
  {
      delay(500);
      if ((StartTime+10000) < millis()) break;
  } 

  /*
  int8_t power;
  esp_wifi_set_max_tx_power(20);
  esp_wifi_get_max_tx_power(&power);
  Serial.printf("wifi power: %d \n",power); 
  */
  
  startCameraServer();

  if (WiFi.status() == WL_CONNECTED) 
  {
    Serial.println("");
    Serial.println("WiFi connected");    
    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
  } else {
    Serial.println("");
    Serial.println("WiFi disconnected");      
    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.softAPIP());
    Serial.println("' to connect");
    const char* apssid = "ESP32-CAM";
    const char* appassword = "12345678";         //AP password require at least 8 characters.
    WiFi.softAP((WiFi.softAPIP().toString()+"_"+(String)apssid).c_str(), appassword);    
  }

  for (int i=0;i<5;i++) 
  {
    ledcWrite(FlashPWM,10);  // flash led
    delay(200);
    ledcWrite(FlashPWM,0);
    delay(200);    
  }       
}

unsigned long nowus = 0;

void loop() {
	nowus = micros();
	for (uint8_t pinIdx=0; pinIdx<INPINCNT; pinIdx++) {
		uint8_t newPinVal = digitalRead(inPinNo[pinIdx]);
		if (newPinVal == HIGH) {
			if (inPinVal[pinIdx] == LOW) {
				//Low to High transition
				inPulseStart[pinIdx] = nowus;
				DEBUGOUT("\n");
				DEBUGOUT(pinIdx);
				DEBUGOUT(" . ");
				DEBUGOUT(inPulseStart[pinIdx]);
			}
		} else {
			if (inPinVal[pinIdx] == HIGH) {
				//High to Low transition
				inPulseTime[pinIdx] = nowus - inPulseStart[pinIdx];
				DEBUGOUT("\n");
				DEBUGOUT(pinIdx);
				DEBUGOUT(" _ ");
				DEBUGOUT(nowus);
				DEBUGOUT(" ");
				DEBUGOUT(inPulseTime[pinIdx]);
				inPulseStart[pinIdx] = nowus;
				//DEBUGOUT("-");
				//DEBUGOUT(servoTrim[pinIdx]);
				//DEBUGOUT("=");
				//DEBUGOUT(engVal);
				if ((inPulseTime[pinIdx] > 2700) || (inPulseTime[pinIdx] < 300)) {
					inPulseTime[pinIdx] = -1;
				} else {
					int engInt = inPulseTime[pinIdx] - servoTrim[pinIdx];
					if (abs(engInt) > servoDeadZone) {
						engInt = engInt >> 2; // 0-1000 / 4 ~ 0-255
						//TODO: motor driver class
						int pwmChn = MotPWM0+(pinIdx<<1);
						if (engInt > 0) {
							pwmChn++;
						}
						DEBUGOUT("\n");
						DEBUGOUT(pinIdx);
						DEBUGOUT(" E ");
						DEBUGOUT(pwmChn);
						DEBUGOUT(" ");
						DEBUGOUT(engInt);
						ledcWrite(pwmChn, abs(engInt));
					}
				}
			} 
		}
		inPinVal[pinIdx] = newPinVal;
		if (inPulseStart[pinIdx] < nowus - 21000) {
			//now pulse in last 20ms period
			inPulseTime[pinIdx] = -1;
		}
	}
}
