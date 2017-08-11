#if !defined(PARTICLE)
 #include <Wire.h>
#endif
#include <Adafruit_BMP085.h>

#include "Adafruit_DHT.h"
// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5, Particle on D1
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4, Particle on D0
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;

#define DHTPIN 3    // what pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11		// DHT 11
/*#define DHTTYPE DHT22		// DHT 22 (AM2302)*/
//#define DHTTYPE DHT21		// DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTPIN, DHTTYPE);

//power cam bien anh sang (analog)
/*int power_light_intensity = A5;*/
int data_analog_light_intensity = A0;

//power cam bien anh sang (digital)
int data_digital_light_intensity = A1;

//led bao gui publish
int led_show = D7;
int relay = D6;

int time_delay = 20000;

int lowTemp = 20;
int heightTemp = 40;

//data

int pa; // ap suat
int height; // do cao
int light_analog; // cuong do anh sang analog
int light_digital; // cuong do anh sang digital
int h; //do am
int t; // do C
int f; // do F
int k; // do K
int hi; // nhiet do cao
int dp; // diem suong
bool isLed = true;
String environment = "";

String notificationTemp = "";

//variable
String enviCurrent = "";



void setup() {
  Serial.begin(9600);
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }

	/*pinMode(power_light_intensity, OUTPUT);*/
  pinMode(data_analog_light_intensity, INPUT);
	pinMode(data_digital_light_intensity, INPUT);
	pinMode(led_show, OUTPUT);
  pinMode(relay, OUTPUT);

	/*digitalWrite(power_light_intensity, HIGH);*/

  digitalWrite(led_show, HIGH);

	dht.begin();

  Particle.variable("enviCurrent", enviCurrent);
  Particle.variable("isLed", isLed);
  Particle.variable("timeDelay", time_delay);
  Particle.variable("lowTemp", lowTemp);
  Particle.variable("heightTemp", heightTemp);

  Particle.function("controllLed", controllLed);
  Particle.function("setCurrent", setCurrent);
  Particle.function("setTimeDelay", setTimeDelay);
  Particle.function("setlTemp", setLowTemp);
  Particle.function("sethTemp", setHeightTemp);

}

void loop() {


    /*Serial.print("ap suat = ");*/
    pa = bmp.readPressure();
    Serial.print(pa);
    /*Serial.println(" Pa");*/

    /*Serial.print("Do cao thuc te = ");*/
    height = bmp.readAltitude(101500);
    /*Serial.print(height);*/
    /*Serial.println(" meters");*/

    /*Serial.println();*/

	  /*Serial.print("cuong do anh sang (analog)= ");*/
		light_analog = analogRead(data_analog_light_intensity);
	  /*Serial.println(light_analog);*/

    /*Serial.print("cuong do anh sang (digital)= ");*/
		light_digital = digitalRead(data_digital_light_intensity);
	  /*Serial.println(light_digital);*/

		// Reading temperature or humidity takes about 250 milliseconds!
		// Sensor readings may also be up to 2 seconds 'old' (its a
		// very slow sensor)
		h = dht.getHumidity();
		// Read temperature as Celsius
		t = dht.getTempCelcius();
		// Read temperature as Farenheit
		f = dht.getTempFarenheit();

		// Check if any reads failed and exit early (to try again).
		if (isnan(h) || isnan(t) || isnan(f)) {
			Serial.println("Failed to read from DHT sensor!");
			delay(time_delay);
			return;
		}

		// Compute heat index
		// Must send in temp in Fahrenheit!
		hi = dht.getHeatIndex();
		dp = dht.getDewPoint();
		k = dht.getTempKelvin();

		/*Serial.print("Độ ẩm: ");
		Serial.print(h);
		Serial.print("% - ");
		Serial.print("Nhiệt độ: ");
		Serial.print(t);
		Serial.print("*C ");
		Serial.print(f);
		Serial.print("*F ");
		Serial.print(k);
		Serial.print("*K - ");
		Serial.print("diem suong: ");
		Serial.print(dp);
		Serial.print("*C - ");
		Serial.print("Nhiet do cao : ");
    Serial.print(hi);
		Serial.println("*C");
		Serial.println(Time.timeStr());

    Serial.print("time delay ");
    Serial.println(time_delay);
		Serial.println("-------------------------------");
		Serial.println();*/

    environment = "{";
    environment += "\"pa\" : ";
    environment += pa;
    environment += ", \"height\" : ";
    environment += height;
    environment += ", \"la\" : ";
    environment += light_analog;
    environment += ", \"ld\" : ";
    environment += light_digital;
    environment += ", \"h\" : ";
    environment += h;
    environment += ", \"t\" : ";
    environment += t;
    environment += ", \"f\" : ";
    environment += f;
    environment += ", \"k\" : ";
    environment += k;
    environment += ", \"hi\" : ";
    environment += hi;
    environment += ", \"dp\" : ";
    environment += dp;
    environment += "}";
    Particle.publish("Environment", environment);

    if(t > heightTemp){
      digitalWrite(relay,HIGH);
    }else{
      digitalWrite(relay,LOW);
    }

    if(t < lowTemp || t > heightTemp){
      notificationTemp = "{";
      notificationTemp += "\"channel\" : ";
      notificationTemp += "\"temp\"";
      notificationTemp += ", \"content\" : ";
      notificationTemp += " \"tempC : ";
      notificationTemp += t;
      notificationTemp += " - tempF : ";
      notificationTemp += f;
      notificationTemp += " \"";
      notificationTemp += "}";

      Particle.publish("NotificationTemp", notificationTemp);
    }
    delay(time_delay);
}


//controll led
int controllLed(String command) {

    if (command=="on") {
        digitalWrite(led_show,HIGH);
        isLed = true;
        return 1;
    }
    else if (command=="off") {
        digitalWrite(led_show,LOW);
        isLed = false;
        return 0;
    }
    else {
        return -1;
    }

}

//lay cac thong so hien tai
int setCurrent(String command){
  // set enviCurrent
  enviCurrent = "{";
  enviCurrent += "\"pa\" : ";
  enviCurrent += pa;
  enviCurrent += ", \"height\" : ";
  enviCurrent += height;
  enviCurrent += ", \"la\" : ";
  enviCurrent += light_analog;
  enviCurrent += ", \"ld\" : ";
  enviCurrent += light_digital;
  enviCurrent += ", \"h\" : ";
  enviCurrent += h;
  enviCurrent += ", \"t\" : ";
  enviCurrent += t;
  enviCurrent += ", \"f\" : ";
  enviCurrent += f;
  enviCurrent += ", \"k\" : ";
  enviCurrent += k;
  enviCurrent += ", \"hi\" : ";
  enviCurrent += hi;
  enviCurrent += ", \"dp\" : ";
  enviCurrent += dp;
  enviCurrent += "}";
  return 1;
}

// thoi gian phan hoi
int setTimeDelay(String new_time_delay){
  time_delay = new_time_delay.toInt();
  return 1;
}

// set low temp
int setLowTemp(String lowTempNew){
  lowTemp = lowTempNew.toInt();
  return 1;
}

// set hight temp
int setHeightTemp(String heightTempNew){
  heightTemp = heightTempNew.toInt();
  return 1;
}
