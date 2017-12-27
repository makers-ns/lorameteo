#include <Adafruit_BME280.h>
#include "flip_click_defs.h"
#include <rn2xx3.h>


/******************************************************************************
 * Module Preprocessor Macros
 *******************************************************************************/
#define get_temp()               weather_data.temp
#define get_humidity()           weather_data.humidity
#define get_pressure()           weather_data.pressure

typedef struct
{
    float temp;
    float humidity;
    float pressure;
} weather_data_t;

/*
union {
    float float_variable;
    byte temp_array[4];
} u;
*/
union {
    int32_t float_variable;
    byte temp_array[4];
} u;


static weather_data_t weather_data;
uint8_t txBuffer[12];
//create an instance of the rn2483 library, using the given Serial port
rn2xx3 myLora(Serial1);
Adafruit_BME280 bme; 

void bme_init( void );
void bme_update( void );

// the setup routine runs once when you press reset:
void setup()
{
  //output LED pin
  pinMode(13, OUTPUT);
  led_on();

  // Open serial communications and wait for port to open:
  Serial.begin(57600); //serial port to computer
  Serial1.begin(57600); //serial port to radio

  while ((!Serial) && (millis() < 10000));

  Serial.println("Startup");

  initialize_radio();
  bme_init(); 
  
  //transmit a startup message
  //myLora.tx("TTN Mapper on TTN Uno node");

  led_off();
  delay(2000);
}

// the loop routine runs over and over again forever:
void loop()
{
    led_on();
    Serial.println("Meassuring");
    bme_update();
    Serial.println("TXing");
    for( int ii = 0; ii < 12; ii++){
      txBuffer[ii] = ii;
    }
    
    u.float_variable = get_temp()*100;
    txBuffer[0] = u.temp_array[0];
    txBuffer[1] = u.temp_array[1];
    txBuffer[2] = u.temp_array[2];
    txBuffer[3] = u.temp_array[3];

    u.float_variable = get_humidity()*100;
    txBuffer[4] = u.temp_array[0];
    txBuffer[5] = u.temp_array[1];
    txBuffer[6] = u.temp_array[2];
    txBuffer[7] = u.temp_array[3];

    u.float_variable = get_pressure()*100;
    txBuffer[8] = u.temp_array[0];
    txBuffer[9] = u.temp_array[1];
    txBuffer[10] = u.temp_array[2];
    txBuffer[11] = u.temp_array[3];
    
    myLora.txBytes(txBuffer,12); 

    led_off();
    delay(60*1000);
}



void led_on()
{
  digitalWrite(13, 1);
}

void led_off()
{
  digitalWrite(13, 0);
}

void bme_init()
{
  if( !bme.begin( 0x76 ) ) 
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

void bme_update()
{
    weather_data.temp = bme.readTemperature();
    Serial.println(weather_data.temp);
    weather_data.pressure = bme.readPressure() / 100.0f;
    weather_data.humidity = bme.readHumidity();
}

void initialize_radio()
{
  delay(100); //wait for the RN2xx3's startup message
  Serial1.flush();

  //print out the HWEUI so that we can register it via ttnctl
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    Serial.println("Communication with RN2xx3 unsuccessful. Power cycle the TTN UNO board.");
    delay(10000);
    hweui = myLora.hweui();
  }
  Serial.println("When using OTAA, register this DevEUI: ");
  Serial.println(hweui);
  Serial.println("RN2xx3 firmware version:");
  Serial.println(myLora.sysver());

  //configure your keys and join the network
  Serial.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  join_result = myLora.initABP(REPLACE_ME, REPLACE_ME, REPLACE_ME);

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA(REPLACE_ME, REPLACE_ME);

  while(!join_result)
  {
    Serial.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  Serial.println("Successfully joined TTN");

}

