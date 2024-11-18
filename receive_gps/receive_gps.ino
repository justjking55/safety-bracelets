#include <RH_RF95.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// LoRa //

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4
#endif

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


// GPS //
/*
//Hardware Serial
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
*/
// I2C
Adafruit_GPS GPS(&Wire);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();


// BNO085 //

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


// OLED Screen

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char buffer[50];
double pi = 3.14159265;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  // while (!Serial) delay(10);
  
  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();
  Serial.println("Reading events for BNO085");

  Serial.println("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial.println("Adafruit GPS library basic parsing test!");
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  // GPS.begin(9600);
  GPS.begin(0x10);  // The I2C address to use is 0x10
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPS.println(PMTK_Q_RELEASE);

  Serial.println("ELEGOO OLED Test!");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  } else {
    Serial.println("OLED Screen found!");
  }
  display.clearDisplay();

}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  float bno085_real = sensorValue.un.rotationVector.real;
  float bno085_i = sensorValue.un.rotationVector.i;
  float bno085_j = sensorValue.un.rotationVector.j;
  float bno085_k = sensorValue.un.rotationVector.k;

  char bno085_data[50];
  sprintf(bno085_data, "r:%.2f, i:%.2f, j:%.2f, k:%.2f", bno085_real, bno085_i, bno085_j, bno085_k);
  // Serial.print("Quaternion from the BNO085 Rotation Vector: ");
  // Serial.println(bno085_data);

  float absolute_angle_rad = atan2(2*(bno085_i*bno085_j+bno085_real*bno085_k),(pow(bno085_real, 2)+pow(bno085_i, 2)-pow(bno085_j, 2)-pow(bno085_k,2)));
  Serial.println(absolute_angle_rad);

  float angle_offset = 0.9; // estimated from building angle--this is the value read when pointing at N

  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) {
      Serial.print("c: "); 
      Serial.print(c);
    }
  // if a sentence is received, we can check the checksum, parse it...
  
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print("GPS.lastNMEA: ");
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
      Serial.println("NMEA sentence parse failed");
      return; // we can fail to parse a sentence in which case we should just wait for another
    }
  }
  
  double my_lat;
  double my_lon;
  if (GPS.fix) {
    // Serial.println("GPS satellite found!");
    my_lat = GPS.latitudeDegrees;
    my_lon = GPS.longitudeDegrees;

    sprintf(buffer, "%.5f,%.5f", my_lat, my_lon);
  } /* else {
    // Serial.println("GPS satellite not found!");
    sprintf(buffer, "GPS satellite not found!");
  } */
  // Serial.println(buffer);

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (rf95.available()) {
    // Should be a message for us now

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      // uint8_t data[] = "And hello back to you";
      rf95.send((uint8_t *) buffer, sizeof(buffer));
      rf95.waitPacketSent();
      Serial.println("Sent a reply");
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Receive failed");
    }
  }

  double arr[2];
  parse_gps(buf, arr);
  double received_lat = arr[0];
  double received_lon = arr[1];

  float bearing = atan2(cos(my_lat)*sin(received_lat)-sin(my_lat)*cos(received_lat)*cos(received_lon-my_lon), sin(received_lon-my_lon)*cos(received_lat));
  // Serial.println(bearing);

  float arrow_angle = (-1*bearing) - angle_offset + absolute_angle_rad;
  drawArrow(arrow_angle);

}

void parse_gps(uint8_t* buf, double* arr) {
  String LoRa_message = String((char*)buf);
  String latitude_str;
  String longitude_str;
  
  latitude_str = LoRa_message.substring(0, LoRa_message.indexOf(","));
  longitude_str = LoRa_message.substring(LoRa_message.indexOf(",")+1);
  char lat_c[10];
  char lon_c[10];
  latitude_str.toCharArray(lat_c, 10);
  longitude_str.toCharArray(lon_c, 10);
  double lat = atof(lat_c);
  double lon = atof(lon_c);

  arr[0] = lat;
  arr[1] = lon;

}

void drawArrow(float angle) {
  int point_a_x = rotate_x(0, 16, angle);
  int point_a_y = rotate_y(0, 16, angle);
  int point_b_x = rotate_x(-8, 0, angle);
  int point_b_y = rotate_y(-8, 0, angle);
  int point_c_x = rotate_x(8, 0, angle);
  int point_c_y = rotate_y(8, 0, angle);
  int base_a_x = rotate_x(-3, 0, angle);
  int base_a_y = rotate_y(-3, 0, angle);
  int base_b_x = rotate_x(3, 0, angle);
  int base_b_y = rotate_y(3, 0, angle);
  int base_c_x = rotate_x(-3, -12, angle);
  int base_c_y = rotate_y(-3, -12, angle);
  int base_d_x = rotate_x(3, -12, angle);
  int base_d_y =  rotate_y(3, -12, angle);

  display.clearDisplay();
  display.fillTriangle(64+point_a_x, 32-point_a_y, 64+point_b_x, 32-point_b_y, 64+point_c_x, 32-point_c_y, WHITE);
  display.fillTriangle(64+base_a_x, 32-base_a_y, 64+base_b_x, 32-base_b_y, 64+base_c_x, 32-base_c_y, WHITE);
  display.fillTriangle(64+base_d_x, 32-base_d_y, 64+base_b_x, 32-base_b_y, 64+base_c_x, 32-base_c_y, WHITE);
  display.display();
}

int rotate_x(float x, float y, float angle) {
  return (int) x*cos(angle)-y*sin(angle);
}
int rotate_y(float x, float y, float angle) {
  return (int) x*sin(angle)+y*cos(angle);
}
