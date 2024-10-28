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
  while (!Serial) delay(10);
  
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

  /*
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

  */

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

  /*
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
  */
  // Seeded values: 36.003773835464806, -78.94115364710927
  double my_lat = 36.003773835464806;
  double my_lon = -78.94115364710927;

  /*
  if (GPS.fix) {
    // Serial.println("GPS satellite found!");
    my_lat = GPS.latitudeDegrees;
    my_lon = GPS.longitudeDegrees;

    sprintf(buffer, "%.5f,%.5f", my_lat, my_lon);
  } *//* else {
    // Serial.println("GPS satellite not found!");
    sprintf(buffer, "GPS satellite not found!");
  } */
  // Serial.println(buffer);
  /*
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
  */
  /*

  double arr[2];
  parse_gps(buf, arr);
  */

  // Seeded Values: 36.00389426198051, -78.94083580536318
  double received_lat = 36.00389426198051;
  double received_lon = -78.94083580536318;

  float bearing = atan2(cos(my_lat)*sin(received_lat)-sin(my_lat)*cos(received_lat)*cos(received_lon-my_lon), sin(received_lon-my_lon)*cos(received_lat));
  // Serial.println(bearing);

  float arrow_angle =  absolute_angle_rad - angle_offset - bearing;
  Serial.println(arrow_angle);
  Serial.println((pi/8));
  char dir[3];
  char prev_dir[3];
  float start = (-1 * 2 * pi) - (pi/8);
  if (arrow_angle <= start) 
    sprintf(dir, "NW");
  else if (arrow_angle <= start + (1*(pi/4)))
    sprintf(dir, "N-");
  else if (arrow_angle <= start + (2*(pi/4)))
    sprintf(dir, "NE");
  else if (arrow_angle <= start + (3*(pi/4)))
    sprintf(dir, "E-");
  else if (arrow_angle <= start + (4*(pi/4)))
    sprintf(dir, "SE");
  else if (arrow_angle <= start + (5*(pi/4)))
    sprintf(dir, "S-");
  else if (arrow_angle <= start + (6*(pi/4)))
    sprintf(dir, "SW");
  else if (arrow_angle <= start + (7*(pi/4)))
    sprintf(dir, "W-");
  else if (arrow_angle <= start + (8*(pi/4)))
    sprintf(dir, "NW");
  else if (arrow_angle <= start + (9*(pi/4)))
    sprintf(dir, "N-");
  else if (arrow_angle <= start + (10*(pi/4)))
    sprintf(dir, "NE");
  else if (arrow_angle <= start + (11*(pi/4)))
    sprintf(dir, "E-");
  else if (arrow_angle <= start + (12*(pi/4)))
    sprintf(dir, "SE");
  else if (arrow_angle <= start + (13*(pi/4)))
    sprintf(dir, "S-");
  else if (arrow_angle <= start + (14*(pi/4)))
    sprintf(dir, "SW");
  else if (arrow_angle <= start + (15*(pi/4)))
    sprintf(dir, "W-");
  else if (arrow_angle <= start + (16*(pi/4)))
    sprintf(dir, "NW");

  Serial.println(dir);
  // Serial.println(prev_dir);
  
  //if (prev_dir != dir) {
    sprintf(prev_dir, dir);
    if (prev_dir[0] == 'N' && prev_dir[1] == '-')
      drawArrow_N();
    else if (prev_dir[0] == 'N' && prev_dir[1] == 'E')
      drawArrow_NE();
    else if (prev_dir[0] == 'E' && prev_dir[1] == '-')
      drawArrow_E();
    else if (prev_dir[0] == 'S' && prev_dir[1] == 'E')
      drawArrow_SE();
    else if (prev_dir[0] == 'S' && prev_dir[1] == '-')
      drawArrow_S();
    else if (prev_dir[0] == 'S' && prev_dir[1] == 'W')
      drawArrow_SW();
    else if (prev_dir[0] == 'W' && prev_dir[1] == '-')
      drawArrow_W();
    else if (prev_dir[0] == 'N' && prev_dir[1] == 'W')
      drawArrow_NW();
  //}
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

void drawArrow_N() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    display.width()/2  , display.height()/2-16,
    display.width()/2-8, display.height()/2,
    display.width()/2+8, display.height()/2, WHITE);
  // fillRect(x, y, width, height, color)
  display.fillRect(display.width()/2-3, display.height()/2, 7, 12, WHITE);
  display.display();

}

void drawArrow_NE() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    58, 26,
    70, 38,
    76, 20, WHITE);
  display.fillTriangle(
    62, 30,
    66, 34,
    53, 39, WHITE);
  display.fillTriangle(
    57, 43,
    53, 39,
    66, 34, WHITE);
  display.display();

}

void drawArrow_E() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    display.width()/2+16, display.height()/2,
    display.width()/2, display.height()/2+8,
    display.width()/2, display.height()/2-8, WHITE);
  // fillRect(x, y, width, height, color)
  display.fillRect(display.width()/2-12, display.height()/2-3, 12, 7, WHITE);
  display.display();

}

void drawArrow_SE() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    58, 38,
    70, 26,
    76, 44, WHITE);
  display.fillTriangle(
    53, 25,
    57, 21,
    62, 34, WHITE);
  display.fillTriangle(
    62, 34,
    66, 30,
    57, 21, WHITE);
  display.display();

}

void drawArrow_S() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    display.width()/2  , display.height()/2+16,
    display.width()/2-8, display.height()/2,
    display.width()/2+8, display.height()/2, WHITE);
  // fillRect(x, y, width, height, color)
  display.fillRect(display.width()/2-3, display.height()/2-12, 7, 12, WHITE);
  display.display();

}

void drawArrow_SW() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    58, 26,
    70, 38,
    52, 44, WHITE);
  display.fillTriangle(
    71, 21,
    75, 25,
    66, 34, WHITE);
  display.fillTriangle(
    62, 30,
    66, 34,
    71, 21, WHITE);
  display.display();

}

void drawArrow_W() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    display.width()/2-16, display.height()/2,
    display.width()/2, display.height()/2+8,
    display.width()/2, display.height()/2-8, WHITE);
  // fillRect(x, y, width, height, color)
  display.fillRect(display.width()/2, display.height()/2-3, 12, 7, WHITE);
  display.display();

}

void drawArrow_NW() {
  display.clearDisplay();
  // fillTriangle(x1, y1, x2, y2, x3, y3, color)
  display.fillTriangle(
    52, 20,
    58, 38,
    70, 26, WHITE);
  display.fillTriangle(
    62, 34,
    66, 30,
    71, 43, WHITE);
  display.fillTriangle(
    71, 43,
    75, 39,
    66, 30, WHITE);
  display.display();

}