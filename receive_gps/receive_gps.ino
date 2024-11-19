#include <LoRa.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <bluefruit.h>

// LoRa //

const int ss = 31;    // Pin CS (Chip Select)
const int rst = 27;  // Pin RST (Reset)
const int dio0 = 30;  // Pin IRQ (Interrupt Request)

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


// BLE

BLEDfu  bledfu;  // OTA DFU service
BLEDis  bledis;  // device information
BLEUart bleuart; // uart over ble
BLEBas  blebas;  // battery


char buffer[50];
double pi = 3.14159265;


void setup() {
  pinMode(A1, OUTPUT);
  
  Serial.begin(115200);
  // while (!Serial) delay(10);
  
  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    digitalWrite(A1, HIGH);
  }
  Serial.println("BNO08x Found!");

  setReports();
  Serial.println("Reading events for BNO085");

  Serial.println("LoRa Test");
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    digitalWrite(A1, HIGH);
  }
  Serial.println("LoRa OK");

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
    digitalWrite(A1, HIGH);
  } else {
    Serial.println("OLED Screen found!");
  }
  display.clearDisplay();


  Serial.println("Bluefruit52 BLEUART Setup");

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behavior, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  Serial.println("got through sensors");
  float bno085_real = sensorValue.un.rotationVector.real;
  float bno085_i = sensorValue.un.rotationVector.i;
  float bno085_j = sensorValue.un.rotationVector.j;
  float bno085_k = sensorValue.un.rotationVector.k;

  char bno085_data[50];
  sprintf(bno085_data, "r:%.2f, i:%.2f, j:%.2f, k:%.2f", bno085_real, bno085_i, bno085_j, bno085_k);
  // Serial.print("Quaternion from the BNO085 Rotation Vector: ");
  // Serial.println(bno085_data);

  float absolute_angle_rad = atan2(2*(bno085_i*bno085_j+bno085_real*bno085_k),(pow(bno085_real, 2)+pow(bno085_i, 2)-pow(bno085_j, 2)-pow(bno085_k,2)));
  // Serial.println(absolute_angle_rad);

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
  }
  // Serial.println(buffer);
  // try to parse packet
  LoRa.beginPacket();
  LoRa.print(buffer);
  LoRa.endPacket();
  char* buf;
  sprintf(buf, "");
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      buf = (char*)LoRa.read();
      Serial.println(buf);
    }
  }
  Serial.println("finished reading");

  double arr[2];
  parse_gps(buf, arr);
  Serial.println("finished parsing");
  double received_lat = arr[0];
  double received_lon = arr[1];

  float bearing = atan2(cos(my_lat)*sin(received_lat)-sin(my_lat)*cos(received_lat)*cos(received_lon-my_lon), sin(received_lon-my_lon)*cos(received_lat));
  float distance = getDistanceFromLatLonInKm(my_lat, my_lon, received_lat, received_lon);
  // Serial.println(bearing);
  bool distance_warning = distance > 0.05;

  float arrow_angle = (-1*bearing) - angle_offset + absolute_angle_rad;
  display.clearDisplay();
  if(distance_warning) {
    drawDistanceWarning();
  }
  drawArrow(arrow_angle);
  display.display();

  
  char buf1[20];
  sprintf(buf1, "1:%.5f", my_lat);
  int count = sizeof(buf1);
  bleuart.write((uint8_t*) buf1, count );
  char buf2[20];
  sprintf(buf2, "2:%.5f", my_lon);
  count = sizeof(buf2);
  bleuart.write((uint8_t*) buf2, count );
  char buf3[20];
  sprintf(buf3, "3:%.5f", received_lat);
  count = sizeof(buf3);
  bleuart.write((uint8_t*) buf3, count );
  char buf4[20];
  sprintf(buf4, "4:%.5f", received_lon);
  count = sizeof(buf4);
  bleuart.write((uint8_t*) buf4, count );
  char buf5[20];
  sprintf(buf5, "5:%.5f", distance);
  count = sizeof(buf5);
  bleuart.write((uint8_t*) buf5, count );

}

void parse_gps(char* buf, double* arr) {
  String LoRa_message = String(buf);
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

float getDistanceFromLatLonInKm(float lat1,float lon1,float lat2,float lon2) {
  float R = 6371; // Radius of the earth in km
  float dLat = deg2rad(lat2-lat1);  // deg2rad below
  float dLon = deg2rad(lon2-lon1); 
  float a = sin(dLat/2) * sin(dLat/2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon/2) * sin(dLon/2); 
  float c = 2 * atan2(sqrt(a), sqrt(1-a)); 
  float d = R * c; // Distance in km
  return d;
}

float deg2rad(float deg) {
  return deg * (pi/180);
}

void drawDistanceWarning() {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("! ! ! ! !");
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

  display.fillTriangle(64+point_a_x, 32-point_a_y, 64+point_b_x, 32-point_b_y, 64+point_c_x, 32-point_c_y, WHITE);
  display.fillTriangle(64+base_a_x, 32-base_a_y, 64+base_b_x, 32-base_b_y, 64+base_c_x, 32-base_c_y, WHITE);
  display.fillTriangle(64+base_d_x, 32-base_d_y, 64+base_b_x, 32-base_b_y, 64+base_c_x, 32-base_c_y, WHITE);
}

int rotate_x(float x, float y, float angle) {
  return (int) x*cos(angle)-y*sin(angle);
}
int rotate_y(float x, float y, float angle) {
  return (int) x*sin(angle)+y*cos(angle);
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);

  uint8_t buf[64] = "Init";
  int count = sizeof(buf);
  bleuart.write( buf, count );
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}