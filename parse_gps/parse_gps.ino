uint8_t buffer[20] = "36.00901,-78.92581";
String latitude_str;
String longitude_str;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println((char*)buffer);
  String LoRa_message = String((char*)buffer);
  
  latitude_str = LoRa_message.substring(0, LoRa_message.indexOf(","));
  longitude_str = LoRa_message.substring(LoRa_message.indexOf(",")+1);
  char lat_c[10];
  char lon_c[10];
  latitude_str.toCharArray(lat_c, 10);
  longitude_str.toCharArray(lon_c, 10);
  double lat = atof(lat_c);
  double lon = atof(lon_c);

}

void loop() {
  

}
