#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Begin program: OLED Display arrows");

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  Serial.println("Drawing Arrow N");
  drawArrow_N();
  Serial.println("Done drawing");
  delay(2000);
  Serial.println("Drawing Arrow NE");
  drawArrow_NE();
  Serial.println("Done drawing");
  delay(2000);
  Serial.println("Drawing Arrow E");
  drawArrow_E();
  Serial.println("Done drawing");
  delay(2000);
  Serial.println("Drawing Arrow SE");
  drawArrow_SE();
  Serial.println("Done drawing");
  delay(2000);
  Serial.println("Drawing Arrow S");
  drawArrow_S();
  Serial.println("Done drawing");
  delay(2000);
  Serial.println("Drawing Arrow SW");
  drawArrow_SW();
  Serial.println("Done drawing");
  delay(2000);
  Serial.println("Drawing Arrow W");
  drawArrow_W();
  Serial.println("Done drawing");
  delay(2000);
  Serial.println("Drawing Arrow NW");
  drawArrow_NW();
  Serial.println("Done drawing");
  delay(2000);

}

void loop() {

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