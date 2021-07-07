#include <MCP492X.h> // Include the library

#define PIN_SPI_CHIP_SELECT_DAC 10 // Or any pin you'd like to use

MCP492X myDac(PIN_SPI_CHIP_SELECT_DAC);

void setup() {
  // put your setup code here, to run once:
  myDac.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

  // Write any value from 0-4095
  myDac.analogWrite(2048);
  delay(1000);
  myDac.analogWrite(4095);
  delay(1000);
  myDac.analogWrite(0);
  delay(1000);
  // If using an MCP4922, write a value to DAC B
  myDac.analogWrite(1, 3172);
}
