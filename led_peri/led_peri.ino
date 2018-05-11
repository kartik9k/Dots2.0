#include <SPI.h>
#include <BLEPeripheral.h>

#define LED_PIN   3

BLEPeripheral                    blePeripheral                            = BLEPeripheral();
char arr[20] = {0};
BLEService               ledService           = BLEService("cc0");

BLECharCharacteristic    switchCharacteristic = BLECharCharacteristic("cc1", BLERead | BLEWrite | BLENotify);
BLEDescriptor            switchDesc           = BLEDescriptor("2901", "ABC");
void setup() {
  Serial.begin(9600);
#if defined (__AVR_ATmega32U4__)
  delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
#endif

  pinMode(LED_PIN, OUTPUT);

  blePeripheral.setLocalName("LED");
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchCharacteristic);
  blePeripheral.addAttribute(switchDesc);

  switchCharacteristic.setValue(5);
  // begin initialization
  blePeripheral.begin();

  Serial.println(F("BLE LED Peripheral"));
}

void loop() {
  BLECentral central = blePeripheral.central();

  if (central) {
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());

    while (central.connected()) {
      if (switchCharacteristic.written()) {
          Serial.println(switchCharacteristic.value(), HEX);
//        switchCharacteristic.setValue(110);
//        Serial.println("He has written");
        // central wrote new value to characteristic, update LED
        if (switchCharacteristic.value()) {
          Serial.println(F("LED on"));
          digitalWrite(LED_PIN, HIGH);
        } else {
          Serial.println(F("LED off"));
          digitalWrite(LED_PIN, LOW);
        }
      }
    }

    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
