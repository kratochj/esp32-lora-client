/*
  LoRa Client
  Remote Node for LoRa
  Requires LoRa Library by Sandeep Mistry - https://github.com/sandeepmistry/arduino-LoRa
*/

// Include required libraries
#include <SPI.h>
#include <LoRa.h>
#include <Logger.h>

// Define the pins used by the LoRa module
const int csPin = 4;     // LoRa radio chip select
const int resetPin = 2;  // LoRa radio reset
const int irqPin = 3;    // Must be a hardware interrupt pin

const int ledPin = 13; // Pin for LED

// Start LoRa module at local frequency
// 433E6 for Asia
// 433E6 for Europe
// 866E6 for Europe
// 915E6 for North America
const long loraFrequency = 433E6;

const Logger::Level logLevel = Logger::VERBOSE;

// Outgoing message variable
String outMessage;

// Controller data variable
String inMessage;

// Previous value Controller data variable
String inMessageOld;

// Outgoing Message counter
byte msgCount = 0;

// Source and destination addresses
byte localAddress = 0xAA;  // address of this device (must be unique, 0xAA or 0xBB)
byte destination = 0x01;   // destination to send to (controller = 0x01)

// Receive Callback Function
void onReceive(int packetSize) {
    if (packetSize == 0) return;  // if there's no packet, return

    // Read packet header bytes:
    int recipient = LoRa.read();        // recipient address
    byte sender = LoRa.read();          // sender address
    byte incomingMsgId = LoRa.read();   // incoming msg ID
    byte incomingLength = LoRa.read();  // incoming msg length

    String incoming = "";  // payload of packet

    while (LoRa.available()) {        // can't use readString() in callback, so
        incoming += (char) LoRa.read();  // add bytes one by one
    }

    if (incomingLength != incoming.length()) {  // check length for error
        Serial.println("error: message length does not match length");
        return;  // skip rest of function
    }

    // If the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0xFF) {
        Serial.println("This message is not for me.");
        return;  // skip rest of function
    }

    // If we are this far then this message is for us
    // Update the controller data variable
    inMessage = incoming;


    Logger::log(logLevel, "Received message:");
    Logger::log(logLevel, "  Sender:" + String(sender));
    Logger::log(logLevel, "  Recipient:" + String(recipient));
    Logger::log(logLevel, "  Message ID:" + String(incomingMsgId));
    Logger::log(logLevel, "  Message:" + String(incoming));
}

// Send LoRa Packet
void sendMessage(const String& outgoing) {
    LoRa.beginPacket();             // start packet
    LoRa.write(destination);        // add destination address
    LoRa.write(localAddress);       // add sender address
    LoRa.write(msgCount);           // add message ID
    LoRa.write(outgoing.length());  // add payload length
    LoRa.print(outgoing);           // add payload
    LoRa.endPacket();               // finish packet and send it
    msgCount++;                     // increment message ID

    Logger::log(logLevel, "Sent message:");
    Logger::log(logLevel, "  Target:" + String(destination));
    Logger::log(logLevel, "  Sender:" + String(localAddress));
    Logger::log(logLevel, "  Message ID:" + String(msgCount));
    Logger::log(logLevel, "  Message:" + String(outgoing));
}

unsigned long convertStrToLong(const String& time) {
    unsigned long mili;
    char Tim[9] = "";
    uint16_t timsize = time.length() + 1;
    char TIM[timsize];
    time.toCharArray(TIM, timsize);
    return strtol(time.c_str(), nullptr, 10);
}

void setup() {

    Serial.begin(9600);
    while (!Serial);

    // Set LED as output (if used)
    pinMode(ledPin, OUTPUT);

    // Setup LoRa module
    LoRa.setPins(csPin, resetPin, irqPin);

    // Start LoRa module at local frequency
    // 433E6 for Asia
    // 866E6 for Europe
    // 915E6 for North America
    if (!LoRa.begin(915E6)) {
        Serial.println("Starting LoRa failed!");
        while (1);
    }

    // Set Receive Call-back function
    LoRa.onReceive(onReceive);

    // Place LoRa in Receive Mode
    LoRa.receive();

    Serial.println("LoRa init succeeded.");
}

void loop() {
    // Run only if requested
    if (inMessage != inMessageOld) {
        // New message variable, take reading and send to controller

        if (inMessage.startsWith("OLDS_PING#")) {
            // Getting ping request
            String strTime = inMessage.substring(10);
            unsigned long timeMilis = convertStrToLong(strTime);
            unsigned long durationMilis = millis() - timeMilis;
            Logger::log(logLevel, "Received ping - time: " + String(durationMilis) + "ms");
            sendMessage("OLDS_PONG#Reply from " + String(localAddress) + " - duration: " + String(durationMilis) + "ms, localTime: " + String(millis()));
        }

        // Update the"old" data variable
        inMessageOld = inMessage;

        // Place LoRa in Receive Mode
        LoRa.receive();

        // Optional 2-second LED pulse (remark out if LED not used)
        digitalWrite(ledPin, HIGH);
        Logger::log(logLevel, "Receiving");

        // 2-second delay for DHT sensor
        delay(2000);

        // Optional 2-second LED pulse (remark out if LED not used)
        digitalWrite(ledPin, LOW);
    }
}