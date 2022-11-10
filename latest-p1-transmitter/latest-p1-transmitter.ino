#include <beetle.h>
#include <Arduino.h>

#define PLAYER_ID 0
#define DEVICE_ID 2

#define BAUDRATE 115200
#define PACKET_SIZE 20
#define NUM_DATA_VALUES 3
#define SENDING_INTERVAL 500

#define PACKET_TYPE_SHIFT 6
#define SEQNUM_SHIFT 5
#define PLAYER_ID_SHIFT 4
#define DEVICE_ID_SHIFT 2
#define SEND_SHOT_SHIFT 1
#define RECEIVE_SHOT_SHIFT 0

#define DEBOUNCE_THRESHOLD 2000
#define LED_ON_THRESHOLD 500

#define IR_SEND_PIN 3
#define LED_PIN 5
#define TRIGGER_PIN 2

#include <IRremote.hpp>

// global variables
uint8_t current_seq_num = 0;
volatile unsigned long send_time = 0;
volatile bool handshake_done = false;

// emitter globals
volatile unsigned long db_last_time = 0, db_curr_time = 0;
volatile unsigned long led_on_time = 0;

uint8_t SEND_REPEATS = 1;
uint8_t PLAYER_ONE_SEND_COMMAND = 0xAA;

volatile bool has_sent_shot = false;
uint16_t PLAYER_ONE_ADDRESS = 0x1B12; // Data to be encoded in the IR beam
uint32_t PLAYER_ONE_RAW_DATA = 0xFF1B12;

void setup() {
  // Setup serial interface
  Serial.begin(BAUDRATE);

  // initialize emitter
  initEmitterHardware();
}

void loop() {
  // start off with the 3-way handshake
  performHandshake();

  // check for shot and call necessary functions
  checkIfSentShot();
  
  // handshake done at this point
  checkAnyPacketReceived();
}

/**
 * ISR for pushbutton (gun trigger)
 */
void triggerISR() {
  // debouncing pushbutton
  db_curr_time = millis();
  if (db_curr_time - db_last_time > DEBOUNCE_THRESHOLD) {
    db_last_time = db_curr_time;
    has_sent_shot = true;  
  }
}

void initEmitterHardware() {
  IrSender.begin();
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), triggerISR, RISING);
  pinMode(LED_PIN, OUTPUT);

  // flash to indicate that Beetle is turned on
  digitalWrite(LED_PIN, HIGH);
  delay(250);
  digitalWrite(LED_PIN, LOW);
}

void checkIfSentShot() {
  if (has_sent_shot) {
    // hardware functions to send shot
    IrSender.sendNEC(PLAYER_ONE_ADDRESS, PLAYER_ONE_SEND_COMMAND, SEND_REPEATS);
    digitalWrite(LED_PIN, HIGH);
    led_on_time = millis();
    
    // send shot packet to laptop
    sendData(); 
    has_sent_shot = false;
  }

  if ((millis() - send_time) > SENDING_INTERVAL) {    
    // send blank packet to laptop to keep protocol active (with no shot sent)
    sendData(); 
  }

  // turn off LED after certain period of time
  if ((millis() - led_on_time) > LED_ON_THRESHOLD) {
    digitalWrite(LED_PIN, LOW);
  }
}

void resetGlobals() {
  current_seq_num = 0;
  handshake_done = false;
}

void performHandshake() {
  while(!handshake_done) {
    if (Serial.available()) {
      char packet = Serial.read();
      if (packet == 'H') {
        sendHandshakeAck();
      } else if (packet == 'A') {
        handshake_done = true;
      }
    }
  }
}

void checkAnyPacketReceived() {
  if (handshake_done && Serial.available()) {
    char packet = Serial.read();
    if (packet == 'H') {
      resetGlobals();
      sendHandshakeAck();
    }
  }
}

void createPacket(TPacket *packet, TPacketType type) {
  switch(type) {
    case PACKET_TYPE_ACK:
      packet->details = 0x00;
      packet->details |= PACKET_TYPE_ACK << PACKET_TYPE_SHIFT;
      packet->details |= current_seq_num << SEQNUM_SHIFT;
      packet->details |= PLAYER_ID << PLAYER_ID_SHIFT;
      packet->details |= DEVICE_ID << DEVICE_ID_SHIFT;

      for(int i = 0; i < NUM_DATA_VALUES; i++) {
        packet->accel_data[i] = 0;
        packet->gyro_data[i] = 0.0;
      }

      packet->checksum = getChecksum(packet);
      break;
      
    case PACKET_TYPE_DATA:
      current_seq_num = 1 - current_seq_num;      
      packet->details = 0x00;
      packet->details |= PACKET_TYPE_DATA << PACKET_TYPE_SHIFT;
      packet->details |= current_seq_num << SEQNUM_SHIFT;
      packet->details |= PLAYER_ID << PLAYER_ID_SHIFT;
      packet->details |= DEVICE_ID << DEVICE_ID_SHIFT;

      // add status for whether player shot their gun
      packet->details |= has_sent_shot << SEND_SHOT_SHIFT;

      // dummy data
      for(int i = 0; i < NUM_DATA_VALUES; i++) {
        packet->accel_data[i] = 0;
        packet->gyro_data[i] = 0.0;
      }
      
      packet->checksum = getChecksum(packet);
      break;
      
    default:
      // no effect, won't reach this branch
      packet->details = 0x00;
  }
}

void sendHandshakeAck() {
  TPacket ack_packet;
  char buffer[PACKET_SIZE];

  createPacket(&ack_packet, PACKET_TYPE_ACK);

  serialize(buffer, &ack_packet);
  sendSerial(buffer, PACKET_SIZE);
}

void sendData() {
  TPacket data_packet;
  char buffer[PACKET_SIZE];

  createPacket(&data_packet, PACKET_TYPE_DATA);

  serialize(buffer, &data_packet);
  sendSerial(buffer, PACKET_SIZE);
}

void sendSerial(const char *buffer, int len) {
  Serial.write(buffer, len);

  // set send time
  send_time = millis();
}
