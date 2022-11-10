#include <beetle.h>
#include <Arduino.h>
#define DECODE_NEC  // Library specifies to do this, based on which protocol you are using

#define PLAYER_ID 1
#define DEVICE_ID 3

#define BAUDRATE 115200
#define PACKET_SIZE 20
#define NUM_DATA_VALUES 3
#define SENDING_INTERVAL 500

#define IR_RECEIVE_PIN    2 
#define BUZZER 3
#define DEBOUNCE_THRESHOLD 500
#define BUZZER_LED_ON_THRESHOLD 500
#define LED_PIN 4

#include <IRremote.hpp>

#define PACKET_TYPE_SHIFT 6
#define SEQNUM_SHIFT 5
#define PLAYER_ID_SHIFT 4
#define DEVICE_ID_SHIFT 2
#define SEND_SHOT_SHIFT 1
#define RECEIVE_SHOT_SHIFT 0

// global variables
uint8_t current_seq_num = 0;
volatile unsigned long send_time = 0;
volatile bool handshake_done = false;

// receiver globals
unsigned long db_last_time = 0, db_curr_time = 0;
unsigned long buzzer_led_on_time = 0;

volatile bool has_received_shot = false;

// has P1's address to detect if they were shot by P1
uint8_t PLAYER_ONE_RECEIVE_COMMAND = 0xAA;
uint16_t PLAYER_ONE_ADDRESS = 0x1B12;
uint32_t PLAYER_ONE_RAW_DATA = 0xFF1B12;

void setup() {
  // Setup serial interface
  Serial.begin(BAUDRATE);

  // initialize receiver
  initReceiverHardware();
}

void loop() {
  // start off with the 3-way handshake
  performHandshake();
  
  // check if got shot and call necessary functions
  checkIfReceivedShot();
  
  // handshake done at this point
  checkAnyPacketReceived();
}

void initReceiverHardware() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  IrReceiver.begin(IR_RECEIVE_PIN); // Start the receiver
  digitalWrite(LED_PIN, LOW);
}

void checkIfReceivedShot() {
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.command == PLAYER_ONE_RECEIVE_COMMAND) {
      // hardware functions when shot is received
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BUZZER, HIGH);
      buzzer_led_on_time = millis();

      // send shot received packet to laptop
      has_received_shot = true;
      sendData();
      has_received_shot = false;
      delay(250);
    }
    IrReceiver.resume(); //resume next value
  }

  if ((millis() - send_time) > SENDING_INTERVAL) {
    // send 'blank' packet to laptop to keep protocol active (with no shot received)
    sendData();
  }

  if ((millis() - buzzer_led_on_time) > BUZZER_LED_ON_THRESHOLD) {
    digitalWrite(BUZZER, LOW);
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

      // add status for whether player got shot
      packet->details |= has_received_shot << RECEIVE_SHOT_SHIFT;

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
