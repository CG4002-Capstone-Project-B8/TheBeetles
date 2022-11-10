#include <beetle.h>
#include <Wire.h>

#define PLAYER_ID 0
#define DEVICE_ID 1

#define BAUDRATE 115200
#define PACKET_SIZE 20
#define NUM_DATA_VALUES 3
#define TIMEOUT 2000 
#define SAMPLE_INTERVAL 47.75

#define PACKET_TYPE_SHIFT 6
#define SEQNUM_SHIFT 5
#define PLAYER_ID_SHIFT 4
#define DEVICE_ID_SHIFT 2

// global variables
uint8_t current_seq_num = 0;
unsigned long send_time = 0;
bool handshake_done = false;

//void(* resetFunc) (void) = 0;

// IMU globals
const int MPU = 0x68; // MPU6050 I2C address
int16_t gyro_x, gyro_y, gyro_z;
float acc_x, acc_y, acc_z;
float acc_error_x, acc_error_y, gyro_error_x, gyro_error_y, gyro_error_z;
int c = 0;

void setup() {
  // Setup serial interface
  Serial.begin(BAUDRATE);

  // Initialize IMU
  initIMU();
}

void loop() {
  // start off with the 3-way handshake
  performHandshake();

  // send IMU data at fixed rate of 20 Hz
  sendGyroData();
  
  // check for any handshake from laptop in case of disconnection
  checkAnyPacketReceived();
}

void initIMU() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  calculateIMUError();
}

void calculateIMUError() {
  // We can call this function in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    acc_z = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    
    // Sum all readings
    acc_error_x = acc_error_x + ((atan((acc_y) / sqrt(pow((acc_x), 2) + pow((acc_z), 2))) * 180 / PI));
    acc_error_y = acc_error_y + ((atan(-1 * (acc_x) / sqrt(pow((acc_y), 2) + pow((acc_z), 2))) * 180 / PI));
    c++;
  }
  
  //Divide the sum by 200 to get the error value
  acc_error_x = acc_error_x / 200;
  acc_error_y = acc_error_y / 200;
  
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    
    // Sum all readings
    gyro_error_x = gyro_error_x + (gyro_x / 131.0);
    gyro_error_y = gyro_error_y + (gyro_y / 131.0);
    gyro_error_z = gyro_error_z + (gyro_z / 131.0);
    c++;
  }
  
  //Divide the sum by 200 to get the error value
  gyro_error_x = gyro_error_x / 200;
  gyro_error_y = gyro_error_y / 200;
  gyro_error_z = gyro_error_z / 200;
}

void sendGyroData() {
  if ((millis() - send_time) > SAMPLE_INTERVAL) {
    sendData();
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

      // read IMU data from sensors and set their values in the packet
      readData();
      
      packet->gyro_data[0] = gyro_x;
      packet->gyro_data[1] = gyro_y;
      packet->gyro_data[2] = gyro_z;

      packet->accel_data[0] = acc_x;
      packet->accel_data[1] = acc_y;
      packet->accel_data[2] = acc_z;
            
      packet->checksum = getChecksum(packet);
      break;
      
    default:
      // no effect, won't reach this branch
      packet->details = 0x00;
  }
}

void readData(){
  // === Read accelerometer data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  acc_x = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  acc_y = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  acc_z = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  
  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gyro_x = (Wire.read() << 8 | Wire.read()) ; 
  gyro_y = (Wire.read() << 8 | Wire.read()) ;
  gyro_z = (Wire.read() << 8 | Wire.read()) ;
  
  // Correct the outputs with the calculated error values
  gyro_x = gyro_x + gyro_error_x; // ~(-0.56)
  gyro_y = gyro_y + gyro_error_y; // ~(2)
  gyro_z = gyro_z + gyro_error_z; // ~(-0.8)
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
