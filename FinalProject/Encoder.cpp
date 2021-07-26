#include "Encoder.h"
#include <Arduino.h>
#include <SPI.h>

void setupEncoder() {
  pinMode(2, OUTPUT);
}

unsigned int readEncoder() {
    byte enc_msb;
    byte enc_lsb; 
    int enc_resp;
    int enc_pos;
    // Start an SPI transaction with proper config
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
    digitalWrite(2, LOW); // enable Slave Select

    // Get reading
    enc_msb = SPI.transfer(0xFF); // read encoder high bits
    enc_lsb = SPI.transfer(0xFF); // read encoder low bits
    enc_resp = (enc_msb << 8) | enc_lsb; // combine encoder readings into 16 bits
    enc_pos = enc_resp & 0b0011111111111111; // extract 14 bit position

    // End transaction
    digitalWrite(2, HIGH); // disable Slave Select
    SPI.endTransaction(); // End transaction
    
    return enc_pos;
}

// Get angle of sphere relative to frame
float getBallAngle(int enc_pos) {
    float angle;

    // Get angle depending on whether encoder count is greater or lower than zero position
    if (enc_pos > ENCODER_OFFSET) {
      angle = 360 * (((float)(enc_pos - ENCODER_OFFSET)) / ENCODER_MAX_COUNT);
    } else {
      angle = 360 + 360 * (((float)(enc_pos - ENCODER_OFFSET)) / ENCODER_MAX_COUNT);
    }

    // Force the sign convention to match what we want
    if (abs(angle) > 180) return - (angle - 360.0);
    else return -angle;
}