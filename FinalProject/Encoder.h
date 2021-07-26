#ifndef _Encoder
#define _Encoder

#define ENCODER_MAX_COUNT 16383.0
#define ENCODER_OFFSET 6900

void setupEncoder();
unsigned int readEncoder();
float getBallAngle(int enc_pos);


#endif