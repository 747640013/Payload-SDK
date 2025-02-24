#ifndef __FC_DATA_TRANSMISSION_H
#define __FC_DATA_TRANSMISSION_H

#include <dji_typedef.h>
#include <stdint.h>

#define Packet_Length 105
#define FRAME1 0xAA
#define FRAME2 0x55
#define MESSAGE_ID 0x01
#define DATA_LENGTH 0x63
#define CHECK_START 3
#define CHECK_LENGTH 103



#ifdef __cplusplus
extern "C" {
#endif

typedef union {
  struct __attribute__((packed)) {
    uint8_t start_frame1;
    uint8_t start_frame2;
    uint8_t message_id;
    uint8_t data_length;
    uint32_t timestamp_ymd;
    uint32_t timestamp_hms;
    uint32_t timestamp_ms;
    dji_f64_t longitude;
    dji_f64_t latitude;
    dji_f32_t altitude;
    dji_f32_t velocity_north;
    dji_f32_t velocity_east;
    dji_f32_t velocity_up;
    dji_f64_t roll;
    dji_f64_t pitch;
    dji_f64_t yaw;
    uint16_t total_satellite_number_used;
    dji_f32_t fixed_status;
    uint8_t reserved1;
    int32_t laser_distance;
    dji_f64_t tgt_longitude;
    dji_f64_t tgt_latitude;
    int32_t tgt_altitude;
    uint8_t sum_check;
    uint8_t xor_check;
  } Senses_Data_t;
  uint8_t raw_large[Packet_Length];
} LARGE_PACKET_t;



#ifdef __cplusplus
}
#endif

#endif
