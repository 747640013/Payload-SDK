#ifndef __FC_DATA_TRANSMISSION_H
#define __FC_DATA_TRANSMISSION_H

#include <dji_typedef.h>
#include <stdint.h>

#define Packet_Length 110
#define FRAME1 0xAA
#define FRAME2 0x55
#define MESSAGE_ID 0x01
#define DATA_LENGTH 0x68 // 载荷长度104
#define CHECK_START 3
#define CHECK_LENGTH 108
#define LEN 4  // 保留字段长度


#ifdef __cplusplus
extern "C" {
#endif

typedef union {
  struct __attribute__((packed)) {
    uint8_t start_frame1;
    uint8_t start_frame2;
    uint8_t message_id;
    uint8_t data_length;

    /* 时间戳 12字节*/
    uint32_t timestamp_ymd;
    uint32_t timestamp_hms;
    uint32_t timestamp_ms;

    /* 经纬度、高度 20字节*/
    dji_f64_t longitude;
    dji_f64_t latitude;
    dji_f32_t altitude;

    /* 速度 12字节*/
    dji_f32_t velocity_north;
    dji_f32_t velocity_east;
    dji_f32_t velocity_up;

    /* 欧拉角 12字节*/
    dji_f32_t roll;
    dji_f32_t pitch;
    dji_f32_t yaw;

    /* 定位精度 20字节*/
    dji_f32_t hdop;
    dji_f32_t pdop;
    dji_f32_t vacc;
    dji_f32_t hacc;
    dji_f32_t sacc;

    uint8_t total_satellite_number_used;
    uint8_t fixed_status;

    /* 测距相关信息 24字节*/
    uint16_t laser_distance;
    dji_f64_t tgt_longitude;
    dji_f64_t tgt_latitude;
    int32_t tgt_altitude;
    
    /* 保留字段 4字节*/
    uint8_t reserved[LEN];

    uint8_t sum_check;
    uint8_t xor_check;
  } Senses_Data_t;
  uint8_t raw_large[Packet_Length];
} LARGE_PACKET_t;

void sum_checksum(LARGE_PACKET_t *packet);
void xor_checksum(LARGE_PACKET_t *packet);

#ifdef __cplusplus
}
#endif

#endif
