#include "fc_data_transmission.h"

#include <math.h>

#include "dji_camera_manager.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "uart.h"
#include "util_misc.h"

LARGE_PACKET_t packet = {FRAME1, FRAME2, MESSAGE_ID, DATA_LENGTH};

#define FC_SUBSCRIPTION_TASK_FREQ (5)
#define FC_SUBSCRIPTION_TASK_STACK_SIZE (1024)

static bool s_userFcSubscriptionDataShow = false;
static uint8_t s_totalSatelliteNumberUsed = 0;
static uint32_t s_userFcSubscriptionDataCnt = 0;

static T_DjiTaskHandle s_userFcSubscriptionThread;

static T_DjiReturnCode Dji_FcSubscriptionReceiveGpsDateCallback(
    const uint8_t *data, uint16_t dataSize,
    const T_DjiDataTimestamp *timestamp);

static T_DjiReturnCode Dji_FcSubscriptionReceiveGpsTimeCallback(
    const uint8_t *data, uint16_t dataSize,
    const T_DjiDataTimestamp *timestamp);

static T_DjiReturnCode Dji_FcSubscriptionReceiveQuaternionCallback(
    const uint8_t *data, uint16_t dataSize,
    const T_DjiDataTimestamp *timestamp);

static T_DjiReturnCode Dji_CameraManagerGetLidarRangingInfo(
    E_DjiMountPosition position);

static void *UserFcSubscription_Task(void *arg);

T_DjiReturnCode Fc_SubscriptionStartService(void) {
  T_DjiReturnCode djiStat;
  T_DjiOsalHandler *osalHandler = NULL;

  osalHandler = DjiPlatform_GetOsalHandler();
  djiStat = DjiFcSubscription_Init();
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("init data subscription module error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  }

  /*GPS年月日订阅,回调方式*/
  djiStat = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
      Dji_FcSubscriptionReceiveGpsDateCallback);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic gps data error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic gps data success.");
  }

  /*GPS时分秒订阅,回调方式*/
  djiStat = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
      Dji_FcSubscriptionReceiveGpsTimeCallback);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic gps time error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic gps time success.");
  }

  /*GPS经纬度订阅,回调*/
  djiStat =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                       DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic gps position error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic gps position success.");
  }

  /*RTK经纬度订阅，非回调*/
  djiStat =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                       DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic rtk position error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic rtk position success.");
  }

  /*GPS速度订阅，非回调*/
  djiStat =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY,
                                       DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic gps velocity error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic gps velocity success.");
  }

  /*RTk速度订阅，非回调*/
  djiStat =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
                                       DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic rtk velocity error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic rtk velocity success.");
  }

  /*速度订阅，非回调*/
  djiStat =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                       DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic velocity error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic velocity success.");
  }

  /*姿态四元数数据订阅,回调方式*/
  djiStat = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
      Dji_FcSubscriptionReceiveQuaternionCallback);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic quaternion error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic quaternion success.");
  }

  /* gps星数,定位状态 */
  djiStat =
      DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                       DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic gps details error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic gps details success.");
  }

  /* rtk定位状态 */
  djiStat = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
      DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, NULL);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Subscribe topic rtk details error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  } else {
    USER_LOG_DEBUG("Subscribe topic rtk details success.");
  }

  if (osalHandler->TaskCreate("user_subscription_task", UserFcSubscription_Task,
                              FC_SUBSCRIPTION_TASK_STACK_SIZE, NULL,
                              &s_userFcSubscriptionThread) !=
      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("user data subscription task create error.");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Dji_FcSubscriptionReceiveGpsDateCallback(
    const uint8_t *data, uint16_t dataSize,
    const T_DjiDataTimestamp *timestamp) {
  T_DjiFcSubscriptionGpsDate *date = (T_DjiFcSubscriptionGpsDate *)data;

  USER_UTIL_UNUSED(dataSize);
  packet.Senses_Data_t.timestamp_ymd = *date;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Dji_FcSubscriptionReceiveGpsTimeCallback(
    const uint8_t *data, uint16_t dataSize,
    const T_DjiDataTimestamp *timestamp) {
  T_DjiFcSubscriptionGpsTime *time = (T_DjiFcSubscriptionGpsTime *)data;

  USER_UTIL_UNUSED(dataSize);
  packet.Senses_Data_t.timestamp_hms = *time;
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Dji_FcSubscriptionReceiveQuaternionCallback(
    const uint8_t *data, uint16_t dataSize,
    const T_DjiDataTimestamp *timestamp) {
  T_DjiFcSubscriptionQuaternion *quaternion =
      (T_DjiFcSubscriptionQuaternion *)data;

  USER_UTIL_UNUSED(dataSize);

  packet.Senses_Data_t.pitch =
      (dji_f32_t)asinf(-2 * quaternion->q1 * quaternion->q3 +
                       2 * quaternion->q0 * quaternion->q2) *
      57.3f;
  packet.Senses_Data_t.roll =
      (dji_f32_t)atan2f(2 * quaternion->q2 * quaternion->q3 +
                            2 * quaternion->q0 * quaternion->q1,
                        -2 * quaternion->q1 * quaternion->q1 -
                            2 * quaternion->q2 * quaternion->q2 + 1) *
      57.3f;
  packet.Senses_Data_t.yaw =
      (dji_f32_t)atan2f(2 * quaternion->q1 * quaternion->q2 +
                            2 * quaternion->q0 * quaternion->q3,
                        -2 * quaternion->q2 * quaternion->q2 -
                            2 * quaternion->q3 * quaternion->q3 + 1) *
      57.3f;

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void *UserFcSubscription_Task(void *arg) {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionVelocity velocity = {0};
  T_DjiDataTimestamp timestamp = {0};

  T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
  T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
  T_DjiFcSubscriptionGpsVelocity gpsVelocity = {0};

  T_DjiFcSubscriptionRtkPositionInfo rtkPositionInfo = {0};
  T_DjiFcSubscriptionRtkPosition rtkPosition = {0};
  T_DjiFcSubscriptionRtkVelocity rtkVelocity = {0};
  T_DjiOsalHandler *osalHandler = NULL;

  memset(packet.Senses_Data_t.reserved, 0, LEN);

  USER_UTIL_UNUSED(arg);
  osalHandler = DjiPlatform_GetOsalHandler();

  while (1) {
    osalHandler->TaskSleepMs(1000 / FC_SUBSCRIPTION_TASK_FREQ);

    /*主动获取gps细节信息*/
    djiStat = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, (uint8_t *)&gpsDetails,
        sizeof(T_DjiFcSubscriptionGpsDetails), &timestamp);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      USER_LOG_ERROR("get value of topic gps details error.");
    }
    packet.Senses_Data_t.timestamp_ms = timestamp.microsecond;
    packet.Senses_Data_t.total_satellite_number_used =
        gpsDetails.totalSatelliteNumberUsed;
    packet.Senses_Data_t.hdop = gpsDetails.hdop;
    packet.Senses_Data_t.pdop = gpsDetails.pdop;
    packet.Senses_Data_t.vacc = gpsDetails.vacc;
    packet.Senses_Data_t.hacc = gpsDetails.hacc;
    packet.Senses_Data_t.sacc = gpsDetails.sacc;
    /*主动获取rtk定位状态信息*/
    djiStat = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        (uint8_t *)&rtkPositionInfo, sizeof(T_DjiFcSubscriptionRtkPositionInfo),
        &timestamp);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      USER_LOG_ERROR("get value of topic rtk details error.");
    }

    if (rtkPositionInfo >
        DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_SINGLE_PNT_SOLUTION) {
      packet.Senses_Data_t.fixed_status = rtkPositionInfo;  // RTK定位有效
      djiStat = DjiFcSubscription_GetLatestValueOfTopic(
          DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, (uint8_t *)&rtkPosition,
          sizeof(T_DjiFcSubscriptionRtkPosition), &timestamp);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get value of topic gps position error.");
      }

      packet.Senses_Data_t.longitude = rtkPosition.longitude * 1e7;
      packet.Senses_Data_t.latitude = rtkPosition.latitude * 1e7;
      packet.Senses_Data_t.altitude = rtkPosition.hfsl;
      djiStat = DjiFcSubscription_GetLatestValueOfTopic(
          DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, (uint8_t *)&rtkVelocity,
          sizeof(T_DjiFcSubscriptionRtkVelocity), &timestamp);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get value of topic gps position error.");
      }
      packet.Senses_Data_t.velocity_north = rtkVelocity.x / 100.0f;
      packet.Senses_Data_t.velocity_east = rtkVelocity.y / 100.0f;
      packet.Senses_Data_t.velocity_up = rtkVelocity.z / 100.0f;
    } else if (gpsDetails.fixState >=
                   DJI_FC_SUBSCRIPTION_GPS_FIX_STATE_2D_FIX ||
               gpsDetails.fixState <=
                   DJI_FC_SUBSCRIPTION_GPS_FIX_STATE_GPS_PLUS_DEAD_RECKONING) {
      /*主动获取gps经纬度信息*/
      packet.Senses_Data_t.fixed_status = gpsDetails.fixState;  // GPS定位有效
      djiStat = DjiFcSubscription_GetLatestValueOfTopic(
          DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, (uint8_t *)&gpsPosition,
          sizeof(T_DjiFcSubscriptionGpsPosition), &timestamp);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get value of topic gps position error.");
      }
      packet.Senses_Data_t.longitude = gpsPosition.x;
      packet.Senses_Data_t.latitude = gpsPosition.y;
      packet.Senses_Data_t.altitude = gpsPosition.z / 1000.0;  // 单位转换为m

      /*主动获取gps速度信息*/
      djiStat = DjiFcSubscription_GetLatestValueOfTopic(
          DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY, (uint8_t *)&gpsVelocity,
          sizeof(T_DjiFcSubscriptionVelocity), &timestamp);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get value of topic velocity error.");
      }
      packet.Senses_Data_t.velocity_north = gpsVelocity.x / 100.0f;
      packet.Senses_Data_t.velocity_east = gpsVelocity.y / 100.0f;
      packet.Senses_Data_t.velocity_up = gpsVelocity.z / 100.0f;
    } else {
      packet.Senses_Data_t.fixed_status = 0;  // 定位无效
      packet.Senses_Data_t.longitude = 0.0;
      packet.Senses_Data_t.latitude = 0.0;
      packet.Senses_Data_t.altitude = 0.0;
      /*主动获取飞行器速度信息*/
      djiStat = DjiFcSubscription_GetLatestValueOfTopic(
          DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, (uint8_t *)&velocity,
          sizeof(T_DjiFcSubscriptionVelocity), &timestamp);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get value of topic velocity error.");
      }
      packet.Senses_Data_t.velocity_north = velocity.data.x / 100.0f;
      packet.Senses_Data_t.velocity_east = velocity.data.y / 100.0f;
      packet.Senses_Data_t.velocity_up = velocity.data.z / 100.0f;
    }

    Dji_CameraManagerGetLidarRangingInfo(
        DJI_MOUNT_POSITION_UNKNOWN);  // 注意修改与载荷接口对应

    sum_checksum(&packet);
    xor_checksum(&packet);

    // 串口发送数据
    UART_Write(DJI_TRANSMISSION_UART_NUM, (uint8_t *)&packet, Packet_Length);
  }
}

// 和校验（Sum Checksum）计算函数
void sum_checksum(LARGE_PACKET_t *packet) {
  uint8_t sum = 0;
  // 累加从载荷长度位到和校验位前的数据
  for (int i = CHECK_START; i < CHECK_LENGTH;
       i++) {  // CHECK_START = 3, CHECK_LENGTH = 103
    sum += packet->raw_large[i];
  }
  packet->Senses_Data_t.sum_check = sum;
}

// 异或校验（XOR Checksum）计算函数
void xor_checksum(LARGE_PACKET_t *packet) {
  uint8_t xor_result = 0;
  // 从载荷长度位到异或校验位前的数据
  for (int i = CHECK_START; i <= CHECK_LENGTH; i++) {
    xor_result ^= packet->raw_large[i];
  }
  packet->Senses_Data_t.xor_check = xor_result;
}

static T_DjiReturnCode Dji_CameraManagerGetLidarRangingInfo(
    E_DjiMountPosition position) {
  T_DjiReturnCode returnCode = 0;

  T_DjiCameraManagerLaserRangingInfo rangingInfo = {0};

  returnCode = DjiCameraManager_GetLaserRangingInfo(position, &rangingInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR(
        "Get mounted position %d laser ranging info failed, error code: "
        "0x%08X.",
        position, returnCode);
    return returnCode;
  }

  packet.Senses_Data_t.tgt_longitude = rangingInfo.longitude;
  packet.Senses_Data_t.tgt_altitude = rangingInfo.altitude;
  packet.Senses_Data_t.tgt_latitude = rangingInfo.latitude;
  packet.Senses_Data_t.laser_distance = rangingInfo.distance;
  USER_LOG_INFO(
      "Receive lidar range info, lon:%.6f, lat:%.6f, alt:%.1f, dis:%d, "
      "enable:%d, exception:%d, x:%d, y:%d",
      rangingInfo.longitude, rangingInfo.latitude,
      (float)rangingInfo.altitude / 10, rangingInfo.distance,
      rangingInfo.enable_lidar, rangingInfo.exception, rangingInfo.screenX,
      rangingInfo.screenY);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
