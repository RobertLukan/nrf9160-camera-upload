/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _APPLICATION_H_
#define _APPLICATION_H_

/**
 * @brief Main application -- Wait for valid connection, start location tracking, and then
 * periodically sample sensors and send them to nRF Cloud.
 */
struct cameraControl
  {
    //   char request_Type[100];
      bool takePictureNow;
      bool processing;
      bool processed;
      int64_t lastPictureTaken;
      int64_t nextScheduledTime;
      int8_t delayUdpSend;
      int8_t quality;
      bool powerSave;
      int8_t cameraFrequency;
      int8_t solarChargerEnable;
	//   uint8_t m_hash[10];
  };


struct boardStatus
  {
    //   char request_Type[100];
      uint16_t batteryVoltage;
      int8_t solarChargerEnable;
      int8_t solarChargerStatus;
      int8_t solarChargerPGood;
      int8_t rpiChargerEnable;
      int8_t rpiChargerLow;
      int64_t lastPictureTaken;
      int64_t nextScheduledTime;
      int8_t delayUdpSend;
      uint16_t modemBatteryVoltage;
      uint16_t rpiBatteryVoltage;
      uint16_t rpiBatteryPercent;
      int8_t lastPictureQuality;
   //   int8_t md5Array[16];
      int8_t rpiShutdown;
      bool icarusChargerEnable;
      bool icarusChargerStatus;

	//   uint8_t m_hash[10];
  };

void main_application_thread_fn(void);



#endif /* _APPLICATION_H_ */
