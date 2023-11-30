# nrf9160-camera-upload

The code available here processes the data provided by Raspberry pi over serial interface and sends it to UDP socket. The code also sends telemetry, such as battery status and other details. This code complies with several warnings - no warranty provided :) I did not have much time to fix them as I needed the code to work very quickly. I have implemented remote reboot and several other options to control the hardware from remote(over nrfcloud API). Also there is no security implemented on the level of telemetry and picture upload - just raw udp packets. This code was originally written for NRF9160DK and later on I ported it to IcarusV2 as I needed smaller format of hardware with GPIO. I will provide block diagram and more details later on.

The code to process the transmitted data is yet to come.

This code contains code of other projects such as: 
1. Samples from NRFcode(main skeleton) - nrf_cloud_multi_mqtt, udp sample 
2. https://github.com/jimmywong2003/nrf52_ble_transfer_jpg/tree/master - Slightly rewritten code as I dont use BLE
3. Some b-parasite project code(https://github.com/rbaron/b-parasite), but not really used, Work in progress.
