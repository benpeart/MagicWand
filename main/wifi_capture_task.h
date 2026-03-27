#ifndef WIFI_CAPTURE_TASK_H
#define WIFI_CAPTURE_TASK_H

/**
 * @brief Start the WiFi capture task
 * 
 * Initializes WiFi connection and starts the task that sends IMU data
 * from the fusion queue to a remote server over TCP/IP socket.
 * 
 * Configuration:
 * - SSID and password should be set via menuconfig or hardcoded below
 * - Remote server IP and port should be configured
 * 
 * Data format: Same CSV format as data_capture_task but sent over network
 */
void wifi_capture_task_start(void);

#endif // WIFI_CAPTURE_TASK_H
