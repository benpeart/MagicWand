#pragma once

/* --------------------------------------------------------------------------
 * ha_task()
 *
 * PURPOSE:
 *   The Home Assistant task is the dedicated runtime loop responsible for
 *   connecting the wand to the Home Assistant ecosystem. It initializes Wi‑Fi,
 *   establishes the MQTT session, publishes HA discovery metadata, and then
 *   continuously publishes telemetry (battery, RSSI) and gesture events. This
 *   task isolates all network and MQTT activity from the real‑time gesture
 *   pipeline to ensure deterministic IMU, fusion, and inference performance.
 *
 * WHAT THIS TASK DOES:
 *   1. Initializes Wi‑Fi in station mode.
 *      Brings up the network interface and connects to the given SSID.
 *      This step is blocking and must complete before MQTT can start.
 *
 *   2. Initializes and starts the MQTT client.
 *      Configures the broker URI, keepalive settings, and registers the global
 *      MQTT event handler. Once started, the client handles reconnects and
 *      session maintenance in the background.
 *
 *   3. Publishes Home Assistant discovery messages.
 *      After a short delay to ensure the MQTT session is active, the task
 *      publishes discovery payloads for:
 *        - Last gesture sensor
 *        - Battery sensor
 *        - Wi‑Fi RSSI sensor
 *      This allows Home Assistant to automatically create entities for the
 *      wand without manual configuration.
 *
 *   4. Periodically publishes device telemetry.
 *      Every 10 seconds the task publishes:
 *        - Battery percentage (fuel‑gauge value)
 *        - Wi‑Fi RSSI (via esp_wifi_sta_get_ap_info)
 *      These updates keep HA dashboards and automations in sync with the
 *      wand’s current status.
 *
 *   5. Forwards gesture events to Home Assistant.
 *      The task listens on g_gesture_queue for GestureEvent messages produced
 *      by the inference pipeline. Each event is serialized to JSON and
 *      published to the gesture MQTT topic. This is the primary mechanism by
 *      which HA automations react to wand gestures.
 *
 *   6. Runs indefinitely as a non‑blocking network loop.
 *      The task never blocks the gesture pipeline. All MQTT operations are
 *      asynchronous, and queue polling uses timeouts to ensure the loop
 *      remains responsive.
 *
 * WHY THIS DESIGN IS ESSENTIAL:
 *   - Network operations can stall or retry; isolating them prevents Wi‑Fi or
 *     MQTT delays from interfering with IMU sampling, fusion, or inference.
 *   - Home Assistant discovery and telemetry must be published reliably and
 *     independently of gesture recognition timing.
 *   - A dedicated task ensures clean separation of concerns: gesture logic
 *     stays real‑time, while HA integration remains robust and asynchronous.
 *
 * RESULT:
 *   A resilient, asynchronous Home Assistant integration loop that maintains
 *   Wi‑Fi connectivity, publishes MQTT discovery and telemetry, and forwards
 *   gesture events without impacting the real‑time motion pipeline.
 * -------------------------------------------------------------------------- */
void ha_task_start(void);
