# LoRa ESP32 Component



Basic usage:

1. Call `LoRaComponHwInit()` at start up, to initialise related hardware.
2. Call `LoRaComponStart()` to start the LoRa component.
3. Using `LoRaComponIsJoined()` to check JOIN status.
4. After JOIN, you can use `LoRaComponSendData()` to send data. Then use `LoRaComponIsSendSuccess()` and `LoRaComponIsSendFailure()` to check the result.
5. Using `LoRaComponIsRxReady()` to check if there is data received. Then use `LoRaComponGetData()` to get the received data, and related information if needed.
