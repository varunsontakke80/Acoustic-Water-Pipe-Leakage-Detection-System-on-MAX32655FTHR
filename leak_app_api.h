/**
 * @file    leak_app_api.h
 * @brief   Water Pipe Leakage Detection BLE Application Interface
 */

#ifndef LEAK_APP_API_H_
#define LEAK_APP_API_H_

#include "wsf_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Start BLE advertising and initialize the leak detection application.
 */
void LeakAppStart(void);

/**
 * @brief  Application handler initialization function called during system boot.
 * @param  handlerId WSF handler ID assigned to this application.
 */
void LeakAppHandlerInit(wsfHandlerId_t handlerId);

/**
 * @brief  WSF event handler for BLE messages, connection events, and periodic timer.
 * @param  event WSF event mask.
 * @param  pMsg  WSF message header.
 */
void LeakAppHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/**
 * @brief  Broadcast or notify an ASCII string over BLE data characteristic.
 * @param  msg Null-terminated text string.
 */
void LeakAppSendTelemetry(const char *msg);

#ifdef __cplusplus
}
#endif

#endif /* LEAK_APP_API_H_ */

