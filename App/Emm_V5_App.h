#ifndef __EMM_V5_APP_H
#define __EMM_V5_APP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EMM_GIMBAL_PAN_MOTOR_ADDR            (0x01U)
#define EMM_GIMBAL_TILT_MOTOR_ADDR           (0x02U)

#define EMM_GIMBAL_DIR_CW                    (0U)
#define EMM_GIMBAL_DIR_CCW                   (1U)

#define EMM_GIMBAL_PAN_DIR_INVERT            (0U)
#define EMM_GIMBAL_TILT_DIR_INVERT           (0U)

#define EMM_GIMBAL_MAX_RPM                   (5000)
#define EMM_GIMBAL_DEFAULT_ACC               (80U)
#define EMM_GIMBAL_SYNC_FLAG                 (false)

#define EMM_GIMBAL_ENABLE_ON_BOOT            (false)

#define EMM_GIMBAL_USB_CMD_BUF_SIZE          (96U)
#define EMM_GIMBAL_USB_TX_BUF_SIZE           (128U)

void Emm_V5_App_Init(void);
void Emm_V5_App_SetEnable(bool enable);
void Emm_V5_App_SetVelocity(int16_t pan_rpm, int16_t tilt_rpm);
void Emm_V5_App_Stop(void);
void Emm_V5_App_USB_CDC_Input(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
