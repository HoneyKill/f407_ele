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
#define EMM_GIMBAL_PAN_AXIS_ENABLE           (1U)
#define EMM_GIMBAL_TILT_AXIS_ENABLE          (1U)

#define EMM_GIMBAL_MAX_RPM                   (1000)
#define EMM_GIMBAL_DEFAULT_ACC               (80U)
#define EMM_GIMBAL_SYNC_FLAG                 (false)
#define EMM_GIMBAL_AXIS_CMD_GAP_MS           (0U)
/* Shared UART bus needs a minimum turnaround gap between axis commands. */
#define EMM_GIMBAL_AXIS_CMD_MIN_GAP_MS       (5U)

#define EMM_GIMBAL_ENABLE_ON_BOOT            (false)
#define EMM_GIMBAL_INIT_SEND_STOP            (0U)
#define EMM_GIMBAL_INIT_SEND_ENABLE          (0U)
/* Some EMM variants use active-low enable command value. */
#define EMM_GIMBAL_ENABLE_ACTIVE_HIGH        (0U)

#define EMM_GIMBAL_USB_CMD_BUF_SIZE          (96U)
#define EMM_GIMBAL_USB_TX_BUF_SIZE           (128U)
#define EMM_GIMBAL_USB_DEBUG_REPLY_ENABLE    (1U)
#define EMM_GIMBAL_CMD_QUEUE_SIZE            (16U)

#define EMM_GIMBAL_VOFA_ENABLE               (1U)
#define EMM_GIMBAL_VOFA_HEADER_0             (0x00U)
#define EMM_GIMBAL_VOFA_HEADER_1             (0xFFU)
#define EMM_GIMBAL_VOFA_HEADER_2             (0xFAU)
#define EMM_GIMBAL_VOFA_FRAME_SIZE           (8U)
#define EMM_GIMBAL_VOFA_CMD_CORNER_X         (0xC0U)
#define EMM_GIMBAL_VOFA_CMD_CORNER_Y         (0xC1U)
#define EMM_GIMBAL_VOFA_CMD_STATE            (0xCCU)
#define EMM_GIMBAL_VOFA_CMD_VECTOR_X         (0xD0U)
#define EMM_GIMBAL_VOFA_CMD_VECTOR_Y         (0xD1U)
#define EMM_GIMBAL_VOFA_NO_TARGET_ABS_MIN    (900.0f)
#define EMM_GIMBAL_VOFA_VECTOR_DEADBAND      (3.0f)
#define EMM_GIMBAL_VOFA_PAN_GAIN             (1.2f)
#define EMM_GIMBAL_VOFA_TILT_GAIN            (1.2f)
#define EMM_GIMBAL_VOFA_PAN_INVERT           (0U)
#define EMM_GIMBAL_VOFA_TILT_INVERT          (0U)
#define EMM_GIMBAL_VOFA_TIMEOUT_MS           (200U)
#define EMM_GIMBAL_VOFA_CTRL_PERIOD_MS       (20U)

void Emm_V5_App_Init(void);
void Emm_V5_App_Task(void);
void Emm_V5_App_SetEnable(bool enable);
void Emm_V5_App_SetVelocity(int16_t pan_rpm, int16_t tilt_rpm);
void Emm_V5_App_Stop(void);
void Emm_V5_App_USB_CDC_Input(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif
