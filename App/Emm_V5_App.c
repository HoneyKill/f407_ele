#include "Emm_V5_App.h"

#include "Emm_V5.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

static char s_usb_cmd_buf[EMM_GIMBAL_USB_CMD_BUF_SIZE] = {0};
static uint16_t s_usb_cmd_len = 0U;

static void Emm_V5_App_USB_Reply(const char *msg)
{
    static uint8_t tx_buf[EMM_GIMBAL_USB_TX_BUF_SIZE] = {0};
    size_t msg_len = strnlen(msg, EMM_GIMBAL_USB_TX_BUF_SIZE - 3U);

    memcpy(tx_buf, msg, msg_len);
    tx_buf[msg_len++] = '\r';
    tx_buf[msg_len++] = '\n';

    (void)CDC_Transmit_FS(tx_buf, (uint16_t)msg_len);
}

static void Emm_V5_App_ToUpper(char *str)
{
    while (*str != '\0') {
        if ((*str >= 'a') && (*str <= 'z')) {
            *str = (char)(*str - 'a' + 'A');
        }
        ++str;
    }
}

static int16_t Emm_V5_App_ClampRpm(int32_t rpm)
{
    if (rpm > EMM_GIMBAL_MAX_RPM) {
        return (int16_t)EMM_GIMBAL_MAX_RPM;
    }
    if (rpm < -EMM_GIMBAL_MAX_RPM) {
        return (int16_t)(-EMM_GIMBAL_MAX_RPM);
    }
    return (int16_t)rpm;
}

static uint16_t Emm_V5_App_AbsRpm(int16_t rpm)
{
    return (rpm >= 0) ? (uint16_t)rpm : (uint16_t)(-rpm);
}

static uint8_t Emm_V5_App_GetDir(int16_t rpm, bool invert)
{
    bool cw = (rpm >= 0);

    if (invert) {
        cw = !cw;
    }

    return cw ? EMM_GIMBAL_DIR_CW : EMM_GIMBAL_DIR_CCW;
}

static void Emm_V5_App_SendAxisVelocity(uint8_t addr, int16_t rpm, bool invert)
{
    uint8_t dir = Emm_V5_App_GetDir(rpm, invert);
    uint16_t abs_rpm = Emm_V5_App_AbsRpm(rpm);

    Emm_V5_Vel_Control(addr, dir, abs_rpm, EMM_GIMBAL_DEFAULT_ACC, EMM_GIMBAL_SYNC_FLAG);
}

static void Emm_V5_App_ProcessCmd(char *line)
{
    char cmd[16] = {0};
    int32_t v0 = 0;
    int32_t v1 = 0;

    if (sscanf(line, "%15s", cmd) != 1) {
        return;
    }

    Emm_V5_App_ToUpper(cmd);

    if (strcmp(cmd, "HELP") == 0) {
        Emm_V5_App_USB_Reply("OK CMD: HELP | EN <0|1> | VEL <PAN_RPM> <TILT_RPM> | STOP");
        return;
    }

    if (strcmp(cmd, "EN") == 0) {
        if (sscanf(line, "%*s %ld", &v0) == 1) {
            Emm_V5_App_SetEnable(v0 != 0);
            Emm_V5_App_USB_Reply((v0 != 0) ? "OK EN 1" : "OK EN 0");
        } else {
            Emm_V5_App_USB_Reply("ERR EN FORMAT");
        }
        return;
    }

    if (strcmp(cmd, "VEL") == 0) {
        if (sscanf(line, "%*s %ld %ld", &v0, &v1) == 2) {
            int16_t pan_rpm = Emm_V5_App_ClampRpm(v0);
            int16_t tilt_rpm = Emm_V5_App_ClampRpm(v1);
            char ack[EMM_GIMBAL_USB_TX_BUF_SIZE] = {0};

            Emm_V5_App_SetVelocity(pan_rpm, tilt_rpm);
            (void)snprintf(ack, sizeof(ack), "OK VEL %d %d", pan_rpm, tilt_rpm);
            Emm_V5_App_USB_Reply(ack);
        } else {
            Emm_V5_App_USB_Reply("ERR VEL FORMAT");
        }
        return;
    }

    if (strcmp(cmd, "STOP") == 0) {
        Emm_V5_App_Stop();
        Emm_V5_App_USB_Reply("OK STOP");
        return;
    }

    Emm_V5_App_USB_Reply("ERR UNKNOWN CMD");
}

void Emm_V5_App_Init(void)
{
    Emm_V5_App_Stop();
    Emm_V5_App_SetEnable(EMM_GIMBAL_ENABLE_ON_BOOT);
}

void Emm_V5_App_SetEnable(bool enable)
{
    Emm_V5_En_Control(EMM_GIMBAL_PAN_MOTOR_ADDR, enable, EMM_GIMBAL_SYNC_FLAG);
    Emm_V5_En_Control(EMM_GIMBAL_TILT_MOTOR_ADDR, enable, EMM_GIMBAL_SYNC_FLAG);
}

void Emm_V5_App_SetVelocity(int16_t pan_rpm, int16_t tilt_rpm)
{
    Emm_V5_App_SendAxisVelocity(EMM_GIMBAL_PAN_MOTOR_ADDR,
                                Emm_V5_App_ClampRpm(pan_rpm),
                                (EMM_GIMBAL_PAN_DIR_INVERT != 0U));
    Emm_V5_App_SendAxisVelocity(EMM_GIMBAL_TILT_MOTOR_ADDR,
                                Emm_V5_App_ClampRpm(tilt_rpm),
                                (EMM_GIMBAL_TILT_DIR_INVERT != 0U));
}

void Emm_V5_App_Stop(void)
{
    Emm_V5_Stop_Now(EMM_GIMBAL_PAN_MOTOR_ADDR, EMM_GIMBAL_SYNC_FLAG);
    Emm_V5_Stop_Now(EMM_GIMBAL_TILT_MOTOR_ADDR, EMM_GIMBAL_SYNC_FLAG);
}

void Emm_V5_App_USB_CDC_Input(const uint8_t *data, uint32_t len)
{
    uint32_t i;

    if ((data == NULL) || (len == 0U)) {
        return;
    }

    for (i = 0U; i < len; ++i) {
        const char ch = (char)data[i];

        if ((ch == '\r') || (ch == '\n')) {
            if (s_usb_cmd_len > 0U) {
                s_usb_cmd_buf[s_usb_cmd_len] = '\0';
                Emm_V5_App_ProcessCmd(s_usb_cmd_buf);
                s_usb_cmd_len = 0U;
            }
            continue;
        }

        if (((uint8_t)ch < 0x20U) || ((uint8_t)ch > 0x7EU)) {
            continue;
        }

        if (s_usb_cmd_len < (EMM_GIMBAL_USB_CMD_BUF_SIZE - 1U)) {
            s_usb_cmd_buf[s_usb_cmd_len++] = ch;
            continue;
        }

        s_usb_cmd_len = 0U;
        Emm_V5_App_USB_Reply("ERR LINE TOO LONG");
    }
}
