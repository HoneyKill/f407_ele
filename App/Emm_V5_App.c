#include "Emm_V5_App.h"

#include "Emm_V5.h"
#include "Uart1_Dma.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <string.h>

#if (EMM_GIMBAL_CMD_QUEUE_SIZE < 2U)
#error "EMM_GIMBAL_CMD_QUEUE_SIZE must be at least 2"
#endif

static char s_usb_cmd_buf[EMM_GIMBAL_USB_CMD_BUF_SIZE] = {0};
static uint16_t s_usb_cmd_len = 0U;

typedef enum {
    EMM_PENDING_NONE = 0,
    EMM_PENDING_VEL,
    EMM_PENDING_EN,
    EMM_PENDING_STOP,
} EmmPendingType_t;

typedef struct {
    EmmPendingType_t type;
    int16_t pan_rpm;
    int16_t tilt_rpm;
    uint8_t enable;
} EmmPendingCmd_t;

typedef struct {
    EmmPendingCmd_t buf[EMM_GIMBAL_CMD_QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint32_t drop_count;
} EmmPendingQueue_t;

typedef struct {
    volatile uint8_t active;
    volatile uint8_t stage; /* 0: axis1, 1: axis2 */
    volatile uint32_t due_tick;
    EmmPendingCmd_t cmd;
} EmmExecState_t;

typedef struct {
    uint8_t frame[EMM_GIMBAL_VOFA_FRAME_SIZE];
    uint8_t len;
} EmmVofaParser_t;

typedef struct {
    float corner_x;
    float corner_y;
    float vector_x;
    float vector_y;
    float gimbal_state;
    uint32_t last_vector_x_tick;
    uint32_t last_vector_y_tick;
    uint32_t last_ctrl_tick;
    int16_t last_pan_cmd;
    int16_t last_tilt_cmd;
    uint8_t has_vector_x;
    uint8_t has_vector_y;
} EmmVofaState_t;

static EmmPendingQueue_t s_pending_queue = {0};
static EmmExecState_t s_exec_state = {0};
static EmmVofaParser_t s_vofa_parser = {0};
static EmmVofaState_t s_vofa_state = {0};

static uint32_t Emm_V5_App_Lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Emm_V5_App_Unlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint8_t Emm_V5_App_QueueNext(uint8_t idx)
{
    idx++;
    if (idx >= EMM_GIMBAL_CMD_QUEUE_SIZE) {
        idx = 0U;
    }
    return idx;
}

static void Emm_V5_App_QueueClearLocked(void)
{
    s_pending_queue.tail = s_pending_queue.head;
}

static uint8_t Emm_V5_App_QueueDepthLocked(void)
{
    if (s_pending_queue.head >= s_pending_queue.tail) {
        return (uint8_t)(s_pending_queue.head - s_pending_queue.tail);
    }
    return (uint8_t)(EMM_GIMBAL_CMD_QUEUE_SIZE + s_pending_queue.head - s_pending_queue.tail);
}

static void Emm_V5_App_QueuePushLocked(const EmmPendingCmd_t *cmd)
{
    uint8_t head = s_pending_queue.head;
    uint8_t next = Emm_V5_App_QueueNext(head);

    if (next == s_pending_queue.tail) {
        s_pending_queue.tail = Emm_V5_App_QueueNext(s_pending_queue.tail);
        ++s_pending_queue.drop_count;
    }

    s_pending_queue.buf[head] = *cmd;
    s_pending_queue.head = next;
}

static uint8_t Emm_V5_App_QueuePopLocked(EmmPendingCmd_t *cmd)
{
    uint8_t tail = s_pending_queue.tail;

    if (tail == s_pending_queue.head) {
        return 0U;
    }

    *cmd = s_pending_queue.buf[tail];
    s_pending_queue.tail = Emm_V5_App_QueueNext(tail);
    return 1U;
}

static void Emm_V5_App_USB_Reply(const char *msg)
{
    static uint8_t tx_buf[EMM_GIMBAL_USB_TX_BUF_SIZE] = {0};
    size_t msg_len = strnlen(msg, EMM_GIMBAL_USB_TX_BUF_SIZE - 3U);

    memcpy(tx_buf, msg, msg_len);
    tx_buf[msg_len++] = '\r';
    tx_buf[msg_len++] = '\n';

    (void)CDC_Transmit_FS(tx_buf, (uint16_t)msg_len);
}

static void Emm_V5_App_BuildDebugText(char *out, size_t out_size)
{
#if (EMM_GIMBAL_USB_DEBUG_REPLY_ENABLE != 0U)
    UART1_DmaDebugInfo_t dbg = {0};
    uint8_t app_depth;
    uint8_t app_active;
    uint8_t app_stage;
    uint32_t app_drop;
    uint32_t primask;

    UART1_DmaGetDebugInfo(&dbg);

    primask = Emm_V5_App_Lock();
    app_depth = Emm_V5_App_QueueDepthLocked();
    app_active = s_exec_state.active;
    app_stage = s_exec_state.stage;
    app_drop = s_pending_queue.drop_count;
    Emm_V5_App_Unlock(primask);

    (void)snprintf(out,
                   out_size,
                   "DBG aq%u ad%lu as%u/%u q%lu b%lu e%lu s%lu/%lu c%lu r%u ov%lu L%u:%02X%02X%02X%02X R:%02X%02X%02X%02X",
                   (unsigned)app_depth,
                   (unsigned long)app_drop,
                   (unsigned)app_active,
                   (unsigned)app_stage,
                   (unsigned long)dbg.tx_enqueue_ok,
                   (unsigned long)dbg.tx_enqueue_busy,
                   (unsigned long)dbg.tx_enqueue_error,
                   (unsigned long)dbg.tx_start_ok,
                   (unsigned long)dbg.tx_start_fail,
                   (unsigned long)dbg.tx_complete,
                   (unsigned)dbg.rx_available,
                   (unsigned long)dbg.rx_overwrite,
                   (unsigned)dbg.last_tx_len,
                   dbg.last_tx_data[0],
                   dbg.last_tx_data[1],
                   dbg.last_tx_data[2],
                   dbg.last_tx_data[3],
                   dbg.last_rx_data[0],
                   dbg.last_rx_data[1],
                   dbg.last_rx_data[2],
                   dbg.last_rx_data[3]);
#else
    (void)out_size;
    out[0] = '\0';
#endif
}

static void Emm_V5_App_ReplyWithDebug(const char *prefix)
{
#if (EMM_GIMBAL_USB_DEBUG_REPLY_ENABLE != 0U)
    char msg[EMM_GIMBAL_USB_TX_BUF_SIZE] = {0};
    char dbg[EMM_GIMBAL_USB_TX_BUF_SIZE] = {0};
    size_t used = 0U;
    size_t n = 0U;

    Emm_V5_App_BuildDebugText(dbg, sizeof(dbg));

    n = strnlen(prefix, sizeof(msg) - 1U);
    memcpy(msg, prefix, n);
    used = n;

    if (used < (sizeof(msg) - 1U)) {
        msg[used++] = ' ';
    }
    if (used < (sizeof(msg) - 1U)) {
        msg[used++] = '|';
    }
    if (used < (sizeof(msg) - 1U)) {
        msg[used++] = ' ';
    }

    n = strnlen(dbg, (sizeof(msg) - 1U) - used);
    memcpy(&msg[used], dbg, n);
    msg[used + n] = '\0';

    Emm_V5_App_USB_Reply(msg);
#else
    Emm_V5_App_USB_Reply(prefix);
#endif
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

static float Emm_V5_App_AbsFloat(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float Emm_V5_App_DecodeLEFloat(const uint8_t *data4)
{
    union {
        uint8_t b[4];
        float f;
    } u;

    u.b[0] = data4[0];
    u.b[1] = data4[1];
    u.b[2] = data4[2];
    u.b[3] = data4[3];

    return u.f;
}

static int16_t Emm_V5_App_FloatToRpm(float rpm)
{
    if (rpm > (float)EMM_GIMBAL_MAX_RPM) {
        rpm = (float)EMM_GIMBAL_MAX_RPM;
    } else if (rpm < -(float)EMM_GIMBAL_MAX_RPM) {
        rpm = -(float)EMM_GIMBAL_MAX_RPM;
    }

    return (rpm >= 0.0f) ? (int16_t)(rpm + 0.5f) : (int16_t)(rpm - 0.5f);
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

static bool Emm_V5_App_MapEnableState(bool enable)
{
#if (EMM_GIMBAL_ENABLE_ACTIVE_HIGH != 0U)
    return enable;
#else
    return !enable;
#endif
}

static void Emm_V5_App_SetVelocityAxis1(int16_t rpm)
{
#if (EMM_GIMBAL_PAN_AXIS_ENABLE != 0U)
    Emm_V5_App_SendAxisVelocity(EMM_GIMBAL_PAN_MOTOR_ADDR,
                                Emm_V5_App_ClampRpm(rpm),
                                (EMM_GIMBAL_PAN_DIR_INVERT != 0U));
#else
    (void)rpm;
#endif
}

static void Emm_V5_App_SetVelocityAxis2(int16_t rpm)
{
#if (EMM_GIMBAL_TILT_AXIS_ENABLE != 0U)
    Emm_V5_App_SendAxisVelocity(EMM_GIMBAL_TILT_MOTOR_ADDR,
                                Emm_V5_App_ClampRpm(rpm),
                                (EMM_GIMBAL_TILT_DIR_INVERT != 0U));
#else
    (void)rpm;
#endif
}

static void Emm_V5_App_QueueVel(int16_t pan_rpm, int16_t tilt_rpm)
{
    EmmPendingCmd_t cmd;
    uint32_t primask;

    cmd.type = EMM_PENDING_VEL;
    cmd.pan_rpm = Emm_V5_App_ClampRpm(pan_rpm);
    cmd.tilt_rpm = Emm_V5_App_ClampRpm(tilt_rpm);
    cmd.enable = 0U;

    primask = Emm_V5_App_Lock();
    Emm_V5_App_QueuePushLocked(&cmd);
    Emm_V5_App_Unlock(primask);
}

static void Emm_V5_App_QueueEnable(bool enable)
{
    EmmPendingCmd_t cmd;
    uint32_t primask;

    cmd.type = EMM_PENDING_EN;
    cmd.pan_rpm = 0;
    cmd.tilt_rpm = 0;
    cmd.enable = (uint8_t)(enable ? 1U : 0U);

    primask = Emm_V5_App_Lock();
    Emm_V5_App_QueueClearLocked();
    s_exec_state.active = 0U;
    s_exec_state.stage = 0U;
    s_exec_state.due_tick = 0U;
    Emm_V5_App_QueuePushLocked(&cmd);
    Emm_V5_App_Unlock(primask);
}

static void Emm_V5_App_QueueStop(void)
{
    EmmPendingCmd_t cmd;
    uint32_t primask;

    cmd.type = EMM_PENDING_STOP;
    cmd.pan_rpm = 0;
    cmd.tilt_rpm = 0;
    cmd.enable = 0U;

    primask = Emm_V5_App_Lock();
    Emm_V5_App_QueueClearLocked();
    s_exec_state.active = 0U;
    s_exec_state.stage = 0U;
    s_exec_state.due_tick = 0U;
    Emm_V5_App_QueuePushLocked(&cmd);
    Emm_V5_App_Unlock(primask);
}

static void Emm_V5_App_HandleVofaFrame(const uint8_t *frame)
{
#if (EMM_GIMBAL_VOFA_ENABLE != 0U)
    const uint8_t command = frame[3];
    const float data_value = Emm_V5_App_DecodeLEFloat(&frame[4]);
    const uint32_t now = HAL_GetTick();

    switch (command) {
    case EMM_GIMBAL_VOFA_CMD_CORNER_X:
        s_vofa_state.corner_x = data_value;
        break;

    case EMM_GIMBAL_VOFA_CMD_CORNER_Y:
        s_vofa_state.corner_y = data_value;
        break;

    case EMM_GIMBAL_VOFA_CMD_STATE:
        s_vofa_state.gimbal_state = data_value;
        break;

    case EMM_GIMBAL_VOFA_CMD_VECTOR_X:
        s_vofa_state.vector_x = data_value;
        s_vofa_state.has_vector_x = 1U;
        s_vofa_state.last_vector_x_tick = now;
        break;

    case EMM_GIMBAL_VOFA_CMD_VECTOR_Y:
        s_vofa_state.vector_y = data_value;
        s_vofa_state.has_vector_y = 1U;
        s_vofa_state.last_vector_y_tick = now;
        break;

    default:
        break;
    }
#else
    (void)frame;
#endif
}

static void Emm_V5_App_VofaFeedByte(uint8_t byte)
{
#if (EMM_GIMBAL_VOFA_ENABLE != 0U)
    uint8_t len = s_vofa_parser.len;

    if (len == 0U) {
        if (byte == EMM_GIMBAL_VOFA_HEADER_0) {
            s_vofa_parser.frame[0] = byte;
            s_vofa_parser.len = 1U;
        }
        return;
    }

    if (len == 1U) {
        if (byte == EMM_GIMBAL_VOFA_HEADER_1) {
            s_vofa_parser.frame[1] = byte;
            s_vofa_parser.len = 2U;
        } else if (byte == EMM_GIMBAL_VOFA_HEADER_0) {
            s_vofa_parser.frame[0] = byte;
            s_vofa_parser.len = 1U;
        } else {
            s_vofa_parser.len = 0U;
        }
        return;
    }

    if (len == 2U) {
        if (byte == EMM_GIMBAL_VOFA_HEADER_2) {
            s_vofa_parser.frame[2] = byte;
            s_vofa_parser.len = 3U;
        } else if (byte == EMM_GIMBAL_VOFA_HEADER_0) {
            s_vofa_parser.frame[0] = byte;
            s_vofa_parser.len = 1U;
        } else {
            s_vofa_parser.len = 0U;
        }
        return;
    }

    if (len < EMM_GIMBAL_VOFA_FRAME_SIZE) {
        s_vofa_parser.frame[len] = byte;
        ++len;
        s_vofa_parser.len = len;
    } else {
        s_vofa_parser.len = 0U;
        return;
    }

    if (len >= EMM_GIMBAL_VOFA_FRAME_SIZE) {
        Emm_V5_App_HandleVofaFrame(s_vofa_parser.frame);
        s_vofa_parser.len = 0U;
    }
#else
    (void)byte;
#endif
}

static void Emm_V5_App_PollVofaUartInput(void)
{
#if (EMM_GIMBAL_VOFA_ENABLE != 0U)
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
        uint8_t byte = (uint8_t)(huart3.Instance->DR & 0xFFU);
        Emm_V5_App_VofaFeedByte(byte);
    }

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
    }
#endif
}

static void Emm_V5_App_VofaControlTask(uint32_t now)
{
#if (EMM_GIMBAL_VOFA_ENABLE != 0U)
    uint32_t latest_tick;
    float vector_x;
    float vector_y;
    int16_t pan_cmd = 0;
    int16_t tilt_cmd = 0;

#if (EMM_GIMBAL_VOFA_CTRL_PERIOD_MS > 0U)
    if ((uint32_t)(now - s_vofa_state.last_ctrl_tick) < EMM_GIMBAL_VOFA_CTRL_PERIOD_MS) {
        return;
    }
#endif
    s_vofa_state.last_ctrl_tick = now;

    if ((s_vofa_state.has_vector_x == 0U) || (s_vofa_state.has_vector_y == 0U)) {
        return;
    }

    latest_tick = s_vofa_state.last_vector_x_tick;
    if ((int32_t)(s_vofa_state.last_vector_y_tick - latest_tick) > 0) {
        latest_tick = s_vofa_state.last_vector_y_tick;
    }

    if ((uint32_t)(now - latest_tick) > EMM_GIMBAL_VOFA_TIMEOUT_MS) {
        pan_cmd = 0;
        tilt_cmd = 0;
    } else if ((Emm_V5_App_AbsFloat(s_vofa_state.vector_x) >= EMM_GIMBAL_VOFA_NO_TARGET_ABS_MIN) ||
               (Emm_V5_App_AbsFloat(s_vofa_state.vector_y) >= EMM_GIMBAL_VOFA_NO_TARGET_ABS_MIN)) {
        pan_cmd = 0;
        tilt_cmd = 0;
    } else {
        vector_x = s_vofa_state.vector_x;
        vector_y = s_vofa_state.vector_y;

        if (Emm_V5_App_AbsFloat(vector_x) < EMM_GIMBAL_VOFA_VECTOR_DEADBAND) {
            vector_x = 0.0f;
        }
        if (Emm_V5_App_AbsFloat(vector_y) < EMM_GIMBAL_VOFA_VECTOR_DEADBAND) {
            vector_y = 0.0f;
        }

#if (EMM_GIMBAL_VOFA_PAN_INVERT != 0U)
        vector_x = -vector_x;
#endif
#if (EMM_GIMBAL_VOFA_TILT_INVERT != 0U)
        vector_y = -vector_y;
#endif

        pan_cmd = Emm_V5_App_FloatToRpm(vector_x * EMM_GIMBAL_VOFA_PAN_GAIN);
        tilt_cmd = Emm_V5_App_FloatToRpm(vector_y * EMM_GIMBAL_VOFA_TILT_GAIN);
    }

    if ((pan_cmd == s_vofa_state.last_pan_cmd) && (tilt_cmd == s_vofa_state.last_tilt_cmd)) {
        return;
    }

    s_vofa_state.last_pan_cmd = pan_cmd;
    s_vofa_state.last_tilt_cmd = tilt_cmd;
    Emm_V5_App_SetVelocity(pan_cmd, tilt_cmd);
#else
    (void)now;
#endif
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
        Emm_V5_App_USB_Reply("OK CMD: HELP | EN <0|1> | VEL <PAN_RPM> <TILT_RPM> | VEL1 <RPM> | VEL2 <RPM> | STOP | STAT");
        return;
    }

    if (strcmp(cmd, "STAT") == 0) {
        Emm_V5_App_ReplyWithDebug("OK STAT");
        return;
    }

    if (strcmp(cmd, "EN") == 0) {
        if (sscanf(line, "%*s %ld", &v0) == 1) {
            Emm_V5_App_SetEnable(v0 != 0);
            Emm_V5_App_ReplyWithDebug((v0 != 0) ? "OK EN 1" : "OK EN 0");
        } else {
            Emm_V5_App_USB_Reply("ERR EN FORMAT");
        }
        return;
    }

    if (strcmp(cmd, "VEL") == 0) {
        if (sscanf(line, "%*s %ld %ld", &v0, &v1) == 2) {
            int16_t pan_rpm = Emm_V5_App_ClampRpm(v0);
            int16_t tilt_rpm = Emm_V5_App_ClampRpm(v1);
            char ack[64] = {0};

            Emm_V5_App_SetVelocity(pan_rpm, tilt_rpm);
            (void)snprintf(ack, sizeof(ack), "OK VEL %d %d", pan_rpm, tilt_rpm);
            Emm_V5_App_ReplyWithDebug(ack);
        } else {
            Emm_V5_App_USB_Reply("ERR VEL FORMAT");
        }
        return;
    }

    if (strcmp(cmd, "VEL1") == 0) {
        if (sscanf(line, "%*s %ld", &v0) == 1) {
            int16_t rpm = Emm_V5_App_ClampRpm(v0);
            char ack[64] = {0};

            Emm_V5_App_SetVelocityAxis1(rpm);
            (void)snprintf(ack, sizeof(ack), "OK VEL1 %d", rpm);
            Emm_V5_App_ReplyWithDebug(ack);
        } else {
            Emm_V5_App_USB_Reply("ERR VEL1 FORMAT");
        }
        return;
    }

    if (strcmp(cmd, "VEL2") == 0) {
        if (sscanf(line, "%*s %ld", &v0) == 1) {
            int16_t rpm = Emm_V5_App_ClampRpm(v0);
            char ack[64] = {0};

            Emm_V5_App_SetVelocityAxis2(rpm);
            (void)snprintf(ack, sizeof(ack), "OK VEL2 %d", rpm);
            Emm_V5_App_ReplyWithDebug(ack);
        } else {
            Emm_V5_App_USB_Reply("ERR VEL2 FORMAT");
        }
        return;
    }

    if (strcmp(cmd, "STOP") == 0) {
        Emm_V5_App_Stop();
        Emm_V5_App_ReplyWithDebug("OK STOP");
        return;
    }

    Emm_V5_App_USB_Reply("ERR UNKNOWN CMD");
}

void Emm_V5_App_Init(void)
{
    uint32_t primask = Emm_V5_App_Lock();

    memset(&s_pending_queue, 0, sizeof(s_pending_queue));
    memset(&s_exec_state, 0, sizeof(s_exec_state));
    memset(&s_vofa_parser, 0, sizeof(s_vofa_parser));
    memset(&s_vofa_state, 0, sizeof(s_vofa_state));

    Emm_V5_App_Unlock(primask);

#if (EMM_GIMBAL_INIT_SEND_STOP != 0U)
    Emm_V5_App_Stop();
#endif
#if (EMM_GIMBAL_INIT_SEND_ENABLE != 0U)
    Emm_V5_App_SetEnable(EMM_GIMBAL_ENABLE_ON_BOOT);
#endif
}

void Emm_V5_App_Task(void)
{
    EmmPendingCmd_t cmd = {0};
    uint8_t stage = 0U;
    uint32_t now;
    uint32_t axis_gap_ms;
    uint32_t primask;

    Emm_V5_App_PollVofaUartInput();
    Emm_V5_App_VofaControlTask(HAL_GetTick());

    primask = Emm_V5_App_Lock();
    now = HAL_GetTick();
    axis_gap_ms = EMM_GIMBAL_AXIS_CMD_GAP_MS;
    if (axis_gap_ms < EMM_GIMBAL_AXIS_CMD_MIN_GAP_MS) {
        axis_gap_ms = EMM_GIMBAL_AXIS_CMD_MIN_GAP_MS;
    }

    if (s_exec_state.active == 0U) {
        if (Emm_V5_App_QueuePopLocked(&s_exec_state.cmd) == 0U) {
            Emm_V5_App_Unlock(primask);
            return;
        }
        s_exec_state.active = 1U;
        s_exec_state.stage = 0U;
        s_exec_state.due_tick = now;
    }

    if ((int32_t)(now - s_exec_state.due_tick) < 0) {
        Emm_V5_App_Unlock(primask);
        return;
    }

    cmd = s_exec_state.cmd;
    stage = s_exec_state.stage;

    if (stage == 0U) {
        s_exec_state.stage = 1U;
        s_exec_state.due_tick = now + axis_gap_ms;
    } else {
        s_exec_state.active = 0U;
    }

    Emm_V5_App_Unlock(primask);

    if ((cmd.type != EMM_PENDING_VEL) && (cmd.type != EMM_PENDING_EN) && (cmd.type != EMM_PENDING_STOP)) {
        return;
    }

    if (stage == 0U) {
        if (cmd.type == EMM_PENDING_VEL) {
            Emm_V5_App_SetVelocityAxis1(cmd.pan_rpm);
        } else if (cmd.type == EMM_PENDING_EN) {
#if (EMM_GIMBAL_PAN_AXIS_ENABLE != 0U)
            Emm_V5_En_Control(EMM_GIMBAL_PAN_MOTOR_ADDR,
                              Emm_V5_App_MapEnableState(cmd.enable != 0U),
                              EMM_GIMBAL_SYNC_FLAG);
#endif
        } else {
#if (EMM_GIMBAL_PAN_AXIS_ENABLE != 0U)
            Emm_V5_Stop_Now(EMM_GIMBAL_PAN_MOTOR_ADDR, EMM_GIMBAL_SYNC_FLAG);
#endif
        }
    } else {
        if (cmd.type == EMM_PENDING_VEL) {
            Emm_V5_App_SetVelocityAxis2(cmd.tilt_rpm);
        } else if (cmd.type == EMM_PENDING_EN) {
#if (EMM_GIMBAL_TILT_AXIS_ENABLE != 0U)
            Emm_V5_En_Control(EMM_GIMBAL_TILT_MOTOR_ADDR,
                              Emm_V5_App_MapEnableState(cmd.enable != 0U),
                              EMM_GIMBAL_SYNC_FLAG);
#endif
        } else {
#if (EMM_GIMBAL_TILT_AXIS_ENABLE != 0U)
            Emm_V5_Stop_Now(EMM_GIMBAL_TILT_MOTOR_ADDR, EMM_GIMBAL_SYNC_FLAG);
#endif
        }
    }
}

void Emm_V5_App_SetEnable(bool enable)
{
    Emm_V5_App_QueueEnable(enable);
}

void Emm_V5_App_SetVelocity(int16_t pan_rpm, int16_t tilt_rpm)
{
    Emm_V5_App_QueueVel(pan_rpm, tilt_rpm);
}

void Emm_V5_App_Stop(void)
{
    Emm_V5_App_QueueStop();
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
