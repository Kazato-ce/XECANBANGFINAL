#include "state_machine.h"
#include "stepper_tmc2208.h"
#include "balance.h"
#include "config.h"
#include <math.h>

static VehicleState_t s_state = STATE_BOOT;
static FaultReason_t  s_fault_reason = FAULT_NONE;

void sm_init(void)
{
    s_state = STATE_BOOT;
    s_fault_reason = FAULT_NONE;
    stepper_stop(); /* PA0 = HIGH */
}

void sm_update(float pitch_deg, bool radio_connected, bool arm_cmd, bool disarm_cmd, bool estop_cmd, uint32_t imu_errors)
{
    /* Emergency stop hoặc lệnh DISARM luôn được ưu tiên */
    if (estop_cmd) {
        stepper_stop();
        s_fault_reason = FAULT_ESTOP_CMD;
        s_state = STATE_FAULT;
        return;
    }

    /* Kiểm tra lỗi IMU liên tiếp */
    if (imu_errors >= IMU_MAX_CONSECUTIVE_ERRORS) {
        stepper_stop();
        s_fault_reason = FAULT_IMU_ERROR;
        s_state = STATE_FAULT;
        return;
    }

    switch (s_state) {
        case STATE_BOOT:
            stepper_stop();
            /* Boot xong chuyển sang CALIBRATING */
            s_state = STATE_CALIBRATING;
            break;

        case STATE_CALIBRATING:
            stepper_stop();
            /* Sau khi calibration hoàn tất từ app.c, sẽ chuyển sang DISARMED */
            break;

        case STATE_DISARMED:
            stepper_stop();
            s_fault_reason = FAULT_NONE;

            /* Điều kiện để ARM:
             * 1. Xe đang dựng thẳng (pitch trong ngưỡng ARM_UPRIGHT_ANGLE_DEG)
             * 2. Có lệnh ARM hợp lệ
             * 3. Sóng điều khiển radio hoạt động */
            if (arm_cmd && radio_connected && (fabsf(pitch_deg) <= ARM_UPRIGHT_ANGLE_DEG)) {
                balance_reset();
                stepper_enable(true); /* PA0 = LOW: Bật driver */
                s_state = STATE_ARMED;
            }
            break;

        case STATE_ARMED:
            /* Kiểm tra điều kiện ngã (Failsafe 1) */
            if (fabsf(pitch_deg) > FALL_ANGLE_THRESHOLD_DEG) {
                stepper_stop();
                s_fault_reason = FAULT_FALL_OVER;
                s_state = STATE_FAULT;
                break;
            }

            /* Kiểm tra mất sóng radio quá timeout (Failsafe 2) */
            if (!radio_connected) {
                stepper_stop();
                s_fault_reason = FAULT_RADIO_TIMEOUT;
                s_state = STATE_FAULT;
                break;
            }

            /* Người dùng gửi lệnh DISARM chủ động */
            if (disarm_cmd) {
                stepper_stop();
                s_state = STATE_DISARMED;
                break;
            }
            break;

        case STATE_FAULT:
            stepper_stop(); /* Đảm bảo driver luôn tắt */
            /* Thoát FAULT khi người dùng chủ động gửi lệnh DISARM và xe không còn trong trạng thái nguy hiểm */
            if (disarm_cmd && (fabsf(pitch_deg) <= FALL_ANGLE_THRESHOLD_DEG)) {
                s_fault_reason = FAULT_NONE;
                s_state = STATE_DISARMED;
            }
            break;
    }
}

VehicleState_t sm_get_state(void)
{
    return s_state;
}

FaultReason_t sm_get_fault_reason(void)
{
    return s_fault_reason;
}

const char* sm_get_state_str(VehicleState_t st)
{
    switch (st) {
        case STATE_BOOT:        return "BOOT";
        case STATE_CALIBRATING: return "CALIBRATING";
        case STATE_DISARMED:    return "DISARMED";
        case STATE_ARMED:       return "ARMED";
        case STATE_FAULT:       return "FAULT";
        default:                return "UNKNOWN";
    }
}
