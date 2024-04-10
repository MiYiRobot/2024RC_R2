#ifndef __VESC_CAN_H
#define __VESC_CAN_H
#include "main.h"
#include "stdbool.h"

#ifndef wtrcfg_VESC_COMMAND_DUTY_MAX
#define wtrcfg_VESC_COMMAND_DUTY_MAX 100
#endif 
#ifndef wtrcfg_VESC_COMMAND_CURRENT_MAX
#define wtrcfg_VESC_COMMAND_CURRENT_MAX 10
#endif 
#ifndef wtrcfg_VESC_COMMAND_POS_MAX
#define wtrcfg_VESC_COMMAND_POS_MAX 360
#endif 
#ifndef wtrcfg_VESC_COMMAND_ERPM_MAX
#define wtrcfg_VESC_COMMAND_ERPM_MAX 20000
#endif 
#ifndef wtrcfg_VESC_UART_TIMEOUT 
#define wtrcfg_VESC_UART_TIMEOUT 0xff
#endif



//VESC状态参数结构体
typedef uint32_t systime_t;

typedef struct {
	int id;
	systime_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;

typedef struct {
	int id;
	systime_t rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;

typedef struct {
	int id;
	systime_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;

typedef struct {
	int id;
	systime_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;

typedef struct {
	int id;
	systime_t rx_time;
	float v_in;
	int32_t tacho_value;
} can_status_msg_5;


void can_get_data(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);


//can命令枚举
typedef enum {
	CAN_PACKET_SET_DUTY						= 0,
	CAN_PACKET_SET_CURRENT					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE			= 2,
	CAN_PACKET_SET_RPM						= 3,
	CAN_PACKET_SET_POS						= 4,
	CAN_PACKET_FILL_RX_BUFFER				= 5,
	CAN_PACKET_FILL_RX_BUFFER_LONG			= 6,
	CAN_PACKET_PROCESS_RX_BUFFER			= 7,
	CAN_PACKET_PROCESS_SHORT_BUFFER			= 8,
	CAN_PACKET_STATUS						= 9,
	CAN_PACKET_SET_CURRENT_REL				= 10,
	CAN_PACKET_SET_CURRENT_BRAKE_REL		= 11,
	CAN_PACKET_SET_CURRENT_HANDBRAKE		= 12,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL	= 13,
	CAN_PACKET_STATUS_2						= 14,
	CAN_PACKET_STATUS_3						= 15,
	CAN_PACKET_STATUS_4						= 16,
	CAN_PACKET_PING							= 17,
	CAN_PACKET_PONG							= 18,
	CAN_PACKET_DETECT_APPLY_ALL_FOC			= 19,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES		= 20,
	CAN_PACKET_CONF_CURRENT_LIMITS			= 21,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS	= 22,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN		= 23,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN	= 24,
	CAN_PACKET_CONF_FOC_ERPMS				= 25,
	CAN_PACKET_CONF_STORE_FOC_ERPMS			= 26,
	CAN_PACKET_STATUS_5						= 27,
	CAN_PACKET_POLL_TS5700N8501_STATUS		= 28,
	CAN_PACKET_CONF_BATTERY_CUT				= 29,
	CAN_PACKET_CONF_STORE_BATTERY_CUT		= 30,
	CAN_PACKET_SHUTDOWN						= 31,
	CAN_PACKET_IO_BOARD_ADC_1_TO_4			= 32,
	CAN_PACKET_IO_BOARD_ADC_5_TO_8			= 33,
	CAN_PACKET_IO_BOARD_ADC_9_TO_12			= 34,
	CAN_PACKET_IO_BOARD_DIGITAL_IN			= 35,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL	= 36,
	CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM		= 37,
	CAN_PACKET_BMS_V_TOT					= 38,
	CAN_PACKET_BMS_I						= 39,
	CAN_PACKET_BMS_AH_WH					= 40,
	CAN_PACKET_BMS_V_CELL					= 41,
	CAN_PACKET_BMS_BAL						= 42,
	CAN_PACKET_BMS_TEMPS					= 43,
	CAN_PACKET_BMS_HUM						= 44,
	CAN_PACKET_BMS_SOC_SOH_TEMP_STAT		= 45,
	CAN_PACKET_PSW_STAT						= 46,
	CAN_PACKET_PSW_SWITCH					= 47,
	CAN_PACKET_BMS_HW_DATA_1				= 48,
	CAN_PACKET_BMS_HW_DATA_2				= 49,
	CAN_PACKET_BMS_HW_DATA_3				= 50,
	CAN_PACKET_BMS_HW_DATA_4				= 51,
	CAN_PACKET_BMS_HW_DATA_5				= 52,
	CAN_PACKET_BMS_AH_WH_CHG_TOTAL			= 53,
	CAN_PACKET_BMS_AH_WH_DIS_TOTAL			= 54,
	CAN_PACKET_UPDATE_PID_POS_OFFSET		= 55,
	CAN_PACKET_POLL_ROTOR_POS				= 56,
	CAN_PACKET_NOTIFY_BOOT					= 57,
	CAN_PACKET_STATUS_6						= 58,
	CAN_PACKET_GNSS_TIME					= 59,
	CAN_PACKET_GNSS_LAT						= 60,
	CAN_PACKET_GNSS_LON						= 61,
	CAN_PACKET_GNSS_ALT_SPEED_HDOP			= 62
}
CAN_PACKET_ID;




//限幅函数,高级操作
void assert_param_pos(float *pos);
void assert_param_rpm(float *rpm);
void assert_param_current(float *current);
void assert_param_duty(float *duty);




//buffer函数
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);




//can通信指令，向VESC发指令
void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len);	   //拓展帧发送函数
void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len);		   //标准帧发送函数
void comm_can_set_duty(uint8_t controller_id, float duty);					   //设置对应ID占空比
void comm_can_set_current(uint8_t controller_id, float current);			   //设置对应ID电流环
void comm_can_set_current_brake(uint8_t controller_id, float current);		   //设置对应ID刹车电流
void comm_can_set_rpm(uint8_t controller_id, float rpm);					   //设置对应ID转速环
void comm_can_set_pos(uint8_t controller_id, float pos);					   //设置对应ID位置环
void comm_can_set_current_rel(uint8_t controller_id, float current_rel);	   //设置电流限幅
void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel); //设置刹车电流限幅
void comm_can_set_handbrake(uint8_t controller_id, float current);			   //设置刹车电流
void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel);	   //设置刹车电流限幅
void comm_can_conf_current_limits(uint8_t controller_id, bool store, float min, float max);
void comm_can_conf_current_limits_in(uint8_t controller_id, bool store, float min, float max);


void VESC_Init(void);

extern VESC_MOTOR_INFO VESC_MOTO_INFO[6];

#endif


