/*
 * stepdown.h
 *
 * Created: 3-3-2022 13:39:44
 *  Author: Milo
 */ 


#ifndef STEPDOWN_H_
#define STEPDOWN_H_

#define PORT_EVCTRL_EVACT1_OUT      (PORT_EVCTRL_EVACT0_OUT_Val    << PORT_EVCTRL_EVACT1_Pos)
#define PORT_EVCTRL_EVACT1_SET      (PORT_EVCTRL_EVACT0_SET_Val    << PORT_EVCTRL_EVACT1_Pos)
#define PORT_EVCTRL_EVACT1_CLR      (PORT_EVCTRL_EVACT0_CLR_Val    << PORT_EVCTRL_EVACT1_Pos)
#define PORT_EVCTRL_EVACT1_TGL      (PORT_EVCTRL_EVACT0_TGL_Val    << PORT_EVCTRL_EVACT1_Pos)

#define PORT_EVCTRL_EVACT2_OUT      (PORT_EVCTRL_EVACT0_OUT_Val    << PORT_EVCTRL_EVACT2_Pos)
#define PORT_EVCTRL_EVACT2_SET      (PORT_EVCTRL_EVACT0_SET_Val    << PORT_EVCTRL_EVACT2_Pos)
#define PORT_EVCTRL_EVACT2_CLR      (PORT_EVCTRL_EVACT0_CLR_Val    << PORT_EVCTRL_EVACT2_Pos)
#define PORT_EVCTRL_EVACT2_TGL      (PORT_EVCTRL_EVACT0_TGL_Val    << PORT_EVCTRL_EVACT2_Pos)

#define PORT_EVCTRL_EVACT3_OUT      (PORT_EVCTRL_EVACT0_OUT_Val    << PORT_EVCTRL_EVACT3_Pos)
#define PORT_EVCTRL_EVACT3_SET      (PORT_EVCTRL_EVACT0_SET_Val    << PORT_EVCTRL_EVACT3_Pos)
#define PORT_EVCTRL_EVACT3_CLR      (PORT_EVCTRL_EVACT0_CLR_Val    << PORT_EVCTRL_EVACT3_Pos)
#define PORT_EVCTRL_EVACT3_TGL      (PORT_EVCTRL_EVACT0_TGL_Val    << PORT_EVCTRL_EVACT3_Pos)

#define MAX_TABLE		16

typedef uint16_t (*stepdown_control_vled_func)(void);
typedef bool (*stepdown_control_issafetyoff_func)(void);
typedef uint16_t (*stepdown_control_temp_func)(uint16_t);
typedef void (*stepdown_control_setcurrent_func)(uint8_t);
typedef void (*stepdown_control_channelenable_func)(bool, uint8_t);

// Variables per channel
struct stepdown_control_s {
	int32_t level_filter;			// Dmx level last filter value
	int32_t level_curve;			// Value after temp limit and table convert (after TW/WD ed), to be used for PWM
	int32_t level_curve16;			// Value after temp limit and table convert (after TW/WD ed), to be used for PWM
	int32_t level_curve_raw;		// Value before temp limit and table convert (after TW/WD ed)
	int32_t freq_new;				// Current frequency of PWM
	uint32_t period;				// Current period in ticks used for PWM
	int32_t pwm_new;				// Last duty cycle in ticks for PWM
	uint16_t level;					// Dmx value (as 16-bit)
	uint16_t channel;				// Channel number 1 to 4
	stepdown_control_vled_func vled_func;			// Function to get led voltage in mV (low pass filtered)
	stepdown_control_vled_func vled_raw_func;		// Function to led voltage in AD ticks (low pass filtered)
	stepdown_control_vled_func vled_last_func;		// Function to led voltage in AD ticks (last vulue no filtering)
	stepdown_control_setcurrent_func set_current;	// Function to set current for channel (mA is retrieved from emitter data)
	stepdown_control_temp_func temp_limit;
	stepdown_control_issafetyoff_func issafety_off;
	stepdown_control_channelenable_func channel_enable;		// Function to enable or disable channel (logic, timer and event system)
	volatile uint16_t stepdown_offtime;				// Current offtime in timer ticks
	uint16_t mA;									// Current mA for channel
	uint16_t vled_ad;				// Value where mA calc is done with
	uint16_t vled_max;				// Max since start
	uint16_t toff;					// Toff time in ticks calculated
	uint8_t no_change_cnt;			// Used to check changes in channel level
	uint8_t pwm_on_timer;			// Prescaler used for vfled if to low, keep last value
	uint16_t dac_data;				// Current DAC value
	uint16_t vin_ad;				// Last Vin ad value (low pass filtered)
	uint16_t e_temp;				// Emitter temperature
	uint16_t vled_max_store_mv;		// Max Vled since off (when at least 50%), used to check if to be stored (only done when both channels are off)
	uint16_t dac_data_max;			// Used fr low pass dac value
	uint8_t filter_shift;			// Used for low pass filter for TW
	uint8_t filter_fast_time;		// Used for dynamic mode time (fast delta handling)
	uint8_t filter_fast_speed;		// Used for dynamic mode speed (fast delta handling)
	uint8_t is_16b;					// Indicates if TCC timer is 16b (true) or 24-bit
	uint32_t test;					// Used for test output (serial, debug only))
	uint32_t test2;					// Used for test output (serial, debug only))
	int32_t level_curve_correct_prev;	// Used as previous value for temp corrections
	uint8_t timing_sel;				// Indicates which period/compare buffers are used
	uint8_t cnt;					// Index for randomized period and compare array
	uint16_t toff_min;				// Minimal toff value
	int per[2][MAX_TABLE];				// Randomized period buffer
	int cc0[2][MAX_TABLE];				// Randomized compare buffer
	int ccx[2][MAX_TABLE];				// Randomized compare buffer
	uint32_t tcc_freq;				// Frequency used for TCC timer
	uint32_t vfled;			// LED voltage (calculated wit ad and pwm)
	uint32_t power;					// Power per channel in mV
	uint8_t master;					// Master of channel to reduce power
	uint8_t chan_org;				// Non patched channel index
	uint16_t toff_min2;				// Minimal toff value 2 
	int8_t switch_sel;
	uint8_t fe;
	uint16_t mA_scaled;
	uint16_t dac_data_real;
	uint32_t switch_level_prev;
	uint32_t imax_avg_th;
	uint16_t vled_max_session;	// Max Vled since off (when at least 50%)
	uint16_t vled_max_session_prs;	// Max Vled since off (when at least 50%)
	uint16_t dac_scale_max;
	uint16_t oc_warning_prs;
	uint16_t sc_error_prs;
	int16_t pwm_offset;
	uint8_t force_calibration;
	bool update_timer;
} __attribute__((packed));

typedef struct stepdown_control_s stepdown_control_t;

// Proto
void StepDown_Init();
void StepDown_Close(uint16_t time);
bool StepDown_IsClosed();
void StepDown_Debug(uint8_t* par1, uint16_t length);
void StepDown_Module_Update();
void Stepdown_SetCurrentCh1(uint8_t chan_idx);
void Stepdown_SetCurrentCh2(uint8_t chan_idx);
void Stepdown_SetCurrentCh3(uint8_t chan_idx);
void Stepdown_SetCurrentCh4(uint8_t chan_idx);
uint8_t StepDown_DebugLevel();
void StepDown_ForceCalibration();
bool StepDown_SleepCheck();
void StepDown_ChannelEnable_Ch1(bool start, uint8_t chan_idx);
void StepDown_ChannelEnable_Ch2(bool start, uint8_t chan_idx);
void StepDown_ChannelEnable_Ch3(bool start, uint8_t chan_idx);
void StepDown_ChannelEnable_Ch4(bool start, uint8_t chan_idx);
uint32_t StepDown_GetPower();
uint32_t Filter_LowPass(uint32_t old, uint32_t new, uint8_t shift);
uint8_t StepDown_GetDutyPercentage(uint8_t chan_idx);

#endif /* STEPDOWN_H_ */