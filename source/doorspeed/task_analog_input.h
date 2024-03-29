#ifndef _TASK_ANALOG_INPUT_
#define _TASK_ANALOG_INPUT_

void analog_input_task_init();
int8_t analog_input_read(uint8_t channel, int32_t *data);
void analog_input_set_sample_interval(uint16_t ms);
int8_t analog_input_read_packed(uint8_t channel, uint8_t *data);
#endif