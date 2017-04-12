#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "sys/alt_irq.h"
#include "system.h"
#include "io.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include <altera_avalon_pio_regs.h>
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"


#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/timers.h"
#include "FreeRTOS/semphr.h"

typedef enum state {IDLE, SHED, UNSTABLE, STABLE, RECONNECT} State;
typedef enum request {CONNECT, DISCONNECT} Request;

//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw

#define FREQ_THRESHOLD_INC_AMOUNT 0.5  // amount of threshold (Hz) to increment/decrement
#define FREQ_THRESHOLD_INC_AMOUNT_P 0.1  // when precise_increment flag is high

#define ROC_THRESHOLD_INC_AMOUNT 1
#define ROC_THRESHOLD_INC_AMOUNT_P 0.2

// Priorities
#define PRVGADraw_Task_P    (tskIDLE_PRIORITY+1)
#define LOAD_CTRL_TASK_P	(tskIDLE_PRIORITY+2)
#define LOAD_MNGR_TASK_P    (tskIDLE_PRIORITY+3)
#define KEYB_UPDATE_TASK_P	(tskIDLE_PRIORITY+4)
#define FREQ_UPDATE_TASK_P  (tskIDLE_PRIORITY+5)

// Handlers
TaskHandle_t PRVGADraw;
TaskHandle_t load_ctrl_handle;
TaskHandle_t load_mngr_handle;
TaskHandle_t keyb_update_handle;
TaskHandle_t freq_update_handle;
TaskHandle_t idle_task_handle;
TimerHandle_t timer;

// Semaphores
SemaphoreHandle_t threshold_sem;  // for r/w thresholds
SemaphoreHandle_t all_connected_sem;  // for all_connected flag
SemaphoreHandle_t load_mngr_idle_sem;  // task blocker 
SemaphoreHandle_t freq_roc_sem;  // For freq[100], dfreq[100] and freq_index


// global
static QueueHandle_t Q_freq_data;
static QueueHandle_t Q_time_stamp;
static QueueHandle_t Q_keyb_data;
static QueueHandle_t Q_load_request;

volatile unsigned char maintenance_mode = 0;
volatile unsigned char precise_increment = 0;
volatile unsigned char timer_expired = 0;
unsigned char all_connected = 0;

double freq_threshold = 49.5;
double roc_threshold = 10.0;
double freq[100];
double dfreq[100];
int freq_index = 99;
int freq_index_new = 98;
TickType_t time_stamps[100];
TickType_t start_time;
int shed_index;
TickType_t time_taken[5];

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;


// freq_relay - Retrieve frequency data and push to queue
void freq_relay(){
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	start_time = xTaskGetTickCountFromISR();
	xQueueSendToBackFromISR(Q_freq_data, &temp, pdFALSE);
	xQueueSendToBackFromISR(Q_time_stamp, &start_time, pdFALSE);

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(load_mngr_handle, &xHigherPriorityTaskWoken);
	return;
}

// button_isr - Toggle maintenance mode and threshold precision
void button_isr(void* context, alt_u32 id){
	if (IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) == 4){
		// toggle
		maintenance_mode ^= 1;
		// set maintenance LED
		// IOWR(RED_LEDS_BASE, 0, 0 | (IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) & 0x0FFFFFFF ) | maintenance_mode << 17);

	} else if (IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE) == 2){
		precise_increment ^= 1;
	}

	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

// Keyboard ISR - Push Keyboard byte to queue
void ps2_isr(void* ps2_device, alt_u32 id){
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	status = decode_scancode (ps2_device, &decode_mode , &key , &ascii) ;
	if ( status == 0 ) //success
	{
		// print out the result
		switch ( decode_mode )
		{
		case KB_ASCII_MAKE_CODE :
			xQueueSendToBackFromISR(Q_keyb_data, &key, pdFALSE);
			break ;

		default :
			break ;
		}
	}
}

// timer_500ms_isr - Does nothing
void timer_500ms_isr(xTimerHandle t_timer){
	timer_expired = 1;
}

//update_ROC_and_frequency_task - Receives frequency and ROC from ISR and stores it
void freq_update_task()
{
	while(1)
	{
		xQueueReceive(Q_freq_data, freq+freq_index, portMAX_DELAY); // Pops queue, blocks indefinitely if empty
		xQueueReceive(Q_time_stamp, time_stamps+freq_index, (TickType_t) 2);

		//Store data into global variables
		xSemaphoreTake(freq_roc_sem, portMAX_DELAY);

		//calculate frequency RoC
		if(freq_index == 0){
			dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
		}
		else{
			dfreq[freq_index] = (freq[freq_index]-freq[freq_index-1]) * 2.0 * freq[freq_index]* freq[freq_index-1] / (freq[freq_index]+freq[freq_index-1]);
		}

		if (dfreq[freq_index] > 100.0){
			dfreq[freq_index] = 100.0;
		}

		freq_index = (++freq_index) % 100;  // point to the next data (oldest) to be overwritten
		freq_index_new = (++freq_index_new) % 100;
		xSemaphoreGive(freq_roc_sem);
	}
}



// Keyboard_update_task - Retrieves keyboard data from queue and processes it.
//		Updates frequency thresholds
void keyboard_update_task(){
	unsigned char key;
	unsigned char key_pressed = 0;  // Only register key down, not key up

	while(1){
		xQueueReceive(Q_keyb_data, &key, portMAX_DELAY);  // Pops queue, blocks indefinitely if empty

		switch (key)
		{
		case 0x72:  // DOWN
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				printf("DOWN PRESSED\n");

				if (xSemaphoreTake(threshold_sem, (TickType_t ) 10)){
					freq_threshold -= precise_increment? FREQ_THRESHOLD_INC_AMOUNT_P : FREQ_THRESHOLD_INC_AMOUNT;

					if (freq_threshold < 0.0){
						freq_threshold = 0;
					}

					xSemaphoreGive(threshold_sem);
				}

			}
			break;

		case 0x75:  // UP
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				printf("UP PRESSED\n");

				if (xSemaphoreTake(threshold_sem, (TickType_t ) 10)){
					freq_threshold += precise_increment? FREQ_THRESHOLD_INC_AMOUNT_P : FREQ_THRESHOLD_INC_AMOUNT;

					if (freq_threshold < 0.0){
						freq_threshold = 0;
					}

					xSemaphoreGive(threshold_sem);
				}
			}
			break;

		case 0x6b:  // LEFT
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				printf("LEFT PRESSED\n");

				if (xSemaphoreTake(threshold_sem, (TickType_t ) 10)){
					roc_threshold -= precise_increment? ROC_THRESHOLD_INC_AMOUNT_P : ROC_THRESHOLD_INC_AMOUNT;

					if (roc_threshold < 0.0){
						roc_threshold = 0;
					}

					xSemaphoreGive(threshold_sem);
				}
			}
			break;

		case 0x74:  // RIGHT
			if (key_pressed){
				key_pressed = 0;
			} else {
				key_pressed = 1;
				printf("RIGHT PRESSED\n");

				if (xSemaphoreTake(threshold_sem, (TickType_t ) 10)){
					roc_threshold += precise_increment? ROC_THRESHOLD_INC_AMOUNT_P : ROC_THRESHOLD_INC_AMOUNT;

					if (roc_threshold < 0.0){
						roc_threshold = 0;
					}

					xSemaphoreGive(threshold_sem);
				}
			}
			break;

		default:
			printf("key = %c\n", &key);
			break;
		}

		IOWR(SEVEN_SEG_BASE, 0, key);
	}
}


// load_manager_task -
void load_manager_task(){
	State current_state = IDLE;
	State next_state = IDLE;
	Request req;
	double freq_local, roc_local;
	double freq_threshold_local, roc_threshold_local;

	while(1){

		ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

		// "asynchronous" "reset"
		if (maintenance_mode){
			current_state = IDLE;
		} else {
			current_state = next_state;
		}

		switch(current_state){
		case IDLE:
			xSemaphoreGive(load_mngr_idle_sem);

			if (xSemaphoreTake(freq_roc_sem, (TickType_t) 10)){
				freq_local = freq[freq_index_new];
				roc_local = fabs(dfreq[freq_index_new]);
				xSemaphoreGive(freq_roc_sem);
			}
			if (xSemaphoreTake(threshold_sem, (TickType_t) 10)){
				freq_threshold_local = freq_threshold;
				roc_threshold_local = roc_threshold;
				xSemaphoreGive(threshold_sem);
			}

			if ((freq_local < freq_threshold_local) || (roc_local > roc_threshold_local)) {
				shed_index = freq_index_new;
				next_state = SHED;
			} else {
				next_state = IDLE;
			}
			break;

		case SHED:
			req = DISCONNECT;
			xQueueSendToBack(Q_load_request, &req, 2);

			next_state = UNSTABLE;
			xTimerReset(timer, 5);
			break;

		case UNSTABLE:
			if (xSemaphoreTake(freq_roc_sem, (TickType_t) 10)){
				freq_local = freq[freq_index_new];
				roc_local = fabs(dfreq[freq_index_new]);
				xSemaphoreGive(freq_roc_sem);
			}
			if (xSemaphoreTake(threshold_sem, (TickType_t) 10)){
				freq_threshold_local = freq_threshold;
				roc_threshold_local = roc_threshold;
				xSemaphoreGive(threshold_sem);
			}

			if ((freq_local > freq_threshold_local) && (roc_local < roc_threshold_local)){
				xTimerReset(timer, 5);
				next_state = STABLE;
			} else if (timer_expired){
				shed_index = freq_index_new;
				timer_expired = 0;
				next_state = SHED;
			} else {
				next_state = UNSTABLE;
			}

			break;

		case STABLE:
			if (xSemaphoreTake(freq_roc_sem, (TickType_t) 10)){
				freq_local = freq[freq_index_new];
				roc_local = fabs(dfreq[freq_index_new]);
				xSemaphoreGive(freq_roc_sem);
			}
			if (xSemaphoreTake(threshold_sem, (TickType_t) 10)){
				freq_threshold_local = freq_threshold;
				roc_threshold_local = roc_threshold;
				xSemaphoreGive(threshold_sem);
			}

			if ((freq_local < freq_threshold) || (roc_local > roc_threshold_local)) {
				xTimerReset(timer, 5);
				next_state = UNSTABLE;
			} else if (timer_expired){
				timer_expired = 0;
				next_state = RECONNECT;
			} else {
				next_state = STABLE;
			}

			break;

		case RECONNECT:
			req = RECONNECT;
			xQueueSendToBack(Q_load_request, &req, 20);

			if (xSemaphoreTake(all_connected_sem, (TickType_t ) 10)){
				if (all_connected){
					next_state = IDLE;
				} else {
					xTimerReset(timer, 1);
					next_state = STABLE;
				}

				xSemaphoreGive(all_connected_sem);
			}

			break;

		default:
			printf("\n****** WTF THE GOING ON ******\n");
			break;
		}

//		vTaskDelay(5);
	}
}

// Load control task
// Actually turns on/off loads
void load_control_task(){
	int i;
	int k = 0;
	unsigned int uiSwitchValue = 0;
	unsigned int red_led, green_led;
	Request req;

	const TickType_t task_period = 10;
	TickType_t last_wake_time = xTaskGetTickCount();

	while(1){

		vTaskDelayUntil(&last_wake_time, task_period);

		// read the value of the switch and store to uiSwitchValue
		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);

		if (maintenance_mode){
			// write the value of the switches to the red LEDs
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, (uiSwitchValue & 0x1F) | maintenance_mode << 17);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);

			xQueueReset(Q_load_request);  // clear all requests

			// all loads are connected - no loads shed
			xSemaphoreTake(all_connected_sem, portMAX_DELAY);
			all_connected = 1;
			xSemaphoreGive(all_connected_sem);

		} else {
			// turn off maintenance mode LED
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) & ~(1 << 17));

			red_led = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
			green_led = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

			// Load Manager is in IDLE, ie. everything is stable and good
			if (xSemaphoreTake(load_mngr_idle_sem, (TickType_t) 2)){
				// connect all loads
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiSwitchValue & 0x1F);

				// turn off shed status LEDs - 0xE0 clears the least significant 5 bits.
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE & 0xE0));
				for (i = 4; i >= 0; i--){
					IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green_led & ~(1 << i));
				 	green_led &= ~(1 << i);
				}

				continue;

			// Load Manager is not in IDLE, could be stable or unstable
			} else {

				// turn off load if toggle switch is off for that load
				for (i = 0; i < 5; i++){
					if ((uiSwitchValue & (1 << i)) ^ (1 << i)){  // a toggle was switched off
						IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red_led & ~(1 << i));
						IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green_led & ~(1 << i));
						green_led = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
					}
				}

				// Read request queue from Load manager
				if (uxQueueMessagesWaiting(Q_load_request) != 0){
					xQueueReceive(Q_load_request, &req, (TickType_t) 5);

					switch(req)
					{
					// shed lowest priority load
					case DISCONNECT:


						for (i = 0; i < 5; i++){
							if(red_led & (1 << i)){  // if the load is on, shed that one
								IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red_led & ~(1 << i));
								IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green_led | (1 << i));

								// not all loads are connected any more
								// if(xSemaphoreTake(all_connected_sem, (TickType_t) 10)){
								// 	all_connected = 0;
								// 	xSemaphoreGive(all_connected_sem);
								// }

								time_taken[k] = xTaskGetTickCount() - time_stamps[shed_index];
								k = (++k) % 5;
								break;
							}
						}

						break;

					// unshed highest priority load
					case RECONNECT:
						for (i = 4; i >= 0; i--){
							if ((uiSwitchValue & (1 << i)) ^ (red_led & (1 << i))){  // if toggle is on but load is off, unshed that load
								IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, red_led | (1 << i));
								IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, green_led & ~(1 << i));
								break;
							}
						}

						break;

					default:
						printf("SUMTING WONG\n");
						break;
					}
				}


				// Check if all loads are connected
				if(xSemaphoreTake(all_connected_sem, (TickType_t) 5)){
					if (((IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE) & 0x1F) ^ (uiSwitchValue & 0x1F)) == 0){
						all_connected = 1;
					} else {
						all_connected = 0;
					}

					xSemaphoreGive(all_connected_sem);
				}

			}

		}

//		 vTaskDelay(5);
	}
}


/****** VGA display ******/
// Task
void PRVGADraw_Task(void *pvParameters ){


	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);



	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);

	alt_up_char_buffer_string(char_buf, "Run time (HH:MM:SS.m)    = ", 12, 40);
	alt_up_char_buffer_string(char_buf, "Frequency Threshold (Hz) =       (UP/DOWN arrow keys)", 12, 42);  // 40
	alt_up_char_buffer_string(char_buf, "RoC Threshold (Hz/s)     =       (LEFT/RIGHT arrow keys)", 12, 44);
	alt_up_char_buffer_string(char_buf, "Time taken to shed (ms)  = ", 12, 46);



	int j = 0;
	int k = 0;
	Line line_freq, line_roc;

	char temp_buf[6];
	unsigned int milisec;

	while(1){

		milisec = xTaskGetTickCount();


		sprintf(temp_buf, "%02d:%02d:%02d.%1d", (milisec/3600000) % 24, (milisec/60000) % 60, (milisec/1000) % 60, (milisec/100) % 10);
		alt_up_char_buffer_string(char_buf, temp_buf, 40, 40);

		// Read thresholds and print to screen
		xSemaphoreTake(threshold_sem, (TickType_t ) 10);
		sprintf(temp_buf, "%2.1f", freq_threshold);
		alt_up_char_buffer_string(char_buf, temp_buf, 40, 42);
		sprintf(temp_buf, "%2.1f", roc_threshold);
		xSemaphoreGive(threshold_sem);
		alt_up_char_buffer_string(char_buf, temp_buf, 40, 44);

		sprintf(temp_buf, "%2d, %2d, %2d, %2d, %2d", time_taken[4], time_taken[3], time_taken[2], time_taken[1], time_taken[0]);
		alt_up_char_buffer_string(char_buf, temp_buf, 40, 46);



		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

//		xSemaphoreTake(freq_roc_sem, portMAX_DELAY);
		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(freq_index+j)%100]) > MIN_FREQ) && ((int)(freq[(freq_index+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freq_index+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(freq_index+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freq_index+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(freq_index+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
//		xSemaphoreGive(freq_roc_sem);
		vTaskDelay(10);

	}
}


// Idle LED flash to indicate that the system hasn't crashed
void idle_task(){
	while(1){
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE) ^ (1 << 7));
		vTaskDelay(1000);
	}
}

int main()
{
	// FreeRTOS initialisation
	// Initialise queues
	Q_freq_data = xQueueCreate( 100, sizeof(double) );
	Q_time_stamp = xQueueCreate(100, sizeof(TickType_t));
	Q_keyb_data = xQueueCreate(5, sizeof(unsigned char));
	Q_load_request = xQueueCreate(5, sizeof(Request));
	timer = xTimerCreate("Timer_500ms", 500, pdFALSE, NULL, timer_500ms_isr);

	// Initialise semaphores
	threshold_sem = xSemaphoreCreateBinary();
	all_connected_sem = xSemaphoreCreateBinary();
	load_mngr_idle_sem = xSemaphoreCreateBinary();
	freq_roc_sem = xSemaphoreCreateBinary();

	xSemaphoreGive(threshold_sem);
	xSemaphoreGive(all_connected_sem);
	xSemaphoreGive(load_mngr_idle_sem);
	xSemaphoreGive(freq_roc_sem);

	// Hardware initialisation
	// Initialise ps2 device
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	// Reigster Interrupts
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);
	alt_up_ps2_enable_read_interrupt(ps2_device);

	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	// enable interrupts for all buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	// register the ISR
	alt_irq_register(PUSH_BUTTON_IRQ, NULL, button_isr);

//	// Priorities
//	#define PRVGADraw_Task_P    (tskIDLE_PRIORITY+1)
//	#define LOAD_CTRL_TASK_P	(tskIDLE_PRIORITY+2)
//	#define LOAD_MNGR_TASK_P    (tskIDLE_PRIORITY+3)
//	#define KEYB_UPDATE_TASK_P	(tskIDLE_PRIORITY+4)
//	#define FREQ_UPDATE_TASK_P  (tskIDLE_PRIORITY+5)

	// Tasks

	xTaskCreate(PRVGADraw_Task, "DrawTsk", configMINIMAL_STACK_SIZE, NULL, PRVGADraw_Task_P, &PRVGADraw);
	xTaskCreate(load_control_task, "load_control_task", configMINIMAL_STACK_SIZE, NULL, LOAD_CTRL_TASK_P, &load_ctrl_handle);
	xTaskCreate(load_manager_task, "load_manager_task", configMINIMAL_STACK_SIZE, NULL, LOAD_MNGR_TASK_P, &load_mngr_handle);
	xTaskCreate(keyboard_update_task, "keyboard_update_task", configMINIMAL_STACK_SIZE, NULL, KEYB_UPDATE_TASK_P, &keyb_update_handle);
	xTaskCreate(freq_update_task, "freq_update_task", configMINIMAL_STACK_SIZE, NULL, FREQ_UPDATE_TASK_P, &freq_update_handle);
	xTaskCreate(idle_task, "Idle", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &idle_task_handle);

	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, 0);
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0);

	vTaskStartScheduler();

	while(1);

  return 0;
}

