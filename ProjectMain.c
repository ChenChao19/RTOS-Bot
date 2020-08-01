/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define PTE3 3
#define PTE4 4
#define PTE5 5
#define PTE20 20
#define PTE21 21
#define PTE22 22
#define PTE23 23
#define PTE29 29
#define PTE30 30
#define LED_LENGTH 8
 
#define PTB0_Pin 	0
#define PTB1_Pin 	1
#define TEMPO 		619
#define VICTORY		462

#define PTA4_Pin 	4
#define PTA5_Pin 	5
#define PTC8 			8
#define PTC9 			9
#define MOTOR_PWM 1000

#define RED_LED 	18 // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 	1 // PortD Pin 1
#define MASK(x) 	(1 << (x))

#define Q_SIZE 32

#define CONNECT 			1
#define MOVE_FORWARD 	2
#define PIVOT_LEFT 		3
#define PIVOT_RIGHT 	4
#define MOVE_REVERSE 	5
#define STOP 					6
#define FINISH				7

#define BAUD_RATE 9600
#define UART_TX_PORTC04 04
#define UART_RX_PORTC03 03
#define UART1_INT_PRIO 128

typedef enum {
  RED, 
  GREEN, 
  BLUE
}color_t;

typedef enum {
  LED_ON,
  LED_OFF
}control_t;

unsigned int led_array[] = { PTE3, PTE4, PTE5, PTE20, PTE21, PTE22, PTE23, PTE29 };

unsigned int freq[] = {1431, 1275, 1136, 1074, 956, 852, 759, 
											716, 637, 568, 537, 478, 426, 380, 
											507, 0, 903, 804};

typedef enum {C4, D4, E4, F4, G4, A4, B4, 
							C5, D5, E5, F5, G5, A5, B5, 
							FS, REST, Af, Bf} Note;

Note music[] = {E5, E5, E5, D5, B4, D5, B4, A4, G4,
								A4, A4, A4, G4, A4, B4, D5, FS, G5};

float time[] = {0.5, 0.5, 0.25, 0.25, 0.25, 0.5, 1.25, 0.25, 0.25,
								0.5, 0.5, 0.25, 0.25, 0.25, 0.5, 0.25, 0.75, 0.75};

Note start_tune[] = {A4, E4, A4};
								
float start_time[] = {0.5, 0.25, 1};
								
Note victory[] = {C5, C5, C5, C5, Af, Bf, C5, Bf, C5};

float victory_time[] = {0.33, 0.33, 0.34, 1, 1, 1, 0.67, 0.33, 3};

volatile uint8_t rx_data;
osThreadId_t control_Id;
osEventFlagsId_t control_flag;
osMutexId_t connect_success_mutex;

typedef struct {
	unsigned char Data[Q_SIZE];
	unsigned int Head;
	unsigned int Tail;
	unsigned int Size;
	
} Q_T;

Q_T tx_q, rx_q;

void Q_Init(Q_T *q) {
	unsigned int i;
	for (i = 0; i < Q_SIZE; i++) {
		q->Data[i] = 0;
	}
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

int Q_Empty(Q_T *q) {
	return q->Size == 0;
}

int Q_Full(Q_T *q) {
	return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T *q, unsigned char d) {
	if (!Q_Full(q)) {
		q->Data[q->Tail++] = d;
		q->Tail %= Q_SIZE;
		q->Size++;
		return 1;
	} else {
		return 0;
	}
} 

unsigned char Q_Dequeue(Q_T *q) {
	unsigned char t = 0;
	if(!Q_Empty(q)) {
		t = q->Data[q->Head];
		q->Data[q->Head++] = 0;
		q->Head %= Q_SIZE;
		q->Size--;
	}
	return t;
}								
								
								
void change_freq(Note i) {
	if (i == REST) {
		TPM1_C0V = 0;
	} else {
		TPM1->MOD = freq[i] - 1;
		TPM1_C0V = (freq[i] - 1) / 5;
	}
}

void rest() {
	TPM1_C0V = 1;
}

void UART1_IRQHandler(void) {
  NVIC_ClearPendingIRQ(UART1_IRQn);
  if ((UART1->S1 & UART_S1_RDRF_MASK)) {
    rx_data = UART1->D;
  }
  /*
  if (UART1->S1 & (UART_S1_OR_MASK |
                   UART_S1_NF_MASK |
                   UART_S1_FE_MASK |
                   UART_S1_PF_MASK)){
    // Handle the error
    // Clear the interrupt flag
  
  }*/
}

void init_LED(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	for (int i = 0; i < LED_LENGTH; i++) {
		PORTE->PCR[led_array[i]] |= PORT_PCR_MUX(1);	
		PTE->PDDR |= MASK(led_array[i]);
	}
	PORTE->PCR[PTE30] |= PORT_PCR_MUX(1);	
	PTE->PDDR |= MASK(PTE30);
}

void blinkSequence(void) {
	for (int i = 0; i < LED_LENGTH; i++) {
		PTE->PSOR |= MASK(led_array[i]);
		//delay();
		PTE->PCOR |= MASK(led_array[i]);
	}
}

void offAll(void) {
	for (int i = 0; i < LED_LENGTH; i++) {
		PTE->PCOR |= MASK(led_array[i]);
	}
}

void onAll(void) {
	for (int i = 0; i < LED_LENGTH; i++) {
		PTE->PSOR |= MASK(led_array[i]);
	}
}

void led_control(color_t color, control_t control) {
  if (color == RED) {
    if (control == LED_OFF) {
      PTB->PSOR |= MASK(RED_LED);
    } else {
      PTB->PCOR |= MASK(RED_LED);
    }
  } else if (color == GREEN) {
    if (control == LED_OFF) {
      PTB->PSOR |= MASK(GREEN_LED);
    } else {
      PTB->PCOR |= MASK(GREEN_LED);
    }
  } else if (color == BLUE) {
    if (control == LED_OFF) {
      PTD->PSOR |= MASK(BLUE_LED);
    } else {
      PTD->PCOR |= MASK(BLUE_LED);
    }
  }
}

/* Init UART1*/
void initUART1(uint32_t baud_rate) 
{
	uint32_t divisor, bus_clock;
	
	SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	PORTC->PCR[UART_TX_PORTC04] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[UART_TX_PORTC04] |= PORT_PCR_MUX(3);
	
	PORTC->PCR[UART_RX_PORTC03] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[UART_RX_PORTC03] |= PORT_PCR_MUX(3);
	
	UART1->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (baud_rate * 16);
	UART1->BDH = UART_BDH_SBR(divisor >> 8);
	UART1->BDL = UART_BDL_SBR(divisor);
	
	UART1->C1 = 0;
	UART1->S2 = 0;
	UART1->C3 = 0;

	NVIC_SetPriority(UART1_IRQn, 128);
	NVIC_ClearPendingIRQ(UART1_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);

	UART1->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	UART1->C2 |= UART_C2_RIE_MASK;
}
 
void initPWM(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);
	
	PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
	SIM_SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM1->MOD = MOTOR_PWM;
	TPM1_C0V = 0;
	
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

}

void initMotorPWM(void) {
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	PORTA->PCR[PTA4_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA4_Pin] |= PORT_PCR_MUX(3);
	
	PORTA->PCR[PTA5_Pin] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[PTA5_Pin] |= PORT_PCR_MUX(3);
	
	PORTC->PCR[PTC8] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC8] |= PORT_PCR_MUX(3);
	
	PORTC->PCR[PTC9] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PTC9] |= PORT_PCR_MUX(3);
	
	SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK;
	
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM0->MOD = MOTOR_PWM;
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C4V = 0;
	TPM0_C5V = 0;
	
	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
	
	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void moveForward() {
	TPM0_C1V = 0.5 * MOTOR_PWM;
	TPM0_C2V = 0;
	TPM0_C4V = 0.5 * MOTOR_PWM;
	TPM0_C5V = 0;
}

void reverse() {
	TPM0_C1V = 0;
	TPM0_C2V = 0.5 * MOTOR_PWM;
	TPM0_C4V = 0;
	TPM0_C5V = 0.5 * MOTOR_PWM;
}

void left() {
	TPM0_C1V = 0;
	TPM0_C2V = 0.5 * MOTOR_PWM;
	TPM0_C4V = 0.5 * MOTOR_PWM;
	TPM0_C5V = 0;
}

void right() {
	TPM0_C1V = 0.5 * MOTOR_PWM;
	TPM0_C2V = 0;
	TPM0_C4V = 0;
	TPM0_C5V = 0.5 * MOTOR_PWM;
}

void stop() {
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C4V = 0;
	TPM0_C5V = 0;
}
	
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
void sound (void *argument) {
	for (;;) {
		for (int i = 0; i < 18; i++) {
			osEventFlagsWait(control_flag, 0x1, osFlagsNoClear, osWaitForever);
			Note note = music[i];
			change_freq(note);
			osDelay(0.95 * time[i] * TEMPO);
			rest();
			osDelay(0.05 * time[i] * TEMPO);
		}
	}
}

void red_moving_thread (void *argument) {
	for (;;) {
		osEventFlagsWait(control_flag, 0x21, osFlagsNoClear | osFlagsWaitAll, osWaitForever);
		
		PTE->PSOR |= MASK(PTE30);
		osDelay(500);
		PTE->PCOR |= MASK(PTE30);
		osDelay(500);
	}
}

void green_moving_thread (void *argument) {
	for (;;) {
		for (int i = 0; i < LED_LENGTH; i++) {
			osEventFlagsWait(control_flag, 0x21, osFlagsNoClear | osFlagsWaitAll, osWaitForever);
			offAll();
			PTE->PSOR |= MASK(led_array[i]);
			osDelay(TEMPO / 4);
			PTE->PCOR |= MASK(led_array[i]);
		}
	}
}

void red_stationery_thread (void *argument) {
	for (;;) {
		osEventFlagsWait(control_flag, 0x41, osFlagsNoClear | osFlagsWaitAll, osWaitForever);
		
		PTE->PSOR |= MASK(PTE30);
		osDelay(250);
		PTE->PCOR |= MASK(PTE30);
		osDelay(250);
	}
}

void green_stationery_thread (void *argument) {
	for (;;) {
		osEventFlagsWait(control_flag, 0x41, osFlagsNoClear | osFlagsWaitAll, osWaitForever);
		onAll();
	}
}

void motor_control_thread (void *argument) {
	for (;;) {
		osEventFlagsWait(control_flag, 0x1, osFlagsNoClear, osWaitForever);
		if (rx_data == MOVE_FORWARD) {
			osEventFlagsClear(control_flag, 0x40);
			osEventFlagsSet(control_flag, 0x20);
			moveForward();
		} else if (rx_data == MOVE_REVERSE) {
			osEventFlagsClear(control_flag, 0x40);
			osEventFlagsSet(control_flag, 0x20);
			reverse();
		} else if (rx_data == PIVOT_RIGHT) {
			osEventFlagsClear(control_flag, 0x40);
			osEventFlagsSet(control_flag, 0x20);
			right();
		} else if (rx_data == PIVOT_LEFT) {
			osEventFlagsClear(control_flag, 0x40);
			osEventFlagsSet(control_flag, 0x20);
			left();
		} else if (rx_data == STOP) {
			osEventFlagsClear(control_flag, 0x20);
			osEventFlagsSet(control_flag, 0x40);
			stop();
		}
	}
}

void main_control_thread (void *argument) {
	for (;;) {
		if (rx_data == CONNECT) {
			osEventFlagsSet(control_flag, 0x10);
			osDelay(2000);
			osEventFlagsSet(control_flag, 0x41);
		}
		if (rx_data == FINISH) {
			osEventFlagsClear(control_flag, 0x1);
			osEventFlagsSet(control_flag, 0x2);
		}
	}
}

void connect_success_thread (void *argument) {
	for (;;) {
		osEventFlagsWait(control_flag, 0x10, osFlagsWaitAny, osWaitForever);
		osMutexAcquire(connect_success_mutex, osWaitForever);
		onAll();
		osDelay(500);
		offAll();
		osDelay(500);
		onAll();
		osDelay(500);
		offAll();
	}
}

void victory_thread (void * argument) {
	for (;;) {
		for (int i = 0; i < 9; i++) {
			osEventFlagsWait(control_flag, 0x2, osFlagsNoClear, osWaitForever);
			Note note = victory[i];
			change_freq(note);
			osDelay(0.95 * victory_time[i] * VICTORY);
			rest();
			osDelay(0.05 * victory_time[i] * VICTORY);
		}
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initPWM();
	init_LED();
	initMotorPWM();
	initUART1(BAUD_RATE);
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
	const osThreadAttr_t thread_attr = {
		.priority = osPriorityAboveNormal
	};
	control_flag = osEventFlagsNew(NULL);
	connect_success_mutex = osMutexNew(NULL);
  osThreadNew(main_control_thread, NULL, NULL);
	osThreadNew(motor_control_thread, NULL, NULL);
	osThreadNew(connect_success_thread, NULL, NULL);
	osThreadNew(sound, NULL, NULL);   
  osThreadNew(red_moving_thread, NULL, NULL);    
  osThreadNew(green_moving_thread, NULL, NULL);    
	osThreadNew(victory_thread, NULL, NULL);
	osThreadNew(red_stationery_thread, NULL, NULL);
	osThreadNew(green_stationery_thread, NULL, NULL);
	
	osKernelStart();                      // Start thread execution
  for (;;) {}
}
