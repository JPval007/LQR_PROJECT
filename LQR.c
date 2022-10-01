//*****************************************************************************
//
// Author: Juan Pablo Valenzuela
// Description: LQR controller for the PWM in a Buck converter
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_ints.h" //librerias para el adc
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h" // librerias para el adc
#include "inc/hw_memmap.h"
#include "inc/hw_GPIO.h"    //needed to unlock pins to specific functions such as 
uart2, pwm
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.c" //importante porque sino no funciona uartstudio
//PINS PA6 AND PA7 FOR PWM CONTROL
//*****************************************************************************
//
// Define pin to LED color mapping.
//
//*****************************************************************************
//*****************************************************************************
//  Definitions
//*****************************************************************************
#define RED_LED     GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED   GPIO_PIN_3
#define PWM1_PIN    GPIO_PIN_6
#define PWM2_PIN    GPIO_PIN_7
//Sample freq
#define sample_freq 10000 //default is 10000 luis recomienda 50kHz de periodo de 
muestreo
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
//*****************************************************************************
// Function prototypes
//*****************************************************************************
void setup(void);
void pwm_config(void);
void adc_config(void);
void adc_handler(void);
void timer0_config();
void interrupt_config();
void Timer0IntHandler();
void LQR();
void Observer();
void loop(void);
void uart_config();
//*****************************************************************************
// Variables
//*****************************************************************************
const uint32_t clockfreq = 40000000; // 80MHz
const uint32_t pwm_period = 50000; //Hz is the frecuency of the signal
uint32_t pwm_ticks = 0; //ticks until the period is reset valor máximo es 40,000
//Number of ticks that the signal stays on
uint32_t pwm_out4_duty_cycle = 35000; //PF0 THE ONE THAT IS THE INPUT OF THE BUCK 
CONVERTER
uint32_t pwm_out5_duty_cycle = 300; //PF1
uint32_t pwm_out6_duty_cycle = 300; //PF2
uint32_t pwm_out7_duty_cycle = 300; //PF3
uint32_t increment = 10;
//ADC READINGS
uint32_t Cap_value;
float V_cap = 0; //this is x1 state variable
float previousV_cap = 0; //this is x1[n-1]
float ic = 0; //this is x2 state variable
float previousic = 0; //this is x2[n-1]
//TIMER0
//CONTROL SYSTEM
//*****************************************************************************
// Valores componentes
float L = 9.4/1000; //4.7 mH + 4.7mH
float C = 100/1000000; //100uF
float R = 330.0;
float u = 0;            //variable where the calculations are stored
uint32_t u_int = 0; //integrer variable to write to the pwm
// Operating points and voltajes
float vo = 2.0; //Voltaje en la salida
float  xss1 = 0.0061; //operation point xss = [xss1; xss2];
float  xss2 = 2.0;
float  uss = 0.4762; //You get this from matlab and is a 1x1 matrix
//*****************************************************************************
//              Inputs
//x = [x11,x21]
float x11 = 0; //inductor current
float x21 = 0; //capacitor voltage
//previous values
float previousx11 = 0;
float previousx21 = 0;
float y = 0; //Y is a 1x1 matrix
float previousy = 0;
//*****************************************************************************
float Nbar = 0; //compensation variable don ́t forget
//*****************************************************************************
float vs = 3.3; //voltaje that feeds the circuit to fill the capacitor
//2X2 MATRIX
/*
 * A = [0, -1/L; 1/Cap, -1/(R*Cap)];
 *
 * A = [ a11, a12; a21, a22] representación en matlab
 */
float a11 = 0;
float a12 = -103;
float a21 = 10000;
float a22 = -30;
/*
 * B = B = [Vs/L; 0];   B = [b11;b21];
 */
float b11 = 432.9897;
float b21 = 0;
// C = [0,1];   C = [c11,c12];
float c11 = 0;
float c12 = 1;
//*****************************************************************************
//                          MATLAB POLE PLACEMENT
//              Kpp = Kpp = place(A,B,p) = [kpp1, kpp2];
float kpp1 = 0;
float kpp2 = 0;
//                          MATLAB LQR
//              Klqr = lqr(A,B,Q,R) = [klqr1, klqr2];
float klqr1 = 3.6871;
float klqr2 = 0.2893;
//*****************************************************************************
//                          DISEÑO OBSERVADOR
/*L = lqr(A', C', Q, R)' = [lobs1, lobs2];
Aobs = A - L*C = [aobs11, aobs12; aobs21, aobs22];
Bobs = [B L] = [bobs11,bobs12];
Cobs = [1, 0]= [cobs11,cobs12];
Dobs = zeros(1,2);
*/
float lobs1 = 3.6871;
float lobs2 = 0.2893;
//Observer A matrix
float aobs11 = 0;
float aobs12 = -107;
float aobs21 = 10000;
float aobs22 = -31;
//Observer B matrix -> Bobs = [B,L];
float bobs11 = 432.9897;
float bobs12 = 3.6871;
float bobs21 = 0;
float bobs22 = 0.2893;
//Observer C matrix -> Cobs = [1, 0];
float cobs11 = 1;
float cobs12 = 0;
float cobs21 = 0;
float cobs22 = 1;
float ic_obs = 0;
//*****************************************************************************
float Ts = 1/sample_freq; //Período de muestreo para la discretización
//*****************************************************************************
//
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//
//*****************************************************************************
int
main(void)
{
    //SETTING UP INITIAL VALUES
    //Operating points (bjt transistor)
    xss1 = vo/R;
    xss2 = vo;
    //A matrix of system
    a12 = -1/L;
    a21 = 1/C;
    a22 = -1/(R*C);
    //B matrix of system
    b11 = vs/L;
    //PWM CONFIG INITIAL STATE
    pwm_ticks = clockfreq/pwm_period; //the maximum number that you can write to 
the pwm function
    setup();
    //
    // Loop Forever
    //
    while(1)
    {
        loop();
    }
}
void setup(void){
    //
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);
    //
    // Enable the PORTF WITH THE ON BOARD LEDS
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    //
    // Configure the GPIO port for the LED operation (Output)
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);
    pwm_config(); //configure pwm output
    adc_config(); //configure adc readings with interruption
    timer0_config(); //configure timer 0 with interruption
    interrupt_config(); //CONFIGURE ALL INTERRUPTIONS
    uart_config();
}
void loop(void){
}
//rest of the code
void pwm_config(void){
    //ENABLE THE PWM MODULE FUNCTIONALITY OF PORT F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //ENABLE CLOCK FOR PWM = 80MHZ/2 = 40MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);
    //This function allows the lock placed on the pins of the GPIO module to be 
removed so that
    //additional functionality may be used, in this case PWM
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    //This function configures to use a specific pin on the PWM module.
    GPIOPinConfigure(GPIO_PF0_M1PWM4);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    //This function sets the functionality of the GPIO pin within the GPIO module.
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | 
GPIO_PIN_3);
    //CONFIGURING PWM MODULE
    //This function configures the PWM generator on the module. The generator 
creates the pulses
    //needed for PWM output.
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | 
PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | 
PWM_GEN_MODE_NO_SYNC);
    //SET THE PERIOD OF THE PWM SIGNAL
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, pwm_ticks);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, pwm_ticks);
    //PWM1 DUTY CYCLE WIDTH CONFIGURATION FOR EACH PWM SIGNAL
    //THE NUMBER OF TICKS THAT IT HAS
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, pwm_out4_duty_cycle);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, pwm_out5_duty_cycle);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, pwm_out6_duty_cycle);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pwm_out7_duty_cycle);
    //NOW THAT WE HAVE CONFIGURED THE PWM VALUES WE MAY TURN ON THE GENERATOR
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    //NOW THAT THE CONFIGURATION IS FINISHED, THE PINS CAN BE TURNED ON
    PWMOutputState(PWM1_BASE, (PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | 
PWM_OUT_7_BIT ), true);
}
void adc_config(){
    //AIN0 -> PE3
    //AIN1 -> PE2
    //AIN2 -> PE1
    //AIN3 -> PE0
    // Configuración del ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                         //ENABLE 
ADC MODULE 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                        //TURN ON 
PORT E
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);                        //Set PE0 
as analog input because PE0 is AIN0, P. 801 datasheet
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);       
//Configuring module 0 with sequence 0 to read 1 channel
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0 | ADC_CTL_IE | 
ADC_CTL_END);             //Step 0, end in step0 and no further readings tivaware 
p.42
    ADCSequenceEnable(ADC0_BASE, 0);                                    //Enable 
the sequence 0 of ADC0 after configuring
    //CONFIGURATION OF ADC INTERRUPTION TO GET THE DATA
    ADCIntClear(ADC0_BASE, 0);                                          //Clear ADC
interrupt flag
    ADCIntEnable(ADC0_BASE,0);                                          //ENABLE 
ADC INTERRUPTIONS OF SEQUENCE 0
    ADCIntEnableEx(ADC0_BASE,ADC_INT_SS0 );                             //ADC 
INTERRUPT SOURCE ENABLE (ENABLED SEQUENCE 0)
    ADCIntRegister(ADC0_BASE,0,&adc_handler);                           //FUNCTION 
TO CALL WHEN INTERRUPT TRIGGERS
}
void timer0_config(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                       //TURN ON 
THE TIMER 0 MODULE
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);                    //CONFIGURE
AS FULL WIDTH PERIODIC TIMER
    TimerLoadSet(TIMER0_BASE, TIMER_A, 
(uint32_t)(SysCtlClockGet()/sample_freq)); //Configure timer A, with sample 
frequency
    //Registramos la función a habilitar para la interrupción
    IntRegister(INT_TIMER0A, &Timer0IntHandler);
    IntEnable(INT_TIMER0A);                                             //Enable 
interrupt for timer0 A
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                    //Enable 
timer interrupt flags (Timeout) because we want to count 1 period
    TimerEnable(TIMER0_BASE, TIMER_A);                                  //TURN ON 
THE TIMER READY TO COUNT
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}
void interrupt_config(){
    IntEnable(ADC_INT_SS0);                                             //ENABLE 
ADC0 INTERRUPT
    IntMasterEnable();                                                  //ENABLE 
MASTER INTERRUPTS
}
void uart_config(){
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, SysCtlClockGet());
    //SUCCESSFUL START MESSAGE
    //UARTprintf("Ready\n");
}
//*****************************************************************************
//                      LQR CONTROLLER
void LQR(){
    //Regulation
    //  matlab  ic    icxss   matlab vc    vcxss   matlab solved
    u = -klqr1*(x11 - xss1) - klqr2*(x21 - xss2) + uss; //u es el ciclo de trabajo 
en decimal ej. 0.15,0.5,0.9
    //OBSERVER CALCULATIONS FOR IC
    // ic is x11
    //x21 is the voltage of capacitor and x11 is the current of inductors ic2
    ic_obs = aobs11*previousx11+aobs12*x21 + bobs11*u + bobs12*x21;
    x11 = previousx11 + Ts*(ic_obs);
    if (u <= 0.15){
        u = 0.15; //this is 15% of 40,000
    }
    if (u >= 0.9) {
        u = 0.9; //this is 90% of 40,000
    }
    u_int = (uint32_t) (u*pwm_ticks);
    previousx11 = x11;
    //Update PWM later when the above is correct
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, u_int); //PF0
    return;
}
//*****************************************************************************
//*****************************************************************************
//HANDLERS
void adc_handler(){
    //INTERRUPT FUNCTION OF THE ADC SEQUENCE 0 CALL
    //READ VALUE OF AIN0 AND STORE IT IN A VARIABLE
    ADCSequenceDataGet(ADC0_BASE, 0, &Cap_value); //Store it in Cap_value a 32 bit 
integrer type
    x21 = (vs/4095)*(Cap_value); //convert from 0 - 4095 integrer to float 0 - 4.0 
v aprox, this stores capacitor true value in V
    //*****************************************************************************
    //              Here we execute the control system
    LQR();
    //*****************************************************************************
    ADCIntClear(ADC0_BASE, 0);                          //just in case we clear the
interrupt flag
    //UARTprintf("ADC reading");                  
//*********************************************************************************
***comment this line later
}
void Timer0IntHandler(void){
    ADCProcessorTrigger(ADC0_BASE,0);                                   //MAKE AN 
ADC READING
    /*
    pwm_out4_duty_cycle = pwm_out4_duty_cycle + 10;
    if (pwm_out4_duty_cycle==20000){
        pwm_out4_duty_cycle = 0;
    }
    */
    //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, pwm_out4_duty_cycle); //PF0
    //UARTprintf("Timer0");
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}
//*****************************************************************************
//TO TRIGGER AN ADC READING USE ADCProcessorTrigger(ADC0_BASE,0);
//TO MODIFY THE DUTY CYCLE OF PF0 USE THIS FUNCTION pwm_out4_duty_cycle = 
1*pwm_ticks; AND USE A CAST
//UNTESTED: pwm_out4_duty_cycle = (uint32_t) (% of duty cycle in 
decimal)*pwm_ticks; //PF0
