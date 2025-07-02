
/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

/* Demo application includes. */
#include "BlockQ.h"
#include "crflash.h"
#include "blocktim.h"
#include "integer.h"
#include "comtest2.h"
#include "partest.h"
#include "timertest.h"

// Includes proprii
#include "semphr.h"
#include "adcDrv1.h"
#include "ds18s20.h"
#include "lcd_driver.h"
#include "new_serial.h"
#include "libq.h"

/* Demo task priorities. */
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainCOM_TEST_PRIORITY				( 2 )

/* The check task may require a bit more stack as it calls sprintf(). */
#define mainCHECK_TAKS_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )

/* The execution period of the check task. */
#define mainCHECK_TASK_PERIOD				( ( portTickType ) 3000 / portTICK_RATE_MS )

/* The number of flash co-routines to create. */
#define mainNUM_FLASH_COROUTINES			( 5 )

/* Baud rate used by the comtest tasks. */
//#define mainCOM_TEST_BAUD_RATE				( 19200 )
#define mainCOM_TEST_BAUD_RATE				( 9600 )

// Definire lungime coada UART1
#define comBUFFER_LEN						( 10 )

/* We should find that each character can be queued for Tx immediately and we
don't have to block to send. */
#define comNO_BLOCK					( ( portTickType ) 0 )

/* The Rx task will block on the Rx queue for a long period. */
#define comRX_BLOCK_TIME			( ( portTickType ) 0xffff )
#define comTX_BLOCK_TIME			( ( portTickType ) 0xffff )

/* The LED used by the comtest tasks.  mainCOM_TEST_LED + 1 is also used.
See the comtest.c file for more information. */
#define mainCOM_TEST_LED					( 6 )

/* The frequency at which the "fast interrupt test" interrupt will occur. */
#define mainTEST_INTERRUPT_FREQUENCY		( 20000 )

/* The number of processor clocks we expect to occur between each "fast
interrupt test" interrupt. */
#define mainEXPECTED_CLOCKS_BETWEEN_INTERRUPTS ( configCPU_CLOCK_HZ / mainTEST_INTERRUPT_FREQUENCY )

/* The number of nano seconds between each processor clock. */
#define mainNS_PER_CLOCK ( ( unsigned short ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Dimension the buffer used to hold the value of the maximum jitter time when
it is converted to a string. */
#define mainMAX_STRING_LENGTH				( 20 )

// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC);
// Enable Clock Switching and Configure
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF);		// FRC + PLL
//_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);		// XT + PLL
_FWDT(FWDTEN_OFF); 		// Watchdog Timer Enabled/disabled by user software

/*
 * Setup the processor ready for the demo.
 */
static void prvSetupHardware( void );

xSemaphoreHandle semphr_AppStatus, semphr_Manual, semphr_Auto; // semafoare pentru controlul modurilor
xQueueHandle queue_temp; // coada pentru transmiterea temperaturii intre task-uri

void vAppStatusTask(void *params)
{
	portTickType timer = 0; // timer pentru sincronizare la 200 ms
	for (;;)
	{
		_RB11 = ~(_RB0 | _RB11); // inverseaza starea LED-ului RB11 in functie de RB0 
		vTaskDelayUntil(&timer, 200); // delay fix pentru executie la fiecare 200 ms
	}
}

void vTempTask(void *params)
{
	portTickType timer = 0;
	float temp_float = 0; // temperatura citita de la senzor
	char temp_char[10]; // buffer pentru afisare temperatura

	for (;;)
	{
		temp_float = ds1820_read(); // citire temperatura DS18B20
		xQueueSend(queue_temp, &temp_float, 0); // trimite temperatura in coada

		if(xSemaphoreTake(semphr_Auto, 0) == pdTRUE) // daca suntem in mod automat
		{
			P1DC3 = temp_float * 625 - 12500; // calculeaza valoarea PWM corespunzatoare temperaturii
		}

		LCD_line(1); // selecteaza linia 1 pe LCD
		LCD_printf("TEMP:          "); // afiseaza prefix

		// conversia partii intregi a temperaturii din float in string
// (int)temp_float extrage partea intreaga (ex: 25.78 devine 25)
// _itoaQ15 transforma valoarea 25 in sirul de caractere "25"
		_itoaQ15((int)temp_float, temp_char); // ex: pentru 25.78 ? "25"
		LCD_printf(temp_char);
		LCD_printf("."); // separator zecimal

		// prima cifra zecimala: (ex: 25.78 -> (25.78 - 25) * 10 = 7.8 ? (int)7.8 = 7)
// scoatem partea zecimala din float si o multiplicam cu 10 pentru a obtine prima cifra
		_itoaQ15((int)((temp_float - (int)temp_float) * 10), temp_char); 
		LCD_printf(temp_char);

		// a doua cifra zecimala: (ex: 25.78 * 100 = 2578; temp_float * 10 = 257.8 ? (int) = 257 ? 257 * 10 = 2570; 2578 - 2570 = 8)
// extrage ultima cifra din cele doua zecimale multiplicate cu 100
		_itoaQ15((int)((temp_float * 100) - ((int)(temp_float * 10) * 10)), temp_char);
		LCD_printf(temp_char);

		vTaskDelayUntil(&timer, 2000); // delay fix de 2 secunde
	}
}

void vAdcTask(void *params)
{
	portTickType timer = 0;

	int adc_raw = 0;
	float adc_float = 0;
	char adc_char[10];

	for (;;)
	{
		adc_raw = ADC1BUF0;

		if(xSemaphoreTake(semphr_Manual, 0) == pdTRUE)
		{
			P1DC3 = (((long)adc_raw * 6250) / 4095);
		}

		adc_float = (adc_raw * 3.3f) / 4095.0f;


		LCD_line(3);
		LCD_printf("VOUT:           ");

		_itoaQ15((int)adc_float, adc_char);
		LCD_printf(adc_char);
		LCD_printf(".");

		_itoaQ15((int)((adc_float - (int)adc_float) * 10), adc_char);
		LCD_printf(adc_char);

		_itoaQ15((int)((adc_float * 100) - ((int)(adc_float * 10) * 10)), adc_char);
		LCD_printf(adc_char);

		vTaskDelayUntil(&timer, 100);
	}
}

volatile int mod_lucru = 1; // 1 = automat, 0 = manual
extern xQueueHandle queue_temp;



void vAdcTask(void *params)
{
    portTickType timer = 0; // timer pentru executie periodica
    int adc_raw = 0; // valoare ADC intre 0 si 4095
    float adc_float = 0; // valoare convertita in volti
    char adc_char[10]; // buffer pentru afisare

    for (;;)
    {
        adc_raw = ADC1BUF0; // citeste valoarea curenta din ADC

        if (xSemaphoreTake(semphr_Manual, 0) == pdTRUE) // daca suntem in mod manual
        {
            // calculeaza duty-cycle proportional din intervalul 0...6250
            P1DC3 = (((long)adc_raw * 6250) / 4095);
        }

        // converteste valoarea ADC in tensiune reala:
        // 4095 ? 3.3V, 0 ? 0V, formula: (valoare_ADC * Vref) / 4095
        // unde Vref este tensiunea de referinta (3.3V in cazul de fata)
        adc_float = (adc_raw * 3.3f) / 4095.0f;

        LCD_line(3); // selecteaza linia 3 a LCD-ului
        LCD_printf("VOUT:           ");

        _itoaQ15((int)adc_float, adc_char); // partea intreaga
        LCD_printf(adc_char);
        LCD_printf(".");

        _itoaQ15((int)((adc_float - (int)adc_float) * 10), adc_char); // prima cifra zecimala
        LCD_printf(adc_char);

        _itoaQ15((int)((adc_float * 100) - ((int)(adc_float * 10) * 10)), adc_char); // a doua cifra zecimala
        LCD_printf(adc_char);

        vTaskDelayUntil(&timer, 100); // asteapta 100 ms
    }
}

// Task care primeste comenzi prin UART si afiseaza/gestioneaza starea aplicatiei
void vSerialMenuTask(void *params) {
    portTickType timer = 0; // timer pentru executie periodica
    unsigned char rxChar; // caracter receptionat
    float temp_float = 0; // temperatura pentru afisare
    unsigned char temp_char[10]; // buffer pentru conversie

    const char *meniu =
        "\r\n--- Meniu Principal ---\r\n"
        "m - Interogare mod de lucru\r\n"
        "c - Comutare mod automat/manual\r\n"
        "t - Interogare temperatura\r\n"
        "h - Afisare ajutor\r\n"
        "------------------------\r\n";

    vSerialPutString(NULL, meniu, comNO_BLOCK); // afiseaza meniul initial
    LCD_line(4);
    LCD_printf("Ultima comanda:    ");

    for (;;) {
        LCD_line(2);
        LCD_printf("MODE:         ");
        if (mod_lucru) {
            LCD_printf("  auto");
            xSemaphoreGive(semphr_Auto); // semafor pentru mod auto
            _RB1 = 0;
        } else {
            LCD_printf("manual");
            xSemaphoreGive(semphr_Manual); // semafor pentru mod manual
            _RB1 = 1;
        }

        xQueueReceive(queue_temp, &temp_float, 0); // citeste temperatura din coada

        if (xSerialGetChar(NULL, &rxChar, 0)) {
            char buf_lcd[2] = { (char)rxChar, '\0' };
            LCD_line(4);
            LCD_printf("LASTCOMM:       ");
            LCD_printf(buf_lcd);

            switch (rxChar) {
                case 'm':
                    vSerialPutString(NULL, "Mod curent: ", comNO_BLOCK);
                    vSerialPutString(NULL, mod_lucru ? "auto\r\n" : "manual\r\n", comNO_BLOCK);
                    break;
                case 'c':
                    mod_lucru = !mod_lucru;
                    vSerialPutString(NULL, "Mod comutat.\r\n", comNO_BLOCK);
                    break;
                case 't':
                    _itoaQ15((int)temp_float, temp_char);
                    vSerialPutString(NULL, (char *)temp_char, comNO_BLOCK);
                    vSerialPutString(NULL, ".", comNO_BLOCK);
                    _itoaQ15((int)((temp_float - (int)temp_float) * 10), temp_char);
                    vSerialPutString(NULL, (char *)temp_char, comNO_BLOCK);
                    _itoaQ15((int)((temp_float * 100) - ((int)(temp_float * 10) * 10)), temp_char);
                    vSerialPutString(NULL, (char *)temp_char, comNO_BLOCK);
                    vSerialPutString(NULL, "\r\n", comNO_BLOCK);
                    break;
                case 'h':
                    vSerialPutString(NULL, meniu, comNO_BLOCK);
                    break;
                default:
                    vSerialPutString(NULL, "Comanda necunoscuta.\r\n", comNO_BLOCK);
                    break;
            }
        }
        vTaskDelayUntil(&timer, 100); // asteapta 100 ms
    }
}



void __attribute__ ((interrupt, no_auto_psv)) _INT0Interrupt(void)
{
	_RB0 = ~_RB0;

	_INT0IF = 0;
} 

int main( void )
{
	// Configure any hardware required for this demo.
	prvSetupHardware();

	TRISB = 0x0000;
	_TRISB7 = 1; // RB7 este setat ca intrare
	PORTB = 0xF000;
	
	_INT0IF = 0; // Resetem flagul coresp. intreruperii INT0
	_INT0IE = 1; // Se permite lucrul cu intreruperea INT0
	_INT0EP = 1; // Se stabileste pe ce front se genereaza INT0 
	xTaskCreate(vAppStatusTask, (signed portCHAR *) "vAppStatusTask", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(vTempTask, (signed portCHAR *) "vTempTask", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(vAdcTask, (signed portCHAR *) "vAdcTask", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
	xTaskCreate(vSerialMenuTask, (signed portCHAR *) "vSerialMenuTask", configMINIMAL_STACK_SIZE * 2, NULL, 6, NULL);

	vSemaphoreCreateBinary(semphr_AppStatus);
	vSemaphoreCreateBinary(semphr_Manual);
	vSemaphoreCreateBinary(semphr_Auto); 
	
	queue_temp = xQueueCreate(1, sizeof(float));

	// Finally start the scheduler.
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

void initPLL(void)
{
// Configure PLL prescaler, PLL postscaler, PLL divisor
	PLLFBD = 41; 		// M = 43 FRC
	//PLLFBD = 30; 		// M = 32 XT
	CLKDIVbits.PLLPOST=0; 	// N1 = 2
	CLKDIVbits.PLLPRE=0; 	// N2 = 2

// Initiate Clock Switch to Internal FRC with PLL (NOSC = 0b001)
	__builtin_write_OSCCONH(0x01);	// FRC
	//__builtin_write_OSCCONH(0x03);	// XT
	__builtin_write_OSCCONL(0x01);

// Wait for Clock switch to occur
	while (OSCCONbits.COSC != 0b001);	// FRC
	//while (OSCCONbits.COSC != 0b011);	// XT

// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};
}

void init_PWM1()
{
//lucram cu RB10 -> PWM1H3
 P1TCONbits.PTOPS = 0; // Timer base output scale
 P1TCONbits.PTMOD = 0; // Free running
 P1TMRbits.PTDIR = 0; // Numara in sus pana cand timerul = perioada
 P1TMRbits.PTMR = 0; // Baza de timp
 /*
	Tcy=25ns;
 	T=10ms;
 	fu=20%;
  
  Perioada obtinuta trebuie sa fie mai mica decat 65536/2.
  P1TPER=T/Tcy=10ms/25ns=...=400000;
  cu prescaler de 64=>400000/64=  6250  <65536/2=P1TPER;
 
  fu=(d/T)*100 => d=(fu*T)/100 =...=2ms;
  P1DC1=(d/Tcy)*2=...=160000
  cu prescaler 64=> 160000/64=  2500  <65536/2=P1DC1*/

 P1DC3 = 2500;
 P1TPER= 6250;

 _PWM1MD = 0; //PWM1 module is enabled

 PWM1CON1bits.PMOD3 = 1; // Canalele PWM3H si PWM3L sunt independente
 PWM1CON1bits.PEN3H = 1; // Pinul PWM1H setat pe iesire PWM

 P1TCONbits.PTCKPS=0b11;

 PWM1CON2bits.UDIS = 0; // 1 da disable la update pentru semnale pwm

 P1TCONbits.PTEN = 1; /* Enable the PWM Module */
}

static void prvSetupHardware( void )
{
	ADPCFG = 0xFFFF;				//make ADC pins all digital - adaugat
	vParTestInitialise();
	initPLL();
	initTmr3();
	init_PWM1();
	initAdc1();
	_CN6PUE = 1;
	ONE_WIRE_DIR = 1;
	output_float();
	ONE_WIRE_PIN = 1;
	onewire_reset();
	LCD_init();
	xSerialPortInitMinimal( mainCOM_TEST_BAUD_RATE, comBUFFER_LEN );

}

void vApplicationIdleHook( void )
{
	/* Schedule the co-routines from within the idle task hook. */
	vCoRoutineSchedule();
}

