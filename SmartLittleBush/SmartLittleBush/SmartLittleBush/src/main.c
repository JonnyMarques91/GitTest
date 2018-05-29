
#include <asf.h>

/* Booleanas */
#define	TRUE	1
#define FALSE	0

/* M�scaras do Display */
#define ILI93XX_LCD_CS      1
#define BACKLIGHT_TIME		60		//Tempo - Backligh aceso

/* M�scaras do Bluetooth */
#define BLUETOOTH_TIME		20		//Tempo - Envio de dados via bluetooth

/* M�scaras da Interrup��o */
#define TC			TC0
#define CHANNEL		0
#define ID_TC		ID_TC0
#define TC_Handler  TC0_Handler
#define TC_IRQn     TC0_IRQn

/* M�scaras - ADC */
#define VOLT_REF        (5000)		//Reference voltage for ADC,in mv.
#define TRACKING_TIME    15
#define TRANSFER_PERIOD  2
#define STARTUP_TIME ADC_STARTUP_TIME_4
#define MAX_DIGITAL_AD	4095		//M�ximo valor digital
#define ADC_CHANNEL		5			//PB1  AD5
#define ADC_CHANNEL_L	12			//PC12 AD12

/* M�scaras - Pot�ncia da Bomba (Ajustado Arbitrariamente)  */
/*			  Duty Cycle Update (32bits)					*/
#define MAX_DIGITAL_DC	4095
#define BOMBA_Off		4095		//   0%
#define BOMBA_Low		2731		// ~25%
#define BOMBA_Med		1365		// ~50%
#define BOMBA_Max		0			// 100%
#define DUTY_p25		BOMBA_Low

/* M�scaras - N�vel de Umidade (Ajustado Arbitrariamente)   */
/*			  Duty Cycle Update (32bits)				    */
#define TERRA_SECA	20				//   0% 
#define TERRA_UMID	40				//  50%
#define TERRA_ENX	80				//  80%
#define TERRA_MAX	95				//  95%

/* M�scaras - UART */
#define CONF_UART				UART0
#define CONF_UART_BAUDRATE		9600
#define CONF_UART_CHAR_LENGTH	US_MR_CHRL_8_BIT
#define CONF_UART_PARITY		US_MR_PAR_NO
#define CONF_UART_STOP_BITS		US_MR_NBSTOP_1_BIT

/* M�scaras - Temporiza��o */
#define  TEMPO_AMOSTRAGEM		10	// 1 Hora [Segundos]

/* Prot�tipos */
void presence_cfg();
void presence_interrupt();
void bomba_control();
void leitura_umidade();
void display_update();
void bluetooth_update();
void PWM_init();
void UART_init();

/* Declara��o de vari�veis Globais */
uint16_t uCNTsampl, uCNTseg, uCNTbmb, uCNTpresence, uTimePresence, uTimeSendBluetooth, uPresence_sts, uBomba_Status, AuxStandBy;
uint32_t duty_cycle_bomb, Actual_Duty_Cycle, Actual_Humidity, Actual_Luminosity;
char sPresence_sts[10];

struct ili93xx_opt_t g_ili93xx_display_opt;

void configure_lcd()
{
	/** Enable peripheral clock */
	pmc_enable_periph_clk(ID_SMC);

	/** Configure SMC interface for Lcd */
	smc_set_setup_timing(SMC, ILI93XX_LCD_CS, SMC_SETUP_NWE_SETUP(2)
	| SMC_SETUP_NCS_WR_SETUP(2)
	| SMC_SETUP_NRD_SETUP(2)
	| SMC_SETUP_NCS_RD_SETUP(2));
	
	smc_set_pulse_timing(SMC, ILI93XX_LCD_CS, SMC_PULSE_NWE_PULSE(4)
	| SMC_PULSE_NCS_WR_PULSE(4)
	| SMC_PULSE_NRD_PULSE(10)
	| SMC_PULSE_NCS_RD_PULSE(10));
	
	smc_set_cycle_timing(SMC, ILI93XX_LCD_CS, SMC_CYCLE_NWE_CYCLE(10)
	| SMC_CYCLE_NRD_CYCLE(22));
	
	smc_set_mode(SMC, ILI93XX_LCD_CS, SMC_MODE_READ_MODE
	| SMC_MODE_WRITE_MODE);

	/** Initialize display parameter */
	g_ili93xx_display_opt.ul_width = ILI93XX_LCD_WIDTH;
	g_ili93xx_display_opt.ul_height = ILI93XX_LCD_HEIGHT;
	g_ili93xx_display_opt.foreground_color = COLOR_BLACK;
	g_ili93xx_display_opt.background_color = COLOR_WHITE;

	/** Switch off backlight */
	aat31xx_disable_backlight();

	/** Initialize LCD */
	ili93xx_init(&g_ili93xx_display_opt);

	/** Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(0, 0, ILI93XX_LCD_WIDTH,
	ILI93XX_LCD_HEIGHT);
	/** Turn on LCD */
	ili93xx_display_on();
	ili93xx_set_cursor_position(0, 0);
}

/* Configure Timer Counter 0 to generate an interrupt every ... */
static void tc_config(uint32_t freq_desejada)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t counts;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	
	pmc_enable_periph_clk(ID_TC);
	tc_find_mck_divisor( freq_desejada, ul_sysclk, &ul_div, &ul_tcclks,	BOARD_MCK);
	tc_init(TC, CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	counts = (ul_sysclk/ul_div)/freq_desejada;
	tc_write_rc(TC, CHANNEL, counts);
	NVIC_ClearPendingIRQ(TC_IRQn);
	NVIC_SetPriority(TC_IRQn, 5);	//5
	NVIC_EnableIRQ(TC_IRQn);
	// Enable interrupts for this TC, and start the TC.
	tc_enable_interrupt(TC,	CHANNEL, TC_IER_CPCS);
	tc_start(TC, CHANNEL);
}

/* Rotina de Interrup��o - TC */
void TC_Handler(void)
{
	uCNTbmb++;
	if ((TEMPO_AMOSTRAGEM-uCNTbmb) > 0)	AuxStandBy = (TEMPO_AMOSTRAGEM-uCNTbmb);
	else	AuxStandBy = 0;
	tc_get_status(TC,CHANNEL);
	adc_start(ADC);		//Inicia convers�o A/D	
	if (uPresence_sts == TRUE)
	{ 
		LED_On(LED1_GPIO);
		if (uTimePresence>BACKLIGHT_TIME)
		{
			uPresence_sts = 0;
			uTimePresence = 0;
			aat31xx_disable_backlight();
			LED_Off(LED1_GPIO);
		}
		uTimePresence++;
	}
	else
	{
		uPresence_sts = 0;
		uTimePresence = 0;
		aat31xx_disable_backlight();
		LED_Off(LED1_GPIO);
	}
	uTimeSendBluetooth++;
	if (uTimeSendBluetooth>BLUETOOTH_TIME)
	{
		uTimeSendBluetooth = 0;
		bluetooth_update();
	}
	if (uCNTbmb>TEMPO_AMOSTRAGEM)	bomba_control();	//Chama Controle da Bomba
	display_update();	//Atualiza Display
}

/* Rotina de inicializa��o do PWM */
void PWM_init(void)
{
	// disable the PIO (peripheral controls the pin)
	PIOA->PIO_PDR = PIO_PDR_P19;
	// select alternate function B (PWML0) for pin PA19
	PIOA->PIO_ABCDSR[0] |= PIO_ABCDSR_P19;
	PIOA->PIO_ABCDSR[1] &= ~PIO_ABCDSR_P19;
	// Enable the PWM peripheral from the Power Manger
	PMC->PMC_PCER0 = (1 << ID_PWM);
	// Select the Clock to run at the MCK (4MHz)
	PWM->PWM_CH_NUM[0].PWM_CMR = PWM_CMR_CPRE_MCK;
	// select the period 10msec
	PWM->PWM_CH_NUM[0].PWM_CPRD = 4096;// freq em khz
	// select the duty cycle
	PWM->PWM_CH_NUM[0].PWM_CDTY = 3500;
	// enable the channel
	PWM->PWM_ENA = PWM_ENA_CHID0;
}

/* Rotina de inicializa��o do UART */
void UART_init()
{
	static usart_serial_options_t usart_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS
	};
	usart_serial_init(CONF_UART, &usart_options);
	stdio_serial_init((Usart *)CONF_UART, &usart_options);
}

/* Configura��o da Interrup��o - Sensor de Presen�a */
void presence_cfg()
{
	pio_set_input(PIOB, PIO_PB3, PIO_PULLUP | PIO_DEBOUNCE);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB3, PIO_IT_RISE_EDGE, presence_interrupt);
	pio_enable_interrupt(PIOB, PIO_PB3);
	NVIC_SetPriority (PIOB_IRQn,14);
	NVIC_EnableIRQ(PIOB_IRQn);
}

void configure_adc(void)
{
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, sysclk_get_cpu_hz(), 6400000, STARTUP_TIME);
	adc_configure_timing(ADC, TRACKING_TIME	, ADC_SETTLING_TIME_3, TRANSFER_PERIOD);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
	adc_enable_channel(ADC, ADC_CHANNEL);
	adc_enable_channel(ADC, ADC_CHANNEL_L);	
	NVIC_SetPriority(ADC_IRQn, 16);	//16
	NVIC_EnableIRQ(ADC_IRQn);
//	adc_enable_interrupt(ADC, ADC_IER_DRDY);
	adc_enable_interrupt(ADC, ADC_ISR_EOC12);
 	adc_enable_interrupt(ADC, ADC_ISR_EOC5);
}

void ADC_Handler(void)
{	
//     	if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY)
//     	{
//     		Actual_Humidity = (MAX_DIGITAL_AD - adc_get_latest_value(ADC))/10;
//     		if (Actual_Humidity>100)	Actual_Humidity = 100;
//     		if (Actual_Humidity<0)		Actual_Humidity = 0;
//     		if (uCNTsampl>TEMPO_AMOSTRAGEM)		bomba_control();	//Chama Controle da Bomba
//     	}

    	if (adc_get_status(ADC) & ADC_ISR_EOC5)
    	{
    		Actual_Humidity = (MAX_DIGITAL_AD - adc_get_channel_value(ADC, ADC_CHANNEL))/10;
    		if (Actual_Humidity>100)			Actual_Humidity = 100;
    		if (Actual_Humidity<0)				Actual_Humidity = 0;
    		if (uCNTsampl>TEMPO_AMOSTRAGEM)		bomba_control();	//Chama Controle da Bomba
    	}
    	else if (adc_get_status(ADC) & ADC_ISR_EOC12)
    	{
    		/*Actual_Luminosity = (MAX_DIGITAL_AD - adc_get_channel_value(ADC, ADC_CHANNEL_L))/10;*/
    		Actual_Luminosity = /*(MAX_DIGITAL_AD -*/adc_get_channel_value(ADC, ADC_CHANNEL_L)/*)/10*/;
// 			if (Actual_Luminosity>100)			Actual_Luminosity = 100;
// 			if (Actual_Luminosity<0)			Actual_Luminosity = 0;
    	}
}

int main (void)
{
	unsigned char key_input;
	/* Chamada de rotinas de inicializa��o */
	sysclk_init();
	board_init();
	configure_lcd();
	configure_adc();
	PWM_init();
	UART_init();
	presence_cfg();
	tc_config(1);
	
	/* Draw text on the LCD */
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(10,  20, (uint8_t *)" Duty  Cycle: ");
	ili93xx_draw_string(10,  50, (uint8_t *)" Temperature: ");
	ili93xx_draw_string(10,  80, (uint8_t *)"    Humidity: ");
	ili93xx_draw_string(10, 110, (uint8_t *)"     G. Hum.: ");
	ili93xx_draw_string(10, 140, (uint8_t *)"    Stand-By: ");
	ili93xx_draw_string(10, 170, (uint8_t *)"Luminosidade: ");
	ili93xx_draw_string(10, 200, (uint8_t *)"  Sts. Bomba: ");
	ili93xx_draw_string(10, 230, (uint8_t *)"    Presence: ");

	while(1)
	{

	}
}

/* A��o - Interrup��o do Sensor de Presen�a */
/*
	- Deve chamar rotinas de leitura de TODOS os sensores;
	- Enviar dados via Bluetooth para o celular do usu�rio;
	- Obs: N�o altera funcionamento da bomba;
*/
void presence_interrupt()
{
	uPresence_sts = TRUE;
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);
	bluetooth_update();
}

void bomba_control(void)
{
	if (Actual_Humidity > TERRA_MAX)
	{
		PWM->PWM_CH_NUM[0].PWM_CDTYUPD = BOMBA_Off;
		uCNTbmb = 0;
	}
	if ((Actual_Humidity >= TERRA_ENX) && (Actual_Humidity < TERRA_MAX))	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = BOMBA_Low;
	if (Actual_Humidity < TERRA_UMID)	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = BOMBA_Med;
	if (Actual_Humidity < TERRA_SECA)	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = BOMBA_Max;
	//Testes
// 	if (uCNTbmb<10)	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = MAX_DIGITAL_DC-uCNTbmb*409;
// 	else uCNTbmb = 0;
//  	uCNTbmb++;	
}

/* Envia dados via Bluetooth */
/* Peri�dicamente ou quando detectada presen�a */
void bluetooth_update(void)
{
	printf("Duty Cycle: %d%%\n", Actual_Duty_Cycle);
	printf("Temperatura: \n");
	printf("Umidade: \n");
	printf("Umidade (Solo): %i%%\n", Actual_Humidity);
	printf("Stand-By: %ds\n", AuxStandBy);
	printf("Luminosidade: %ds\n", Actual_Luminosity);
	if (uBomba_Status == TRUE) printf("BOMBA LIGADA!\n");
	if (uPresence_sts == TRUE) printf("PRESENCA DETECTADA!\n");
	printf("\n");
}

void display_update(void)
{	
	Actual_Duty_Cycle = (PWM->PWM_CH_NUM[0].PWM_CDTY);
	Actual_Duty_Cycle = 100 - (Actual_Duty_Cycle * 100)/MAX_DIGITAL_DC;
	if (Actual_Duty_Cycle == 0)	uBomba_Status = FALSE;
	else						uBomba_Status = TRUE;
		
	char buffer1[10], buffer2[10], buffer3[10], buffer4[10], buffer5[10], buffer6[10];
	sprintf (buffer1, "%d%%", Actual_Duty_Cycle);	//DC
// 	sprintf (buffer2, "%d%%", Actual_Duty_Cycle);	//Temperatura
// 	sprintf (buffer3, "%d%%", Actual_Humidity);		//Umidade
 	sprintf (buffer4, "%d%%", Actual_Humidity);		//Umidade-Solo
	sprintf (buffer5, "%ds", AuxStandBy);			//Stand-By at� pr�xima a��o
	sprintf (buffer6, "%d", Actual_Luminosity);		//Luminosidade
 	
		
	// Desenha ret�ngulo
	ili93xx_set_foreground_color(COLOR_SKYBLUE);
	ili93xx_draw_filled_rectangle(160, 0, 240, 320);
	
	// Escreve uma String no LCD na posi��o 140, 180
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(170, 20, (uint8_t*) buffer1);	//(x32, y32, string) DC
 //	ili93xx_draw_string(170, 50, (uint8_t*) buffer2);	//(x32, y32, string) Temperatura
// 	ili93xx_draw_string(170, 80, (uint8_t*) buffer3);	//(x32, y32, string) Umidade
 	ili93xx_draw_string(170, 110, (uint8_t*) buffer4);	//(x32, y32, string) Umidade-Solo
 	ili93xx_draw_string(170, 140, (uint8_t*) buffer5);	//(x32, y32, string) Luminosidade
	ili93xx_draw_string(170, 170, (uint8_t*) buffer6);	//(x32, y32, string) Tempo Display 
	if (uBomba_Status == TRUE)	ili93xx_draw_string(170, 200,"ON");
	else						ili93xx_draw_string(170, 200,"OFF");
 	if (uPresence_sts == TRUE)	ili93xx_draw_string(170, 230,"YES");	//(x32, y32, string)
 	else						ili93xx_draw_string(170, 230,"NO ");	//(x32, y32, string)
}