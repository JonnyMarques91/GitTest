
#include <asf.h>

/* Booleanas */
#define	TRUE	1
#define FALSE	0

/* Máscaras de PIO */
#define LED_ILUM (1<<14)			//LED de iluminação

/* Máscaras do Display */
#define ILI93XX_LCD_CS      1
#define BACKLIGHT_TIME		60		//Tempo - Backligh aceso

/* Máscaras do Bluetooth */
#define BLUETOOTH_TIME		20		//Tempo - Envio de dados via bluetooth

/* Máscaras da Interrupção */
#define TC			TC0
#define CHANNEL		0
#define ID_TC		ID_TC0
#define TC_Handler  TC0_Handler
#define TC_IRQn     TC0_IRQn

/* Máscaras - ADC */
#define VOLT_REF_T		(3300)		//Tensão de referência ADC [mv]
#define TRACKING_TIME    15
#define TRANSFER_PERIOD  2
#define STARTUP_TIME ADC_STARTUP_TIME_4
#define MAX_DIGITAL_AD	(4095)		//Máximo valor digital - Outros sensores
#define MAX_DIGITAL_T	(4095)		//Máximo valor digital - Temperatura
#define ADC_CHANNEL		5			//PB1  AD5
#define ADC_CHANNEL_L	6			//PB2 AD6
#define N_AMOSTRAS_T	10			//Nº de Amostras - Temperatura

/* Máscaras - Potência da Bomba (Ajustado Arbitrariamente)  */
#define MAX_DIGITAL_DC	4095
#define BOMBA_Off		0			//   0%
#define BOMBA_Low		1365		// ~25%
#define BOMBA_Med		3800		// ~50%
#define BOMBA_Max		4095		// 100%

/* Máscaras - Nível de Umidade (Ajustado Arbitrariamente)   */ //limpar
/*			  Duty Cycle Update (32bits)				    */
#define TERRA_SECA	20				//   0% 
#define TERRA_UMID	40				//  50%
#define TERRA_ENX	80				//  80%
#define TERRA_MAX	95				//  95%

/* Máscaras - UART */
#define CONF_UART				UART0
#define CONF_UART_BAUDRATE		9600
#define CONF_UART_CHAR_LENGTH	US_MR_CHRL_8_BIT
#define CONF_UART_PARITY		US_MR_PAR_NO
#define CONF_UART_STOP_BITS		US_MR_NBSTOP_1_BIT

/* Máscaras - Temporização */
#define  TEMPO_AMOSTRAGEM		10	// 1 Hora = 3600 Segundos

/* Nível Luminosidade */
#define LUM_MIN		60				// Luminosidade mínima %

/* Protótipos */
void presence_cfg();
void presence_interrupt();
void bomba_control();
void leitura_umidade();
void display_update();
void bluetooth_update();
void PWM_init();
void UART_init();

/* Declaração de variáveis Globais */
uint32_t uCNTbmb, AuxStandBy;

unsigned char uCNTsampl, uCNTtemp, uCNTpresence, uTimeSendBluetooth, uTimePresence, uBomba_Status, uPresence_sts, Actual_Humidity, Actual_Luminosity, Actual_Duty_Cycle;
float Actual_Temperature = 0.0, f_temp = 0.0;

struct ili93xx_opt_t g_ili93xx_display_opt;

/* Inicialização - LCD */
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

	/* Inicializa parâmetros do LCD */
	g_ili93xx_display_opt.ul_width = ILI93XX_LCD_WIDTH;
	g_ili93xx_display_opt.ul_height = ILI93XX_LCD_HEIGHT;
	g_ili93xx_display_opt.foreground_color = COLOR_BLACK;
	g_ili93xx_display_opt.background_color = COLOR_WHITE;

	/* Desliga Backlight */
	aat31xx_disable_backlight();

	/* Inicializa LCD */
	ili93xx_init(&g_ili93xx_display_opt);

	/*Nível do Backlight */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	ili93xx_set_foreground_color(COLOR_WHITE);
	ili93xx_draw_filled_rectangle(0, 0, ILI93XX_LCD_WIDTH,
	ILI93XX_LCD_HEIGHT);
	/* Liga LCD */
	ili93xx_display_on();
	ili93xx_set_cursor_position(0, 0);
	
	/* Escreve textos - LCD */
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(10,  20, (uint8_t *)" Duty  Cycle: ");
	ili93xx_draw_string(10,  50, (uint8_t *)" Temperature: ");
	ili93xx_draw_string(10,  80, (uint8_t *)"     G. Hum.: ");
	ili93xx_draw_string(10, 110, (uint8_t *)"    Stand-By: ");
	ili93xx_draw_string(10, 140, (uint8_t *)"Luminosidade: ");
	ili93xx_draw_string(10, 170, (uint8_t *)"  Sts. Bomba: ");
	ili93xx_draw_string(10, 200, (uint8_t *)"    Presence: ");
}

/* Configuração - Timer Counter 0 */
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
	NVIC_SetPriority(TC_IRQn, 5);
	NVIC_EnableIRQ(TC_IRQn);
	tc_enable_interrupt(TC,	CHANNEL, TC_IER_CPCS);
	tc_start(TC, CHANNEL);
}

/* Interrupção - TC */
void TC_Handler(void)
{
	uCNTbmb++;
	uTimeSendBluetooth++;
	
	
	tc_get_status(TC,CHANNEL);
	adc_start(ADC);	
	if (uPresence_sts == TRUE)
	{ 
		if (Actual_Luminosity < LUM_MIN) PIOB->PIO_SODR |= LED_ILUM;
		LED_On(LED1_GPIO);
		if (uTimePresence>BACKLIGHT_TIME)
		{
			uPresence_sts = 0;
			uTimePresence = 0;
			aat31xx_disable_backlight();
			PIOB->PIO_CODR |= LED_ILUM;
			LED_Off(LED1_GPIO);
		}
		uTimePresence++;
	}
	else
	{
		uPresence_sts = 0;
		uTimePresence = 0;
		aat31xx_disable_backlight(); 
		PIOB->PIO_CODR |= LED_ILUM;
		LED_Off(LED1_GPIO);
	}
	if (uTimeSendBluetooth>BLUETOOTH_TIME)
	{
		uTimeSendBluetooth = 0;
		bluetooth_update();
	}
	if ((uCNTbmb>TEMPO_AMOSTRAGEM)||(Actual_Humidity >= TERRA_MAX))
	{
		uCNTbmb = 0;
		bomba_control();
	}
	if ((TEMPO_AMOSTRAGEM-uCNTbmb) > 0)	AuxStandBy = (TEMPO_AMOSTRAGEM-uCNTbmb);
	else	AuxStandBy = 0;
	display_update();
}

/* Inicialização - PWM */
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

/* Inicialização - UART */
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

/* Configuração da Interrupção - Sensor de Presença */
void presence_cfg()
{
	pio_set_input(PIOB, PIO_PB3, PIO_PULLUP | PIO_DEBOUNCE);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB3, PIO_IT_RISE_EDGE, presence_interrupt);
	pio_enable_interrupt(PIOB, PIO_PB3);
	NVIC_SetPriority (PIOB_IRQn,14);
	NVIC_EnableIRQ(PIOB_IRQn);
}

/* Configuração das Interrupções - ADC */
void configure_adc(void)
{
	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, sysclk_get_cpu_hz(), 6400000, STARTUP_TIME);
	adc_configure_timing(ADC, TRACKING_TIME	, ADC_SETTLING_TIME_3, TRANSFER_PERIOD);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 0);
	adc_enable_channel(ADC, ADC_CHANNEL);
	adc_enable_channel(ADC, ADC_CHANNEL_L);
	adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);
	adc_enable_ts(ADC);							//Habilita sensor de temperatura	
	NVIC_SetPriority(ADC_IRQn, 16);
	NVIC_EnableIRQ(ADC_IRQn);
	adc_enable_interrupt(ADC, ADC_ISR_EOC5);	//Habilita interrupção (Umidade)
	adc_enable_interrupt(ADC, ADC_ISR_EOC6);	//Habilita interrupção (Luminosidade)
	adc_enable_interrupt(ADC, ADC_ISR_EOC15);	//Habilita interrupção (Temperatura)
}

/* Interrupção - A/D */
void ADC_Handler(void)
{	
	float aux_temp;
	int32_t l_vol;
	
    if (adc_get_status(ADC) & ADC_ISR_EOC5)
    {
    	Actual_Humidity = (MAX_DIGITAL_AD - adc_get_channel_value(ADC, ADC_CHANNEL))/10;
    	if (Actual_Humidity>100)			Actual_Humidity = 100;
    	if (uCNTsampl>TEMPO_AMOSTRAGEM)		bomba_control();
    }
	if (adc_get_status(ADC) & ADC_ISR_EOC6)		Actual_Luminosity = (100*adc_get_channel_value(ADC, ADC_CHANNEL_L))/MAX_DIGITAL_AD;
	if (adc_get_status(ADC) & ADC_ISR_EOC15)
	{		
		l_vol = adc_get_channel_value(ADC, ADC_TEMPERATURE_SENSOR) * VOLT_REF_T / MAX_DIGITAL_T;
		aux_temp = (float)(l_vol - 1440) * 0.21276 + 27.0;
		if (uCNTtemp<N_AMOSTRAS_T) 
		{
			f_temp += aux_temp;
			uCNTtemp++;
		}
		else
		{
			uCNTtemp = 0;
			Actual_Temperature = f_temp/N_AMOSTRAS_T;
			f_temp = 0;
		}	
	}
}

/* Inicialização - PIO */
void pio_init(void)
{
	PIOB->PIO_PER |= LED_ILUM;
	PIOB->PIO_OER |= LED_ILUM;
	PIOB->PIO_SODR |= LED_ILUM;
}

int main (void)
{
	/* Chamada de rotinas de inicialização */
	sysclk_init();
	board_init();
	pio_init();
	configure_lcd();
	configure_adc();
	PWM_init();
	UART_init();
	presence_cfg();
	tc_config(1);
	
	/* INIT - Variáveis globais */
	Actual_Duty_Cycle = 0;
	uBomba_Status = FALSE;
	uPresence_sts = FALSE;
	
	while(1)
	{
	
	}
}

/* Presença detectada */
void presence_interrupt(void)
{
	uPresence_sts = TRUE;
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);
	bluetooth_update();
}

/* Controle da bomba de água */
void bomba_control(void)
{
	if (Actual_Humidity < TERRA_SECA)	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = BOMBA_Max;
	else if ((Actual_Humidity >= TERRA_SECA) && (Actual_Humidity<TERRA_UMID))	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = BOMBA_Med;
	else	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = BOMBA_Off;
//	uCNTbmb = 0;
	display_update();	
}

/* Envia dados via Bluetooth */
/* Periódicamente ou quando detectada presença */
void bluetooth_update(void)
{
	printf("Duty Cycle: %d%%\n", Actual_Duty_Cycle);
	printf("Temperatura: %d graus Celsius\n", (uint32_t)Actual_Temperature);
	printf("Umidade (Solo): %i%%\n", Actual_Humidity);
	printf("Stand-By: %ds\n", AuxStandBy);
	printf("Luminosidade: %d%%\n", Actual_Luminosity);
	if (uBomba_Status == TRUE) printf("BOMBA LIGADA!\n");
	if (uPresence_sts == TRUE) printf("PRESENCA DETECTADA!\n");
	printf("\n");
}

/* Display - Atualização de dados */
void display_update(void)
{	
	Actual_Duty_Cycle = ((PWM->PWM_CH_NUM[0].PWM_CDTY)*100)/MAX_DIGITAL_DC;
	if (Actual_Duty_Cycle == 0)	uBomba_Status = FALSE;
	else						uBomba_Status = TRUE;
		
	char buffer1[10], buffer2[10], buffer3[10], buffer4[10], buffer5[10], buffer6[10];
	sprintf (buffer1, "%d%%", Actual_Duty_Cycle);	//DC
	sprintf (buffer2, "%2.1f", Actual_Temperature);	//Temperatura
 	sprintf (buffer4, "%d%%", Actual_Humidity);		//Umidade-Solo
	sprintf (buffer5, "%ds", AuxStandBy);			//Stand-By até próxima ação
	sprintf (buffer6, "%d%%", Actual_Luminosity);	//Luminosidade
		
	// Desenha retângulo
	ili93xx_set_foreground_color(COLOR_SKYBLUE);
	ili93xx_draw_filled_rectangle(170, 0, 240, 320);
	
	// Escreve uma String no LCD (x32, y32, string)
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(180, 20, (uint8_t*) buffer1);					// DC
	ili93xx_draw_string(180, 50, (uint16_t*) buffer2);					// Temperatura
 	ili93xx_draw_string(180, 80, (uint8_t*) buffer4);					// Umidade-Solo
 	ili93xx_draw_string(180, 110, (uint8_t*) buffer5);					// Luminosidade
	ili93xx_draw_string(180, 140, (uint8_t*) buffer6);					// Tempo Display 
	if (uBomba_Status == TRUE)	ili93xx_draw_string(180, 170,"ON");		//
	else						ili93xx_draw_string(180, 170,"OFF");	//
 	if (uPresence_sts == TRUE)	ili93xx_draw_string(180, 200,"YES");	//
 	else						ili93xx_draw_string(180, 200,"NO ");	//
}