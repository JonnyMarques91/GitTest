
#include <asf.h>

/* Booleanas */
#define	TRUE	1
#define FALSE	0

/* Máscaras do Display */
#define ILI93XX_LCD_CS      1
#define BACKLIGHT_TIME		30	// Segundos

/* Máscaras da Interrupção */
#define TC			TC0
#define CHANNEL		0
#define ID_TC		ID_TC0
#define TC_Handler  TC0_Handler
#define TC_IRQn     TC0_IRQn

/* Máscaras - Potência da Bomba (Ajustar Empiricamente) */
/*			Duty Cycle Update (32bits)					*/
#define MAX_DIGITAL 4095
#define BOMBA_Off	4095	//   0%
#define BOMBA_Low	2731	// ~25%
#define BOMBA_Med	1365	// ~50%
#define BOMBA_Max	0		// 100%
#define DUTY_p25	BOMBA_Low

/* Máscaras - Nível de Umidade (Ajustar Empiricamente) */
/*			Duty Cycle Update (32bits)				   */
#define UMID_Null	U0	//  0% 
#define UMID_Norm	U2	//  75%
#define UMID_High	U3	// 100%

/* Máscaras PIO */

/* Protótipos de Sub-Rotinas e Funções */
void presence_cfg();
void presence_interrupt();
void bomba_control();
void leitura_umidade();
void display_update();
void PWM_init();

/* Declaração de variáveis Globais */
uint16_t uCNTseg, uCNTbmb, uCNTpresence, uTimePresence, uPresence_sts;
uint32_t duty_cycle_bomb;
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

/**
 *  Configure Timer Counter 0 to generate an interrupt every ...
 */
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
	
	// Enable interrupts for this TC, and start the TC.
	tc_enable_interrupt(TC,	CHANNEL, TC_IER_CPCS);
	tc_start(TC, CHANNEL);
}

/* Rotina de Interrupção - TC */
void TC_Handler(void)
{

	tc_get_status(TC,CHANNEL);
	bomba_control();	//Chama Controle da Bomba
	if (uPresence_sts == TRUE)
	{
		uTimePresence++;
		LED_On(LED1_GPIO);
		if (uTimePresence>BACKLIGHT_TIME)
		{
			uPresence_sts = 0;
			uTimePresence = 0;
			aat31xx_disable_backlight();
			LED_Off(LED1_GPIO);
		}
	}
	else
	{
		uPresence_sts = 0;
		uTimePresence = 0;
		aat31xx_disable_backlight();
		LED_Off(LED1_GPIO);
	}
	
//	LED_Off(LED1_GPIO);
// 	pio_set_pin_low(LED0_GPIO);
//	LED_Toggle(LED1_GPIO);
	display_update();	//Atualiza Display
}

/* Rotina de inicialização do PWM */
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

/* Configuração da Interrupção - Sensor de Presença */
void presence_cfg()
{
	pio_set_input(PIOB, PIO_PB3, PIO_PULLUP | PIO_DEBOUNCE);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB3, PIO_IT_RISE_EDGE, presence_interrupt);
	pio_enable_interrupt(PIOB, PIO_PB3);
	NVIC_SetPriority (PIOB_IRQn,14);
	NVIC_EnableIRQ(PIOB_IRQn);
}

int main (void)
{
	/* Chamada de rotinas de inicialização */
	sysclk_init();
	board_init();
	configure_lcd();
	PWM_init();
	
	presence_cfg();
	tc_config(1);
	
	/** Draw text on the LCD */
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(10,  20, (uint8_t *)"Duty  Cycle: ");
	ili93xx_draw_string(10,  50, (uint8_t *)"Temperature: ");
	ili93xx_draw_string(10,  80, (uint8_t *)"   Humidity: ");
	ili93xx_draw_string(10, 110, (uint8_t *)"    G. Hum.:");
	ili93xx_draw_string(10, 140, (uint8_t *)"   Presence:");

	while(1)
	{
		
	}
}

/* Ação - Interrupção do Sensor de Presença */
/*
	- Deve chamar rotinas de leitura de TODOS os sensores;
	- Enviar dados via Bluetooth para o celular do usuário;
*/
void presence_interrupt()
{
//	LED_Toggle(LED0_GPIO);	
//	LED_On(LED1_GPIO);
//	tc_config(0.1);
//	PIOB->PIO_PSR=(1<<3);

	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);
	uPresence_sts = TRUE;
}

void bomba_control(void)
{	
	if (uCNTbmb<10)	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = 4095-uCNTbmb*409;
	else uCNTbmb = 0;
 	uCNTbmb++;	
}

void display_update(void)
{
	uint32_t Actual_Duty_Cycle;
	
	Actual_Duty_Cycle = (PWM->PWM_CH_NUM[0].PWM_CDTY);
	Actual_Duty_Cycle = (Actual_Duty_Cycle * 100)/MAX_DIGITAL;
	
	char buffer1[10], buffer2[10], buffer3[10], buffer4[10], buffer5[10];
	sprintf (buffer1, "%d%%", Actual_Duty_Cycle);
// 	sprintf (buffer2, "%d%%", Actual_Duty_Cycle);
// 	sprintf (buffer3, "%d%%", Actual_Duty_Cycle);
 	sprintf (buffer4, "%d", uTimePresence);	//teste
	sprintf (buffer5, "%s", sPresence_sts);
	
	// Desenha retângulo
	ili93xx_set_foreground_color(COLOR_SKYBLUE);
	ili93xx_draw_filled_rectangle(160, 0, 240, 320);
	
	// Escreve uma String no LCD na posição 140, 180
	ili93xx_set_foreground_color(COLOR_BLACK);
	ili93xx_draw_string(170, 20, (uint8_t*) buffer1);	//(x32, y32, string)
// 	ili93xx_draw_string(170, 50, (uint8_t*) buffer2);	//(x32, y32, string)
// 	ili93xx_draw_string(170, 80, (uint8_t*) buffer3);	//(x32, y32, string)
 	ili93xx_draw_string(170, 110, (uint8_t*) buffer4);	//(x32, y32, string)
 	if (uPresence_sts == TRUE)	ili93xx_draw_string(170, 140,"YES");	//(x32, y32, string)
 	else						ili93xx_draw_string(170, 140,"NO ");	//(x32, y32, string)
}