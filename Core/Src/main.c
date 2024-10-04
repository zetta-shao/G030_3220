#include <stdio.h>
#include "main.h"
#include "gpiodef.h"
#include "swi2c.h"
#include "swspi.h"
#include "INA3221.h"
#include "IP2365.h"
#include "sw35xx_s.h"
#include "ath20.h"
#include "lcd_fonts.h"
#include "ssd1306.h"
#include "lcd1602sw.h"
#include "st7920.h"

#define LCD_ST7920
//#define LCD_SSD1306

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;
int32_t devices = 0;
int32_t vref;
int8_t env_key1 = 0;
int8_t env_key2 = 0;
int8_t env_key3 = 0;
int8_t _val_ = 0;
swi2c_t si2c1={0}, si2c2={0};
swspi_t sspi1={0};
//swspi_t sspi2={0};
//swspi_t sspi3={0};
ina3221_t ina3221 = { 0 };
#ifdef LCD_ST7920
st7920_t lcd128 = { 0 };
#elif defined(LCD_SSD1306)
ssd1306_t lcd128 = { 0 };
#endif
stm32_gpio_t lcd_cs = { GPIOA, GPIO_PIN_4 }; //11
stm32_gpio_t lcd_rs = { GPIOA, GPIO_PIN_6 }; //13
//stm32_gpio_t lcd_mosi = { GPIOA, GPIO_PIN_1 }; //8
//stm32_gpio_t lcd_clk = { GPIOA, GPIO_PIN_2 }; //9
int32_t wV[3], wI[3];

#define deffont Font_5x8

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif /* USE_FULL_ASSERT */

__strfmt str_fmt = &str_3digitL;

void update_ina3221(ina3221_t *ina, void *display) {
	int8_t np;
	int32_t p, i;
	char str[64];
	lcddev_t *d = (lcddev_t *)display;
	int ypos[3] = { 8, 24, 40 };

	if(ina->pDev == NULL) {
		fontdraw_setpos(d, 0, 0);
		fontdraw_string(d, "no INA3221 delect    ");
		d->update(d);
		return;
	}

	for(i=0; i<3; i++) {
		sprintf(str, "%1ld:", i+1);
		fontdraw_setpos(d, 0, ypos[i]);
		fontdraw_stringFont(d, str, 1, &Font_8x16);

		np = str_fmt(wV[i], str);
		fontdraw_setpos(d, 16, ypos[i]);
		fontdraw_stringFont(d, str, np, &Font_8x16);

		np = str_fmt(wI[i], str);
		fontdraw_setpos(d, 56, ypos[i]);
		fontdraw_stringFont(d, str, np, &Font_8x16);

		p = (wV[i] * wI[i]) / 1000;
		np = str_fmt(p, str);
		fontdraw_setpos(d, 96, ypos[i]);
		fontdraw_stringFont(d, str, np, &Font_8x16);
	}
	d->update(d);
}

#define ADV_VREF_PREAMP (4096 * 1200) //(4096 * 1210)

void init_adc(ADC_HandleTypeDef *d) {
	uint32_t idx, res = 0;
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

	HAL_ADC_Start(d);
	for(idx=0; idx<4; idx++) {
		HAL_ADC_PollForConversion(d, 1); //wait for 1mS
		res += HAL_ADC_GetValue(d);
	}
	HAL_ADC_Stop(d);
	res >>= 2;

	vref = ADV_VREF_PREAMP / res;

	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

void update_adc(ADC_HandleTypeDef *d, lcddev_t *plcd) {
	char str[64];
	uint32_t res, idx;
	init_adc(d);

	HAL_ADC_Start(d);
	HAL_ADC_PollForConversion(d, 1); //wsit for 1mS
	res = HAL_ADC_GetValue(d);
	HAL_ADC_Stop(d);
	idx = __LL_ADC_CALC_TEMPERATURE(vref, res, LL_ADC_RESOLUTION_12B);

	fontdraw_setpos(plcd, 0, plcd->frameHeight - plcd->pFont->FontHeight);
	sprintf(str, "coretemp:%ld   ", idx);
	fontdraw_string(plcd, str);
}

void disp_title(lcddev_t *plcd) {
	fontdraw_setpos(plcd, 0, 0);
	fontdraw_string(plcd, "ch  ");
	fontdraw_setpos(plcd, 24, 0);
	fontdraw_string(plcd, "Vol    ");
	fontdraw_setpos(plcd, 64, 0);
	fontdraw_string(plcd, "Cur    ");
	fontdraw_setpos(plcd, 104, 0);
	fontdraw_string(plcd, "Wat ");
}

int main(void) {
	lcddev_t *plcd;
	HAL_Init();
	GPIOinit();
	swi2c_HWinit(&si2c1, &hi2c2);
	//{
	//	i2c_gpio_t clk={ SI2C1P, SI2C1L }, sda={ SI2C1P, SI2C1A };
	//	swi2c_SWinit(&si2c1, &clk, &sda);
	//}
	swspi_HWinit(&sspi1, &hspi1);
	//swspi_SWinit(&sspi2, &lcd_clk, &lcd_mosi, NULL);
	//swspi_setmode(&sspi1, 3);

	HAL_Delay(50);
	init_adc(&hadc1);
	ina3221_init(&ina3221, &si2c1);
	HAL_ADCEx_Calibration_Start(&hadc1);

#if defined(LCD_ST7920)
	st7920_init(&lcd128, &sspi1, NULL, &Font_6x8);
#elif defined(LCD_SSD1306)
	SSD1306_gpioinit4W2(&lcd128, &lcd_cs, &lcd_rs);
	SSD1306_init(&lcd128, &sspi1, &Font_6x8);
#endif
	plcd = &lcd128.d;
	disp_title(plcd);

  _val_ = 0;
  while (1) {
		if(ina3221.pDev == NULL) {
			if(ina3221_detect(&ina3221, &si2c1) != 0) {
				ina3221_init(&ina3221, &si2c1);
				disp_title(plcd);
			}
		}
	update_ina3221(&ina3221, plcd); 

	update_adc(&hadc1, plcd);
	HAL_Delay(500);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if(htim == &htim14) {
			HAL_GPIO_TogglePin(EVB_LED_P, EVB_LED);
			//HAL_GPIO_TogglePin(LCD_RSP, LCD_RS);
			wV[0] = ina3221_getAvgVol(&ina3221, 0);
			wI[0] = ina3221_getAvgCur(&ina3221, 0);
			wV[1] = ina3221_getAvgVol(&ina3221, 1);
			wI[1] = ina3221_getAvgCur(&ina3221, 1);
			wV[2] = ina3221_getAvgVol(&ina3221, 2);
			wI[2] = ina3221_getAvgCur(&ina3221, 2);
        }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	/*switch(GPIO_Pin) { //4-15
	case GPIO_PIN_13: env_key1=1; break;
	case GPIO_PIN_14: env_key2=1; break;
	case GPIO_PIN_15: env_key3=1; break;
	}*/
}
