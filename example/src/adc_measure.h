#ifndef ADC_MEASURE_H
#define ADC_MEASURE_H

#ifdef __cplusplus
extern "C"
{
#endif

#define ADC_INPUT_PIN           18
int adc_read(void);
void adc_init(void);
#ifdef __cplusplus
}
#endif

#endif // ADC_MEASURE_H
