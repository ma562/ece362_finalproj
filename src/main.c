#include "stm32f0xx.h"
#include "lcd.h"
#include <stdint.h>
#include "midi.h"
#include "midiplay.h"
#include "fifo.h"
#include "tty.h"
#include <stdio.h>

#define VOICES 15
//void enable_tty_interrupt(void);
void init_lcd_spi(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~ (GPIO_MODER_MODER8 | GPIO_MODER_MODER11 | GPIO_MODER_MODER14);
    GPIOB->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER11_0 | GPIO_MODER_MODER14_0);
    GPIOB->ODR |= (1<<8) | (1<<11) | (1<<14);
    GPIOB->MODER &= ~ (GPIO_MODER_MODER3 | GPIO_MODER_MODER5);
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER5_1);

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;


    // AFR's default is 0, which is what I need
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    SPI1->CR1 |= SPI_CR1_SPE;
}


void setup_buttons(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    //0, 1, 2, 3
    //set for inputs
    GPIOC->MODER &= ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    //pull downs
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR3);
    GPIOC->PUPDR |= (GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0 | GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0);

}

// An array of "voices".  Each voice can be used to play a different note.
// Each voice can be associated with a channel (explained later).
// Each voice has a step size and an offset into the wave table.
struct {
    uint8_t in_use;
    uint8_t note;
    uint8_t chan;
    uint8_t volume;
    int     step;
    int     offset;
} voice[VOICES];

// We'll use the Timer 6 IRQ to recompute samples and feed those
// samples into the DAC.
void TIM6_DAC_IRQHandler(void)
{
    // TODO: Remember to acknowledge the interrupt right here.
    TIM6->SR &= ~TIM_SR_UIF;

    int sample = 0;
    for(int x=0; x < sizeof voice / sizeof voice[0]; x++) {
        if (voice[x].in_use) {
            voice[x].offset += voice[x].step;
            if (voice[x].offset >= N<<16)
                voice[x].offset -= N<<16;
            sample += (wavetable[voice[x].offset>>16] * voice[x].volume) >> 4;
        }
    }
    sample = (sample >> 10) + 2048;
    if (sample > 4095)
        sample = 4095;
    else if (sample < 0)
        sample = 0;
    DAC->DHR12R2 = sample;
}

// Initialize the DAC so that it can output analog samples
// on PA4.  Configure it to be triggered by TIM6 TRGO.
void init_dac(void)
{
    // TODO: you fill this in.
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA -> MODER |= 3<<(2*4);

    RCC -> APB1ENR |= RCC_APB1ENR_DACEN;
    //DAC -> CR &= ~DAC_CR_EN1;
    //DAC->CR &= ~DAC_CR_BOFF1;
    DAC->CR &= ~DAC_CR_TSEL2;
    DAC->CR |= DAC_CR_TEN2;
    DAC->CR |= DAC_CR_EN2;


}

// Initialize Timer 6 so that it calls TIM6_DAC_IRQHandler
// at exactly RATE times per second.  You'll need to select
// a PSC value and then do some math on the system clock rate
// to determine the value to set for ARR.  Set it to trigger
// the DAC by enabling the Update Trigger in the CR2 MMS field.
void init_tim6(void)
{
    // TODO: you fill this in.
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // RATE = 20000
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->PSC = (uint32_t)0;
    TIM6->ARR = (uint32_t)48000000 / RATE - 1;

    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= TIM_CR2_MMS_1;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] |= 1<<TIM6_DAC_IRQn;

    NVIC_SetPriority(TIM6_DAC_IRQn,0);
}

// Find the voice current playing a note, and turn it off.
void note_off(int time, int chan, int key, int velo)
{
    int n;
    for(n=0; n<sizeof voice / sizeof voice[0]; n++) {
        if (voice[n].in_use && voice[n].note == key) {
            voice[n].in_use = 0; // disable it first...
            voice[n].chan = 0;   // ...then clear its values
            voice[n].note = key;
            voice[n].step = step[key];
            return;
        }
    }
}

// Find an unused voice, and use it to play a note.
void note_on(int time, int chan, int key, int velo)
{
    if (velo == 0) {
        note_off(time, chan, key, velo);
        return;
    }
    int n;
    for(n=0; n<sizeof voice / sizeof voice[0]; n++) {
        if (voice[n].in_use == 0) {
            voice[n].note = key;
            voice[n].step = step[key];
            voice[n].offset = 0;
            voice[n].volume = velo;
            voice[n].chan = chan;
            voice[n].in_use = 1;
            return;
        }
    }
}

void clear_voice_array() {
    int n;
    for(n=0; n<sizeof voice / sizeof voice[0]; n++) {
        voice[n].note = 0;
        voice[n].step = 0;
        voice[n].offset = 0;
        voice[n].volume = 0;
        voice[n].chan = 0;
        voice[n].in_use = 0;
    }
    return;
}

void set_tempo(int time, int value, const MIDI_Header *hdr)
{
    // This assumes that the TIM2 prescaler divides by 48.
    // It sets the timer to produce an interrupt every N
    // microseconds, where N is the new tempo (value) divided by
    // the number of divisions per beat specified in the MIDI file header.
    TIM2->ARR = value/hdr->divisions - 1;
}

const float pitch_array[] = {
0.943874, 0.945580, 0.947288, 0.948999, 0.950714, 0.952432, 0.954152, 0.955876,
0.957603, 0.959333, 0.961067, 0.962803, 0.964542, 0.966285, 0.968031, 0.969780,
0.971532, 0.973287, 0.975046, 0.976807, 0.978572, 0.980340, 0.982111, 0.983886,
0.985663, 0.987444, 0.989228, 0.991015, 0.992806, 0.994599, 0.996396, 0.998197,
1.000000, 1.001807, 1.003617, 1.005430, 1.007246, 1.009066, 1.010889, 1.012716,
1.014545, 1.016378, 1.018215, 1.020054, 1.021897, 1.023743, 1.025593, 1.027446,
1.029302, 1.031162, 1.033025, 1.034891, 1.036761, 1.038634, 1.040511, 1.042390,
1.044274, 1.046160, 1.048051, 1.049944, 1.051841, 1.053741, 1.055645, 1.057552,
};

void pitch_wheel_change(int time, int chan, int value)
{
    //float multiplier = pow(STEP1, (value - 8192.0) / 8192.0);
    float multiplier = pitch_array[value >> 8];
    for(int n=0; n<sizeof voice / sizeof voice[0]; n++) {
        if (voice[n].in_use && voice[n].chan == chan) {
            voice[n].step = step[voice[n].note] * multiplier;
        }
    }
}


#if 0
int time = 0;
int n = 0;
void TIM2_IRQHandler(void)
{
    // TODO: Remember to acknowledge the interrupt here!
    TIM2->SR &= ~TIM_SR_UIF;

    // Look at the next item in the event array and check if it is
    // time to play that note.
    while(events[n].when == time) {
        // If the volume is 0, that means turn the note off.
        note_on(0,0,events[n].note, events[n].volume);
        n++;
    }

//    for(int x=0; x < 10000; x++)
//        ;


    // Increment the time by one tick.
    time += 1;

    // When we reach the end of the event array, start over.
    if ( n >= sizeof events / sizeof events[0]) {
        n = 0;
        time = 0;
    }
}
#endif

void TIM2_IRQHandler(void)
{
    // TODO: remember to acknowledge the interrupt here!
    TIM2->SR &= ~TIM_SR_UIF;

    midi_play();
}

void init_tim2(int n) {
    // TODO: you fill this in.
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->PSC = (uint32_t)48 - 1;
    TIM2->ARR = (uint32_t)n - 1;
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC->ISER[0] |= 1<<TIM2_IRQn;
    NVIC_SetPriority(TIM2_IRQn, 1);
}

#define STEP21
#if defined(STEP21)

int __io_putchar(int c) {

    //if the character passed as the argument c is a '\n' first write a '\r' to the USART5->TDR
    if(c == '\n') {
        while(!(USART5->ISR & USART_ISR_TXE)) { }
        USART5->TDR = '\r';
    }

    while(!(USART5->ISR & USART_ISR_TXE)) { }
    USART5->TDR = c;

    return c;
}

int __io_getchar(void) {
    while (!(USART5->ISR & USART_ISR_RXNE)) { }
    char c = USART5->RDR;

     //if the character read, c, is a carriage return ('\r'), change it to a linefeed ('\n')
     if(c == '\r') {
         c = '\n';
     }
     //echo the character read to the output by calling _io_putchar(c) just before it is returned.
     __io_putchar(c);

     return c;
}

void basic_drawing(void);
void move_ball(void);
void help_content(void);

int status = 0; // 0: power x, measure y
                // 1: power y, measure x

int x = 0;
int y = 0;

// ADC_IN1: PA1 -- x+
//          PA2 -- x-
// ADC_IN3: PA3 -- y+
//          PA4 -- y-
// --- touch screen part ---

void init_tim7(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
    TIM7->DIER |= TIM_DIER_UIE;
    TIM7->PSC = (uint32_t) 480-1;
    TIM7->ARR = (uint32_t) 1000-1; // set to 100 Hz
    TIM7->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1<<TIM7_IRQn;
    NVIC_SetPriority(TIM7_IRQn,2);
}

void init_touch(void){ // master initialization for touch screen ports and peripherals
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    init_tim7();
}

void setup_pa1_adc(void){
    GPIOA->MODER |= GPIO_MODER_MODER1; // analog mode
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 1;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}

void setup_pa3_adc(void){
    GPIOA->MODER |= GPIO_MODER_MODER3; // analog mode
    ADC1->CHSELR = 0;
    ADC1->CHSELR |= 1 << 3;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
}

void setup_pa1_3v(void){
    GPIOA->MODER &= ~GPIO_MODER_MODER1; // output mode
    GPIOA->MODER |= GPIO_MODER_MODER1_0;
    GPIOA->BSRR = 1 << 1;
}

void setup_pa3_3v(void){
    GPIOA->MODER &= ~GPIO_MODER_MODER3; // output mode
    GPIOA->MODER |= GPIO_MODER_MODER3_0;
    GPIOA->BSRR = 1 << 3;
}

void setup_pa2_gnd(void){
    GPIOA->MODER &= ~GPIO_MODER_MODER2; // output mode
    GPIOA->MODER |= GPIO_MODER_MODER2_0;
    GPIOA->BRR = 1 << 2;
}

void setup_pa4_gnd(void){
    GPIOA->MODER &= ~GPIO_MODER_MODER4; // output mode
    GPIOA->MODER |= GPIO_MODER_MODER4_0;
    GPIOA->BRR = 1 << 4;
}

void setup_pa2_open(void){
    GPIOA->MODER &= ~GPIO_MODER_MODER2; // input mode
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR2; // no pull-up and pull-down
}

void setup_pa4_open(void){
    GPIOA->MODER &= ~GPIO_MODER_MODER4; // input mode
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4; // no pull-up and pull-down
}



// ADC_IN1: PA1 -- x+
//          PA2 -- x-
// ADC_IN3: PA3 -- y+
//          PA4 -- y-

void TIM7_IRQHandler(void){

    TIM7->SR &= ~TIM_SR_UIF;
     // power x, measure y
    setup_pa1_3v();
    setup_pa2_gnd();
    setup_pa4_open();
    setup_pa3_adc();
    ADC1->CR |= ADC_CR_ADSTART ;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    // do something with ADC1->DR
    x = ADC1->DR;

    setup_pa3_3v();
    setup_pa4_gnd();
    setup_pa2_open();
    setup_pa1_adc();
    ADC1->CR |= ADC_CR_ADSTART;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    y = ADC1->DR;

    status = 0;


    if(x >2000 && x <2100 && y>1850 && y<2150){
        //enter game
        TIM2->CR1 &= ~TIM_CR1_CEN;//disable TIM2
        clear_voice_array();
        midi_init(midifile);
        TIM2->CR1 |= TIM_CR1_CEN;
        move_ball();
        printf("clicked start\n");
    }else if(x>1600 && x<1800 && y>1850 && y<2150){
        //enter help

        help_content();
        printf("in the help part\n");
    }else if(x>1330 && x<1450 && y>1850 && y<2150){
        //back to main menu

        basic_drawing();
        printf("click back\n");
    }


}

int main(void)
{

	init_touch();
    init_usart5();      //for debugging
    //enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout, 0);
    setbuf(stderr, 0);
    printf("RESTART\n");
    printf("-----------------------------------------------------------------\n");
    printf("-----------------------------------------------------------------\n");
    printf("-----------------------------------------------------------------\n");
    setup_buttons();
    LCD_Setup(); // this will call init_lcd_spi()
    basic_drawing();

    //MUSIC PART
    init_wavetable_hybrid2();
    init_dac();
    init_tim6();
    MIDI_Player *mp = midi_init(midifile);
    init_tim2(10417);

    for(;;) {
        asm("wfi");
        // If we hit the end of the MIDI file, start over.
        if (mp->nexttick == MAXTICKS)
            mp = midi_init(midifile);
    }
    return 0;
}
#endif
