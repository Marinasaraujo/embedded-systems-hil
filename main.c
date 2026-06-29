//
// Arquivos de Inclusao
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"

#pragma DATA_SECTION(vg,"CpuToCla1MsgRAM"); // aloca a variavel em espaços especificos da memória, definidos no .cmd
#pragma DATA_SECTION(ig,"CpuToCla1MsgRAM"); 
#pragma DATA_SECTION(ig_ref,"CpuToCla1MsgRAM"); 
float vg;
float ig;
float ig_ref;

#pragma DATA_SECTION(d,"Cla1ToCpuMsgRAM");
#pragma DATA_SECTION(hb,"Cla1ToCpuMsgRAM");
float d;
float hb = 1;

//
// Definiçoes de Constantes
//
#define F_PWM                  20000.0f     // Frequencia de chaveamento (Hz)
#define T_PWM                  (1.0f / F_PWM) // Periodo de chaveamento (50 us)
#define DT_SIM                 0.000001f    // Passo de simulacao (1 us)

// Nesse exemplo temos passo fixo
#define N_STEPS_PER_CYCLE      (uint32_t)(T_PWM / DT_SIM) // Passos por ciclo PWM = 50

// Parametros do Inversor monofasico
#define VDC                    400.0f       // Tensao de entrada (V)
#define L                      0.003f      // Indutancia (H)
#define R                      0.0377f       // Carga resistiva (Ohm)

// Parametros da rede / referencia
#define TWO_PI                 6.2831853072f
#define F_GRID                 60.0f        // Frequencia da rede (Hz)
#define PN                     2000
#define SQRT2                  1.4142135f
#define VPK                    (220.0f * SQRT2)       
#define IPK                    (2.0f * PN / VPK)    // Pico da corrente de referencia (A)

// Constantes auxiliares (evita divisoes repetidas no loop)
#define TUSTIN_A ((2.0f * L / DT_SIM) + R)
#define TUSTIN_B ((2.0f * L / DT_SIM) - R)

float32_t c1 = TUSTIN_B / TUSTIN_A;   // coef do estado
float32_t c2 = 1.0f / TUSTIN_A;       // coef das tensões

//
// Variaveis Globais da Simulaçao
//
volatile float32_t g_vind_z1 = 0.0f;          // Memoria vind
volatile uint32_t g_step_counter = 0;        // Contador de passos dentro do ciclo PWM
volatile float32_t g_theta = 0.0f;           // Fase unica (rede e referencia)

// Por enquanto tá true ou false mas esses g_switches virão de pinos de gpio reais conectados ao  pwm
// Teremos então 4 gpios, um para cada chave 
volatile bool g1_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g2_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g3_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g4_switch_on = false;           // Estado da chave (true = ligada)
volatile bool wrapped      = false;

//Buffer de visualizaçao dos resultados
#define TAMBUFFER 200
volatile float buffer_vg[TAMBUFFER];
volatile float buffer_ig[TAMBUFFER];
volatile char cnt_buff = 0;
//
// Funçao Principal
//
void main(void)
{

    // Inicializaçoes do dispositivo
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    Board_init();

    CLA_forceTasks(myCLA0_BASE, CLA_TASKFLAG_8); // Roda só uma vez como init das variaveis
    // Habilita interrupçoes globais
    EINT;
    ERTM;

    // Loop principal
    while (1)
    {
        
    }
}

//
// Interrupçao do Timer (gera novo passo de simulaçao HIL)
//
__interrupt void INT_myCPUTIMER0_ISR(void)
{
    float32_t vf, vind_new;

    g_theta += TWO_PI * F_GRID * DT_SIM;
    
    if (g_theta >= TWO_PI){
        g_theta -= TWO_PI;
    }
        
    vg     = VPK * sinf(g_theta);
    ig_ref = IPK * sinf(g_theta);
    
    // Aqui temos a emulação de uma onda pwm
    // No trabalho o pwm já é gerado pelo periferico epwm
    // Define estado da chave com base na razao ciclica
    float DUTY_CICLE = 0.5f * (d + 1.0f); // [-1, 1] para [0,1]
    bool on = (g_step_counter < (uint32_t)(DUTY_CICLE * N_STEPS_PER_CYCLE));
    
    g1_switch_on =  on;   g4_switch_on =  on;     
    g2_switch_on = !on;   g3_switch_on = !on;

    
    // Atualiza contador
    g_step_counter++;

    // Reinicia no fim do ciclo PWM
    if (g_step_counter >= N_STEPS_PER_CYCLE){
        g_step_counter = 0;
        wrapped = true;
    }
        

    // lógica da ponte inversora
    if(g1_switch_on && g4_switch_on){
        vf =  VDC;
    } else if (g2_switch_on && g3_switch_on) {
        vf = -VDC;
    } else {
        vf = 0.0f; //Roda livre
    }
    // Tensao no indutor
    vind_new = vf - vg;

    float32_t ig_new = c1 * ig + c2 * vind_new + c2 * g_vind_z1;
    
    // Atualiza a memória
    g_vind_z1 = vind_new;
    ig = ig_new;
    
    if (wrapped)
        CLA_forceTasks(myCLA0_BASE, CLA_TASKFLAG_1);
    
    //Gravaçao dos dados para visualizar os resultados
    buffer_vg[cnt_buff] = vg;
    buffer_ig[cnt_buff] = ig_new;
    cnt_buff=(cnt_buff+1)%(TAMBUFFER); // coloca outro break point aqui

    // Libera nova interrupçao
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}

__interrupt void cla1Isr1 () // chamada pela cla quando ela finaliza
{
    // CLA roda assincrona e o envio só acontece quando d foi calculado
    Interrupt_clearACKGroup(INT_myCLA01_INTERRUPT_ACK_GROUP);
}



// // 
// // Rotina de Interrupção do ADC (Disparada pelo fim da conversão)
// //
// __interrupt void INT_ADC0_1_ISR(void)  
// {
//     static uint16_t cnt_adc = 0; 
//     cnt_adc = (cnt_adc + 1) % TAM_BUFFER_ADC;
//     adc_buffer[cnt_adc] = ADC_readResult(ADC0_RESULT_BASE, ADC0_SOC0);
//     ADC_clearInterruptStatus(ADC0_BASE, ADC_INT_NUMBER1);
//     Interrupt_clearACKGroup(INT_ADC0_1_INTERRUPT_ACK_GROUP);
// }

// // 
// // Rotina de Interrupção do DAC (Disparada pelo Timer 1)
// //
// __interrupt void INT_myCPUTIMER1_ISR(void)
// {
//     static uint16_t cnt_dac = 0;
    
//     DAC_setShadowValue(DAC0_BASE, (uint16_t) (gain * dac_buffer[cnt_dac]));
//     cnt_dac = (cnt_dac + 1) % TAM_BUFFER_DAC; 
// }