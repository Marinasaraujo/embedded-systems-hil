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

volatile bool g1_switch_on;
volatile bool g2_switch_on;
volatile bool g3_switch_on;
volatile bool g4_switch_on;


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

__interrupt void xint1ISR(void)
{
    g1_switch_on = GPIO_readPin(myGPIO0);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void xint2ISR(void)
{
    g2_switch_on = GPIO_readPin(myGPIO1);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt void xint3ISR(void)
{
    g3_switch_on = GPIO_readPin(myGPIO2);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1); //group12
}

__interrupt void xint4ISR(void)
{
    g4_switch_on = GPIO_readPin(myGPIO3);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);  //group12
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
        
    float32_t s = sinf(g_theta);
    vg     = VPK * s;
    ig_ref = IPK * s;
    

    // Atualiza contador
    g_step_counter++;

    // Reinicia no fim do ciclo PWM
    if (g_step_counter >= N_STEPS_PER_CYCLE){
        g_step_counter = 0;
        CLA_forceTasks(myCLA0_BASE, CLA_TASKFLAG_1);
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

    // float u = 0.5f*(ig / I_FS) + 0.5f;
    // if (u < 0.f) u = 0.f; else if (u > 1.f) u = 1.f;
    // DAC_setShadowValue(DAC0_BASE, (uint16_t)(u*4095.f + 0.5f));
    
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


// Sai pelo DAC da planta e entra pelo ADC na CLA

// // 
// // Rotina de Interrupção do ADC (Disparada pelo fim da conversão)
// //
__interrupt void INT_ADC0_1_ISR(void)  
{

}

// // 
// // Rotina de Interrupção do DAC (Disparada pelo Timer 1)
// //
// __interrupt void INT_myCPUTIMER1_ISR(void)
// {
//     static uint16_t cnt_dac = 0;
    
//     DAC_setShadowValue(DAC0_BASE, (uint16_t) (gain * dac_buffer[cnt_dac]));
//     cnt_dac = (cnt_dac + 1) % TAM_BUFFER_DAC; 
// }