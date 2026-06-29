//
// Arquivos de Inclusao
//
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"

//
// Definiçoes de Constantes
//
#define F_PWM                  20000.0f     // Frequencia de chaveamento (Hz)
#define T_PWM                  (1.0f / F_PWM) // Periodo de chaveamento (s)
#define DT_SIM                 0.000001f    // Passo de simulacao (1 us)

// Nesse exemplo temos passo fixo
#define N_STEPS_PER_CYCLE      (uint32_t)(T_PWM / DT_SIM) // Passos por ciclo PWM

// Parametros do Inversor monofasico
#define VDC                    400.0f       // Tensao de entrada (V)
#define L                      0.003f      // Indutancia (H)
#define R                      0.0377f       // Carga resistiva (Ohm)

// Constantes auxiliares (evita divisoes repetidas no loop)
#define TUSTIN_A ((2.0f * L / DT_SIM) + R)
#define TUSTIN_B ((2.0f * L / DT_SIM) - R)

float32_t c1 = TUSTIN_B / TUSTIN_A;   // coef do estado
float32_t c2 = 1.0f / TUSTIN_A;       // coef das tensões

//
// Variaveis Globais da Simulaçao
//
volatile float32_t g_vg_sim = 0.0f;        // Tensao de saida simulada
volatile float32_t g_ig_sim = 0.0f;          // Corrente no indutor simulada
volatile float32_t g_vind_z1 = 0.0f;          // Memoria vind
volatile uint32_t g_step_counter = 0;        // Contador de passos dentro do ciclo PWM

// Por enquanto tá true ou false mas esses g_switches virão de pinos de gpio reais conectados ao  pwm
// Teremos então 4 gpios, um para cada chave 
volatile bool g1_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g2_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g3_switch_on = false;           // Estado da chave (true = ligada)
volatile bool g4_switch_on = false;           // Estado da chave (true = ligada)

volatile bool g_new_step_ready = false;      // Flag para novo passo de simulaçao

volatile float g_duty_cycle = 0.5f;          // Razao ciclica (virá da CLA)

//Buffer de visualizaçao dos resultados
#define TAMBUFFER 100
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
    // Aqui temos a emulação de uma onda pwm
    // No trabalho o pwm já é gerado pelo periferico epwm
    // Define estado da chave com base na razao ciclica
    bool on = (g_step_counter < (uint32_t)(g_duty_cycle * N_STEPS_PER_CYCLE));
    // ponte: braço A e braço B complementares (unipolar simplificado p/ teste)
    g1_switch_on =  on;   g4_switch_on =  on;
    g2_switch_on = !on;   g3_switch_on = !on;
    
    // Atualiza contador
    g_step_counter++;

    // Reinicia no fim do ciclo PWM
    if (g_step_counter >= N_STEPS_PER_CYCLE)
        g_step_counter = 0;

    // lógica da ponte inversora
    if(g1_switch_on && g4_switch_on){
        vf =  VDC;
    } else if (g2_switch_on && g3_switch_on) {
        vf = -VDC;
    } else {
        vf = 0.0f; //Roda livre
    }
    // Tensao no indutor
    vind_new = vf - g_vg_sim;

    float32_t g_ig_sim_new = c1 * g_ig_sim + c2 * vind_new + c2 * g_vind_z1;
    
    // Atualiza a memória
    g_vind_z1 = vind_new;
    g_ig_sim = g_ig_sim_new;
    
    //Gravaçao dos dados para visualizar os resultados
    buffer_vg[cnt_buff] = g_vg_sim;
    buffer_ig[cnt_buff] = g_ig_sim_new;
    cnt_buff=(cnt_buff+1)%(TAMBUFFER); // coloca outro break point aqui

    // Libera nova interrupçao
    Interrupt_clearACKGroup(INT_myCPUTIMER0_INTERRUPT_ACK_GROUP);
}
