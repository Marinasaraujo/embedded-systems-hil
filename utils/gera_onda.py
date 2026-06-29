import math
import datetime

def gerar_vetor_dac_parametrizado(frequencia_onda, amostras_por_ciclo, dac_bits, amplitude_normalizada):
    """
    Gera um vetor de valores para um DAC, representando uma forma de onda senoidal
    com base nos parâmetros fornecidos, incluindo amplitude normalizada.

    Args:
        frequencia_onda (float): Frequência da forma de onda em Hertz (Hz).
        amostras_por_ciclo (int): Número de amostras para um ciclo completo da onda.
        dac_bits (int): Resolução em bits do DAC (ex: 12 para 0-4095).
        amplitude_normalizada (float): Amplitude da senoide de 0 a 1, onde 1 é a amplitude máxima do DAC.

    Returns:
        tuple: Uma tupla contendo:
            - list: Uma lista de inteiros com os valores do DAC.
            - float: A frequência de amostragem calculada (Hz).
    """
    if amostras_por_ciclo <= 0:
        raise ValueError("O número de amostras por ciclo deve ser maior que zero.")
    if dac_bits <= 0:
        raise ValueError("A resolução do DAC em bits deve ser maior que zero.")
    if frequencia_onda <= 0:
        raise ValueError("A frequência da onda deve ser maior que zero.")
    if not (0 <= amplitude_normalizada <= 1):
        raise ValueError("A amplitude normalizada deve estar entre 0 e 1.")

    max_dac_val = (2**dac_bits) - 1
    offset = max_dac_val / 2.0
    amplitude_dac = amplitude_normalizada * (max_dac_val / 2.0)

    dac_valores = []
    for i in range(amostras_por_ciclo):
        valor = offset + amplitude_dac * math.sin(2 * math.pi * i / amostras_por_ciclo)
        dac_valores.append(int(round(max(0, min(valor, max_dac_val)))))

    frequencia_amostragem = frequencia_onda * amostras_por_ciclo

    return dac_valores, frequencia_amostragem

def salvar_vetor_em_arquivo_c(filename, vetor_dac, freq_onda, num_amostras, res_dac, amp_norm, freq_amostragem, prd_timer_val):
    """
    Salva o vetor DAC em um arquivo .c.
    """
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    with open(filename, "w") as f_c:
        f_c.write(f"/*\n")
        f_c.write(f" * Arquivo gerado por script Python em {current_time}\n")
        f_c.write(f" * Frequência da Onda: {freq_onda} Hz\n")
        f_c.write(f" * Amostras por Ciclo: {num_amostras}\n")
        f_c.write(f" * Resolução do DAC: {res_dac} bits (Valores de 0 a {(2**res_dac)-1})\n")
        f_c.write(f" * Amplitude da Onda (Normalizada 0-1): {amp_norm}\n")
        f_c.write(f" * Frequência de Amostragem Necessária (para Timer): {freq_amostragem:.2f} Hz\n")
        f_c.write(f" * Valor PRD Recomendado para o Timer: {int(round(prd_timer_val))}\n")
        f_c.write(f" */\n\n")
        f_c.write(f"#include <stdint.h>\n\n") # Inclui stdint.h para uint16_t
        f_c.write(f"// Vetor contendo os valores para o DAC\n")
        f_c.write(f"const uint16_t dac_buffer[{len(vetor_dac)}] = {{\n")
        for i, val in enumerate(vetor_dac):
            f_c.write(f"    {val},")
            if (i + 1) % 10 == 0:
                f_c.write("\n")
            else:
                f_c.write(" ")
        f_c.write(f"\n}};\n")

    print(f"\nResultados salvos em '{filename}'")


if __name__ == "__main__":
    print("--- Gerador de Vetor DAC para Senoide ---")

    try:
        # Pede os parâmetros ao usuário
        freq_onda = float(input("Digite a frequência da onda em Hz (ex: 50): "))
        num_amostras = int(input("Digite o número de amostras por ciclo (ex: 200): "))
        res_dac = int(input("Digite a resolução do DAC em bits (ex: 12): "))
        amplitude_norm = float(input("Digite a amplitude da onda (de 0.0 a 1.0, onde 1.0 é máxima): "))

        # Gera o vetor e a frequência de amostragem
        vetor_dac, freq_amostragem = gerar_vetor_dac_parametrizado(freq_onda, num_amostras, res_dac, amplitude_norm)

        # Assume um clock de timer padrão de 200 MHz para o TMS320F28379D
        # Adapte este valor se seu clock for diferente!
        clock_timer = 200_000_000 # 200 MHz
        prd_timer = (clock_timer / freq_amostragem) - 1

        print(f"\n--- Resultados Calculados (para sua referencia) ---")
        print(f"Frequencia da Onda Desejada: {freq_onda} Hz")
        print(f"Número de Amostras por Ciclo: {num_amostras}")
        print(f"Resolucao do DAC: {res_dac} bits (Valores de 0 a {(2**res_dac)-1})")
        print(f"Amplitude da Onda (Normalizada 0-1): {amplitude_norm}")
        print(f"Frequencia de Amostragem Necessaria (para Timer): {freq_amostragem:.2f} Hz")
        print(f"Valor PRD Recomendado para o Timer (com clock de {clock_timer/1e6:.0f} MHz): {int(round(prd_timer))}")
        print(f"Valor Minimo Gerado: {min(vetor_dac)}")
        print(f"Valor Maximo Gerado: {max(vetor_dac)}")

        # Salvar o vetor em arquivo .c
        salvar_vetor_em_arquivo_c("dac_buffer_values.c", vetor_dac, freq_onda, num_amostras, res_dac, amplitude_norm, freq_amostragem, prd_timer)

    except ValueError as e:
        print(f"\nErro de entrada: {e}. Por favor, digite valores numéricos válidos e verifique o range da amplitude.")
    except Exception as e:
        print(f"\nOcorreu um erro inesperado: {e}")