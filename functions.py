import control as ct
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks


def plot_root_locus(transfer_function: control.TransferFunction, title :str):
    """
    Plota o lugar das raízes para uma função de transferência específica.

    Parâmetros:
    transfer_function : control.TransferFunction
        A função de transferência em malha aberta para a qual o lugar das raízes será calculado.
    """
    # Plot do lugar das raízes
    plt.figure(figsize=(8, 6))
    ct.root_locus(transfer_function, kvect=np.linspace(0, 50, 1000))
    plt.title("Mapa do Lugar das Raízes")
    plt.xlabel("Parte Real")
    plt.ylabel("Parte Imaginária")
    plt.grid()
    plt.savefig(title)  # Salva como PNG
    plt.show()
    
def routh_hurwitz(coefficients):
    """
    Verifica a estabilidade de um sistema usando o critério de Routh-Hurwitz.
    
    Parâmetros:
    coefficients : list
        Lista com os coeficientes do polinômio característico.
    
    Retorna:
    tabela : numpy.ndarray
        A tabela de Routh gerada.
    is_stable : bool
        True se o sistema for estável, False caso contrário.
    """
    # Número de coeficientes
    n = len(coefficients)
    
    # Número de colunas na tabela
    cols = (n + 1) // 2

    # Criar tabela de Routh com zeros
    routh_table = np.zeros((n, cols))

    # Preencher as duas primeiras linhas com os coeficientes
    routh_table[0, :len(coefficients[0::2])] = coefficients[0::2]  # Coeficientes das potências pares
    routh_table[1, :len(coefficients[1::2])] = coefficients[1::2]  # Coeficientes das potências ímpares

    # Preencher as outras linhas da tabela de Routh
    for i in range(2, n):
        for j in range(cols - 1):
            numerator = (routh_table[i - 1, 0] * routh_table[i - 2, j + 1] - 
                         routh_table[i - 2, 0] * routh_table[i - 1, j + 1])
            denominator = routh_table[i - 1, 0]
            if denominator == 0:  # Evitar divisão por zero
                denominator = 1e-6  # Perturbação pequena
            routh_table[i, j] = numerator / denominator

        # Verificar se a linha inteira é zero
        if np.allclose(routh_table[i, :], 0):
            # Preencher a linha especial com derivada da linha anterior
            routh_table[i, :] = np.polyder(routh_table[i - 1, :])

    # Verificar se há troca de sinais na primeira coluna
    first_column = routh_table[:, 0]
    sign_changes = np.sum(np.diff(np.sign(first_column[first_column != 0])) != 0)

    is_stable = sign_changes == 0

    return routh_table, is_stable
def malha_fechada_routh(control_system:list, Kp:float):
    """
    Verifica a estabilidade do sistema em malha fechada com controle proporcional.
    
    Parâmetros:
    control_system : TransferFunction
        Função de transferência do sistema em malha aberta.
    Kp : float
        Ganho proporcional do controlador.
    
    Retorna:
    tabela_routh : numpy.ndarray
        Tabela de Routh.
    is_stable : bool
        True se o sistema for estável, False caso contrário.
    """
    # Polinômio característico em malha fechada
    den_open = np.array(control_system.den[0]).ravel()  # Denominador como vetor 1D
    num_open = np.array(control_system.num[0]).ravel()  # Numerador como vetor 1D
    poly_char = np.polyadd(den_open, np.polymul(Kp, num_open))  # P(s) = D(s) + Kp * N(s)
    
    # Verificar estabilidade com o critério de Routh-Hurwitz
    routh_table, is_stable = routh_hurwitz(poly_char)
    
    return routh_table, is_stable

def ziegler_nichols_second_method(open_loop: list, controller_type="PID"):
    """
    Calcula os parâmetros do controlador usando o segundo método de Ziegler-Nichols.
    
    Parâmetros:
    - open_loop: control.TransferFunction
        Função de transferência do sistema em malha aberta.
    - Ku: float
        Ganho crítico (K_u) onde o sistema entra em oscilação contínua.
    - Tu: float
        Período de oscilação contínua (T_u).
    - controller_type: str
        Tipo de controlador ("P", "PI", ou "PID").
    
    Retorna:
    - dict com os valores Kp, Ki, Kd
    """
    out_st_magn = ct.stability_margins(open_loop)
    Ku = out_st_magn[0]
    print(Ku)
    
    Wu = out_st_magn[3]
    Tu = (2*np.pi)/Wu
    
    if controller_type == "P":
        Kp = 0.5 * Ku
        return {"Kp": Kp, "Ki": 0, "Kd": 0}
    elif controller_type == "PI":
        Kp = 0.45 * Ku
        Ki = Kp / (0.83 * Tu)
        return {"Kp": Kp, "Ki": Ki, "Kd": 0}
    elif controller_type == "PID":
        Kp = 0.6 * Ku
        Ki = 2 * Kp / Tu
        Kd = 0.125 * Kp * Tu
        return {"Kp": Kp, "Ki": Ki, "Kd": Kd}
    else:
        raise ValueError("Tipo de controlador inválido. Escolha 'P', 'PI' ou 'PID'.")


def simulate_closed_loop(open_loop, Kp, Ki=0, Kd=0):
    """
    Simula o sistema em malha fechada com controlador proporcional, PI ou PID.
    
    Parâmetros:
    - open_loop: control.TransferFunction
        Função de transferência do sistema em malha aberta.
    - Kp, Ki, Kd: float
        Ganhos do controlador.
    
    Retorna:
    - t: array
        Tempo da simulação.
    - y: array
        Resposta do sistema.
    """
    # Controlador PID
    controller = ct.TransferFunction([Kd, Kp, Ki], [1, 0])  # PID: Kd * s^2 + Kp * s + Ki
    closed_loop = ct.feedback(controller * open_loop)
    Tu = Kp / (0.83 * Ki)
    # Simulação da resposta ao degrau
    t = np.linspace(0, 5 * Tu, 1000)  # Tempo de simulação baseado em Tu
    t, y = ct.step_response(closed_loop, t)
    
    return t, y

def find_critical_gain_and_period_1(open_loop, Kp_range, t_sim):
    """
    Determina o ganho crítico (Ku) e o período crítico (Tu) da malha fechada.
    
    Parâmetros:
    - open_loop: control.TransferFunction
        Função de transferência do sistema em malha aberta.
    - Kp_range: array-like
        Intervalo de valores de Kp a testar.
    - t_sim: float
        Tempo de simulação para cada Kp.
    
    Retorna:
    - Ku: float
        Ganho crítico onde ocorrem oscilações contínuas.
    - Tu: float
        Período das oscilações contínuas.
    """
    for Kp in Kp_range:
        # Malha fechada com controlador proporcional puro
        controller = ct.TransferFunction([Kp], [1])
        closed_loop = ct.feedback(controller * open_loop)

        # Simulação da resposta ao degrau
        t = np.linspace(0, t_sim, 1000)
        t, y = ct.step_response(closed_loop, t)

        # Verifica oscilações contínuas
        if np.max(y) > 1.5 or np.any(np.isnan(y)):  # Condição para instabilidade
            continue

        # Analisa o comportamento oscilatório
        peaks, _ = find_peaks(y, height=1.0)  # Encontra picos da resposta
        if len(peaks) > 1:  # Mais de um pico indica oscilação
            Tu = np.mean(np.diff(t[peaks]))  # Diferença entre picos consecutivos
            return Kp, Tu

    raise ValueError("Oscilações contínuas não encontradas no intervalo de Kp.")

# def find_peaks(y, height):
#     """
#     Simples função para detectar picos em um sinal.
#     """
#     peaks = np.where((y[:-1] < y[1:]) & (y[1:] > y[2:]) & (y[1:] > height))[0] + 1
#     return peaks, y[peaks]

# # Sistema em malha aberta
# servo = ct.TransferFunction([-1], [1, 10])  # Servo do elevador
# dynamics = ct.TransferFunction([-2.0, -0.6], [1, 0.65, 2.15])  # Dinâmica de curto período
# open_loop = servo * dynamics

# # Intervalo de ganhos e tempo de simulação
# Kp_range = np.linspace(1, 100, 1000)  # Testar valores de Kp de 1 a 100
# t_sim = 20  # Tempo de simulação (s)

# # Determina Ku e Tu
# try:
#     Ku, Tu = find_critical_gain_and_period(open_loop, Kp_range, t_sim)
#     print(f"Ganho crítico (Ku): {Ku}")
#     print(f"Período crítico (Tu): {Tu}")
# except ValueError as e:
#     print(str(e))
    
def plot_response(system, time, title, label):
    _, response = ct.step_response(system, time)
    plt.plot(time, response, label=label)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid()


def plot_bode(system, title):
    mag, phase, omega = ct.bode(system, dB=True, plot=False)
    plt.semilogx(omega, 20 * np.log10(mag), label=f"{title}")
    plt.xlabel("Frequency (rad/s)")
    plt.ylabel("Magnitude (dB)")
    plt.grid()
