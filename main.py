from functions import *
import control as ct
from scipy.signal import lti, step


if __name__ == "__main__":
    
    # # Parâmetros do sistema
    # k_a = 1  # Valor inicial do ganho (pode ser ajustado)
    # servo = ct.TransferFunction([-k_a], [1, 10])  # Servo do elevador
    # dynamics = ct.TransferFunction([-2.0, -0.6], [1, 0.65, 2.15])  # Dinâmica de curto período

    # tranfer_functions = [servo, dynamics]
    # plot_root_locus(tranfer_functions)
    
    # # Exemplo de uso
    # coefficients = [2.15, 0.65, 1]  # Coeficientes do polinômio 2.15s^2 + 0.65s + 1
    # routh_table, is_stable = routh_hurwitz(coefficients)

    # print("Tabela de Routh:")
    # print(routh_table)
    # if is_stable:
    #     print("O sistema é estável!")
    # else:
    #     print("O sistema é instável!")
    
#    # Exemplo de uso
#     # Definir o sistema em malha aberta
#     servo = ct.TransferFunction([-1], [1, 10])  # Servo do elevador
#     dynamics = ct.TransferFunction([-2.0, -0.6], [1, 0.65, 2.15])  # Dinâmica de curto período
#     open_loop = servo * dynamics  # Função de transferência em malha aberta

#     # Verificar estabilidade com ganho proporcional
#     Kp = 1  # Escolha do ganho proporcionalz
#     routh_table, is_stable = malha_fechada_routh(open_loop, Kp)

#     print("Tabela de Routh para Kp =", Kp)
#     print(routh_table)
#     if is_stable:
#         print("O sistema em malha fechada é estável para Kp =", Kp)
#     else:
#         print("O sistema em malha fechada é instável para Kp =", Kp)

    # # Exemplo de uso
    # # Definição do sistema em malha aberta
    # servo = ct.TransferFunction([-1], [1, 10])  # Servo do elevador
    # dynamics = ct.TransferFunction([-2.0, -0.6], [1, 0.65, 2.15])  # Dinâmica de curto período
    # open_loop = servo * dynamics  # Função de transferência em malha aberta

    # Valores de K_u e T_u obtidos experimentalmente
    # Ku = 30  # Ganho crítico (exemplo)
    # # Tu = 2.5  # Período de oscilação contínua (exemplo)

    # # Escolha do tipo de controlador
    # controller_type = "PID"  # Pode ser "P", "PI" ou "PID"

    # # Cálculo dos parâmetros do controlador
    # controller_params = ziegler_nichols_second_method(open_loop, Ku, Tu, controller_type)
    # print(f"Parâmetros do controlador {controller_type}: {controller_params}")

    # # Simulação do sistema em malha fechada
    # t, y = simulate_closed_loop(open_loop, **controller_params)

    # # Plot da resposta ao degrau
    # plt.figure()
    # plt.plot(t, y, label=f"Malha Fechada ({controller_type})")
    # plt.title("Resposta ao Degrau com Controlador em Malha Fechada")
    # plt.xlabel("Tempo (s)")
    # plt.ylabel("Saída")
    # plt.grid()
    # plt.legend()
    # plt.show()
    
    #     # Plot da malha fechada para Ku
    # controller = ct.TransferFunction([Ku], [1])
    # closed_loop = ct.feedback(controller * open_loop)

    # t = np.linspace(0, t_sim, 1000)
    # t, y = ct.step_response(closed_loop, t)

    # plt.figure()
    # plt.plot(t, y, label=f"Resposta com Ku={Ku}")
    # plt.title("Oscilações Contínuas em Malha Fechada")
    # plt.xlabel("Tempo (s)")
    # plt.ylabel("Saída")
    # plt.grid()
    # plt.legend()
    # plt.show()
    # Sistema em malha aberta
    # servo = ct.TransferFunction([-1], [1, 10])  # Servo do elevador
    # dynamics = ct.TransferFunction([-2.0, -0.6], [1, 0.65, 2.15])  # Dinâmica de curto período
    # open_loop = servo * dynamics


    #     plt.figure()
    #     plt.plot(t, y, label=f"Resposta com Ku={Ku}")
    #     plt.title("Oscilações Contínuas em Malha Fechada")
    #     plt.xlabel("Tempo (s)")
    #     plt.ylabel("Saída")
    #     plt.grid()
    #     plt.legend()
    #     plt.show()
    


    # Definir a função de transferência do sistema
    # numerator = [3]
    # denominator = [1, 12, 25, 50]
    # system = lti(numerator, denominator)
    # Intervalo de ganhos e tempo de simulação
    # servo = ct.TransferFunction([-1], [1, 10])  # Servo do elevador
    # dynamics = ct.TransferFunction([-2.0, -0.6], [1, 0.65, 2.15])  # Dinâmica de curto período
    # open_loop = servo * dynamics  # Função de transferência em malha aberta
    # Kp_range = np.linspace(1, 1000, 10000)  # Ganhos de 1 a 1000
  
    # t_sim = 20  # Tempo de simulação (s)

    # # Determina Ku e Tu
    # try:
    #     Ku, Tu = find_critical_gain_and_period(open_loop, Kp_range, t_sim)
    #     print(f"Ganho crítico (Ku): {Ku}")
    #     print(f"Período crítico (Tu): {Tu}")
    # except ValueError as e:
    #     print(str(e))
    #     Ku, Tu = None, None  # Garantia para não usar valores indefinidos

    # # Plot da malha fechada para Ku, se encontrado
    # if Ku is not None:
    #     controller = ct.TransferFunction([Ku], [1])
    #     closed_loop = ct.feedback(controller * open_loop)

    #     t = np.linspace(0, t_sim, 1000)
    #     t, y = ct.step_response(closed_loop, t)

    # # Definir os parâmetros dos controladores
    # Kp_P = 0.5*Ku
    # Kp_PI = 0.45*Ku
    # Ki_PI = Tu / 1.2
    # Kp_PID = 0.68*Ku
    # Ki_PID = Tu / 2
    # Kd_PID = Tu * 0.125

    # # Criar funções de transferência com os controladores
    # system_P = lti([Kp_P * n for n in numerator], denominator)
    # system_PI = lti([Kp_PI * n for n in numerator], np.convolve(denominator, [1, Ki_PI]))
    # system_PID = lti([Kd_PID, Kp_PID, Ki_PID], denominator)

    # # Tempo para simulação
    # time = np.linspace(0, 20, 1000)

    # # Respostas no tempo
    # _, response_P = step(system_P, T=time)
    # _, response_PI = step(system_PI, T=time)
    # _, response_PID = step(system_PID, T=time)

    # # Plotar os resultados
    # plt.figure(figsize=(12, 8))

    # # Gráfico 1: Resposta com controladores P, PI, e PID
    # plt.subplot(2, 1, 1)
    # plt.plot(time, response_P, label="P Control")
    # plt.plot(time, response_PI, label="PI Control")
    # plt.plot(time, response_PID, label="PID Control")
    # plt.title("Response of the System with P, PI, and PID Controllers")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Amplitude")
    # plt.legend()
    # plt.grid()

    # # Gráfico 2: Resposta detalhada do controlador PID
    # plt.subplot(2, 1, 2)
    # plt.plot(time, response_PID, label="PID Control", color="red")
    # plt.title("Response of the System with PID Controller")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Amplitude")
    # plt.grid()

    # plt.tight_layout()
    # plt.show()



    # Intervalo de ganhos e tempo de simulação
    servo = ct.TransferFunction([-1], [1, 10])  # Servo do elevador
    dynamics = ct.TransferFunction([-2.0, -0.6], [1, 0.65, 2.15, 0])  # Dinâmica de curto período
    open_loop = servo * dynamics  # Função de transferência em malha aberta
    Kp_range = np.linspace(1, 1000, 2000)  # Ganhos de 1 a 1000

    t_sim = 20  # Tempo de simulação (s)

    # Determina Ku e Tu
    # def find_critical_gain_and_period(open_loop, Kp_range, t_sim):
    #     for Kp in Kp_range:
    #         # Define o controlador proporcional
    #         controller = ct.TransferFunction([Kp], [1])
    #         closed_loop = ct.feedback(controller * open_loop)
            
    #         # Simula a resposta ao degrau
    #         t = np.linspace(0, t_sim, 1000)
    #         _, y = ct.step_response(closed_loop, t)
            
    #         # Encontra os picos na resposta
    #         peaks, _ = find_peaks(y)
    #         if len(peaks) >= 2:  # Verifica se há pelo menos dois picos
    #             # Calcula o período entre os dois primeiros picos
    #             period = t[peaks[1]] - t[peaks[0]]
    #             return Kp, period   
    #     raise ValueError("Ganho crítico não encontrado no intervalo fornecido.")
    
    # def find_critical_gain_and_period(open_loop, Kp_range, t_sim):
    #     for Kp in Kp_range:
    #         # Define o controlador proporcional
    #         controller = ct.TransferFunction([Kp], [1])
    #         closed_loop = ct.feedback(controller * open_loop)

    #         # Simula a resposta ao degrau
    #         t = np.linspace(0, t_sim, 1000)
    #         _, y = ct.step_response(closed_loop, t)

    #         # Encontra os picos na resposta
    #         peaks, properties = find_peaks(y)

    #         if len(peaks) >= 2:
    #             amplitudes = y[peaks]  # Obtém as amplitudes dos picos

    #             # Verifica se os picos são aproximadamente constantes ao longo do eixo
    #             if np.std(amplitudes) < 0.05 * np.mean(amplitudes):  
    #                 period = t[peaks[1]] - t[peaks[0]]
    #                 return Kp, period   

    #     raise ValueError("Ganho crítico não encontrado no intervalo fornecido.")
 
    out_st_magn = ct.stability_margins(open_loop)
    Ku = out_st_magn[0]
    print(Ku)
    
    Wu = out_st_magn[3]
    Tu = (2*np.pi)/Wu
    
    print(Tu)
    
    # try:
    #     Ku, Tu = find_critical_gain_and_period(open_loop, Kp_range, t_sim)
    #     print(f"Ganho crítico (Ku): {Ku}")
    #     print(f"Período crítico (Tu): {Tu}")
    # except ValueError as e:
    #     print(str(e))
    #     Ku, Tu = None, None  # Garantia para não usar valores indefinidos
    
    # try:
    #     Ku, Tu = find_critical_gain_and_period_1(open_loop, Kp_range, t_sim)
    #     print(f"Ganho crítico (Ku): {Ku}")
    #     print(f"Período crítico (Tu): {Tu}")
    # except ValueError as e:
    #     print(str(e))
    #     Ku, Tu = None, None  # Garantia para não usar valores indefinidos
    
    
   
    # Plot da malha fechada para Ku, se encontrado
    if Ku is not None:
        controller = ct.TransferFunction([Ku], [1])
        closed_loop = ct.feedback(controller * open_loop)

        t = np.linspace(0, t_sim, 1000)
        t, y = ct.step_response(closed_loop, t)

        plt.figure()
        plt.plot(t, y)
        plt.title("Closed Loop Response with Critical Gain (Ku)")
        plt.xlabel("Time (s)")
        plt.ylabel("Amplitude")
        plt.grid()
        plt.savefig("response_ku.png")
        plt.show()

    # Definir os parâmetros dos controladores
    if Ku is not None:
        Kp_P = 0.5 * Ku
        print(f"o ganho proporcional{Kp_P}")
        Kp_PI = 0.45 * Ku
        Ki_PI = Tu / 1.2
        print(f'o ganho proporcional{Kp_PI} e o ganho integrativo é {Ki_PI}')
        Kp_PID = 0.68 * Ku
        Ki_PID = Tu / 2
        Kd_PID = Tu * 0.125
        print(f'o ganho proporcional{Kp_PID} e o ganho integrativo é {Ki_PID} e o ganho derivativo é {Kd_PID}')

        # Criar funções de transferência com os controladores
        numerator = [3]
        denominator = [1, 12, 25, 50]

        system_P = lti([Kp_P * n for n in numerator], denominator)
        system_PI = lti([Kp_PI * n for n in numerator], np.convolve(denominator, [1, Ki_PI]))
        system_PID = lti([Kd_PID, Kp_PID, Ki_PID], denominator)
           # Encontrar o valor do pico (ressalto)
        

        # Erro em estado estacionário (diferença entre valor final e o desejado)
        

        # Tempo para simulação
        time = np.linspace(0, 20, 1000)

        # Respostas no tempo
        _, response_P = step(system_P, T=time)
        _, response_PI = step(system_PI, T=time)
        _, response_PID = step(system_PID, T=time)
        # overshoot_p = np.max(response_P) - 1  # Desvio em relação ao valor desejado (1 é o degrau unitário)
        # overshoot_pi = np.max(response_PI) - 1  # Desvio em relação ao valor desejado (1 é o degrau unitário)
        # overshoot_pid = np.max(response_PID) - 1  # Desvio em relação ao valor desejado (1 é o degrau unitário)
        # print(f"O ressalto (overshoot) é {overshoot_p}")
        # print(f"O ressalto (overshoot) é {overshoot_pi}") 
        # print(f"O ressalto (overshoot) é {overshoot_pid}")
        
        # steady_state_error_p = abs(response_P[-1] - 1)
        # steady_state_error_pi = abs(response_PI[-1] - 1)
        # steady_state_error_pid = abs(response_PID[-1] - 1)
        # print(f"Erro em estado estacionário: {steady_state_error_p}")
        # print(f"Erro em estado estacionário: {steady_state_error_pi}")
        # print(f"Erro em estado estacionário: {steady_state_error_pid}")

        if Ku is not None:
            time = np.linspace(0, t_sim, 1000)
            
            # Sistemas de Controle
            system_P = ct.TransferFunction([Kp_P], denominator)
            system_PI = ct.TransferFunction([Kp_PI], np.convolve(denominator, [1, Ki_PI]))
            system_PID = ct.TransferFunction([Kd_PID, Kp_PID, Ki_PID], denominator)

            title = "lugar_raizes_pid.png"
            plot_root_locus(system_PID, title)   


            # # Respostas no tempo
            # plt.figure() 
            # # plot_response(system_P, time, "Response with P Control", "P Control")
            # # plot_response(system_PI, time, "Response with PI Control", "PI Control")
            # plot_response(system_PID, time, "Response with PID Control", "PID Control")
            # plt.savefig("response_pid.png")
            # plt.show()
            # omega, mag_P, phase_P = ct.bode(system_P, plot=True)
            # plt.savefig("bode_diagram-p.png")
            # omega, mag_PI, phase_PI = ct.bode(system_PI, plot=True)
            # plt.savefig("bode_diagram-pi.png")
            # omega, mag_P, phase_P = ct.bode(system_PID, plot=True)
            # plt.savefig("bode_diagram-pid.png")
            
            # Gráficos de frequência (Bode)
            # plt.figure()
            # plot_bode(system_P, "P Control")
            # plot_bode(system_PI, "PI Control")
            # plot_bode(system_PID, "PID Control")
            # # plt.legend()
            # plt.savefig("response_pid-bode.png")
            # plt.show()
            
            
            # # plt.semilogx(omega,20 * np.log10(phase_P))
            # # plt.title("Bode Plot - p")
            
            # plt.semilogx(omega, 20 * np.log10(phase_PI))
            # plt.title("Bode Plot - pi")
            
            # # plt.semilogx(omega, ,20 * np.log10(phase_PID))
            # # plt.title("Bode Plot - pid")
            
            # plt.xlabel("Frequency (rad/s)")
            # plt.ylabel("Phase (rad)")
            # plt.grid()
            # plt.tight_layout()
            # # plt.savefig("bode_diagram-Kp.png")
            # plt.show()
                        


    # Gráficos de frequência (Bode)
    # plt.figure(figsize=(12, 8))

    # # Bode plot para malha aberta
    # plt.subplot(2, 1, 1)
    # mag, phase, omega = ct.bode_plot(open_loop, dB=True, plot=False)
    # plt.semilogx(omega, 20 * np.log10(mag))
    # plt.title("Bode Plot - Open Loop")
    # plt.xlabel("Frequency (rad/s)")
    # plt.ylabel("Magnitude (dB)")
    # plt.grid()

    # Bode plot para malha fechada, se Ku foi encontrado
    if Ku is not None:
       
        mag, phase, omega = ct.bode_plot(closed_loop, dB=True, plot=False)
        plt.semilogx(omega, 20 * np.log10(mag))
        plt.title("Bode Plot - Closed Loop")
        plt.xlabel("Frequency (rad/s)")
        plt.ylabel("Magnitude (dB)")
        plt.grid()

    plt.tight_layout()
    plt.show()
