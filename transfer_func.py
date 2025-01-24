import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

# Definição dos coeficientes do numerador e denominador
numerador = 8.096 * np.convolve([1, -0.0006], [1, 0.3591])
denominador = np.convolve([1, 0.014, 0.0068], [1, 1.009, 5.56])

# Criando a função de transferência
sistema = ctrl.TransferFunction(numerador, denominador)

# Exibindo a função de transferência
print("Função de Transferência:")
print(sistema)

# Plotando a resposta ao degrau
time, response = ctrl.step_response(sistema)

plt.figure()
plt.plot(time, response)
plt.title('Resposta ao Degrau do Sistema')
plt.xlabel('Tempo (s)')
plt.ylabel('Amplitude')
plt.grid(True)
plt.show()