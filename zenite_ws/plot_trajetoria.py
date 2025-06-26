import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv('trajetoria.csv')

plt.figure(figsize=(12,8))

plt.subplot(4,1,1)
plt.plot(df['tempo'], df['x'], label='x(t)', color='blue')
plt.xlabel('Tempo (s)')
plt.ylabel('Posição X (m)')
plt.title('Posição X em função do tempo')
plt.grid()

plt.subplot(4,1,2)
plt.plot(df['tempo'], df['y'], label='y(t)', color='green')
plt.xlabel('Tempo (s)')
plt.ylabel('Posição Y (m)')
plt.title('Posição Y em função do tempo')
plt.grid()

plt.subplot(4,1,3)
plt.plot(df['tempo'], df['theta'], label='theta(t)', color='red')
plt.xlabel('Tempo (s)')
plt.ylabel('Ângulo Theta (graus)')
plt.title('Orientação Theta em função do tempo')
plt.grid()

plt.subplot(4,1,4)
plt.plot(df['x'], df['y'])
plt.title('Trajetória do Robô (x vs y)')
plt.xlabel('Posição x (m)')
plt.ylabel('Posição y (m)')
plt.grid()

plt.tight_layout()
plt.show()
