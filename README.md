# Zênite — Sistema Educacional de Robótica Espacial

O **Zênite** é um sistema robótico baseado em visão zenital desenvolvido para introduzir conceitos de robótica espacial a estudantes do ensino fundamental e médio por meio de atividades práticas fundamentadas na metodologia **STEAM**.

O projeto integra visão computacional, controle de trajetória, modelagem cinemática e arquitetura modular com ROS 2, aplicados em um contexto educacional inspirado em missões de exploração espacial.

---

## Contexto

O projeto foi desenvolvido no âmbito do Centro Vocacional Tecnológico Espacial (CVT-E), no IFRN – Campus Parnamirim.

O sistema foi apresentado no artigo:

> *Development of a Robotic System for the Introduction of Space Robotics Concepts in Basic Education*

O Zênite é simultaneamente uma plataforma funcional e uma ferramenta didática para ensino de robótica e conceitos de engenharia espacial.

---

## Objetivos

- Introduzir conceitos de robótica e astronáutica no ensino básico  
- Ensinar sistemas de coordenadas e controle de movimento  
- Demonstrar planejamento e execução de trajetórias  
- Aplicar visão computacional para localização de robôs  
- Simular missões inspiradas em rovers marcianos  

---

## Arquitetura do Sistema

O Zênite utiliza uma arquitetura modular baseada em ROS 2, composta pelos seguintes nós:

### 🔹 acquisition_node
Responsável pela captura das imagens da câmera zenital e aplicação de ajustes (brilho, saturação e matiz).

### 🔹 calibration_node
Realiza a calibração do ambiente por meio de homografia, convertendo pixels em coordenadas reais (metros).

### 🔹 localization_node
Executa o rastreamento do robô utilizando o algoritmo CSRT do OpenCV e converte a posição para coordenadas reais.

### 🔹 interface_node
Permite ao usuário:
- Visualizar a câmera em tempo real  
- Selecionar o destino do robô com cliques  
- Ajustar parâmetros da imagem  

### 🔹 control_node
Implementa o modelo cinemático de um robô diferencial não-holonômico:
q = [x, y, θ]^T

Calcula:
- Erro angular  
- Erro linear  
- Velocidades linear (v) e angular (ω)  
- Velocidades individuais das rodas  

Utiliza controladores proporcionais–integrais para correção contínua da trajetória.

### 🔹 transmission_node
Realiza comunicação via Bluetooth com o módulo HC-06 e converte velocidades em sinais PWM enviados ao microcontrolador.

---

## Robô

O robô possui configuração diferencial (duas rodas motrizes por lado), sendo cinematicamente equivalente a um robô móvel diferencial clássico.

### Componentes atuais:

- Arduino  
- Ponte H L298N  
- Módulo Bluetooth HC-06  

---

# Como Executar o Projeto

## Instalar o ROS 2 Jazzy

Siga as instruções oficiais de instalação para Ubuntu:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

---

## Configurar o Ambiente ROS 2

Após instalar, execute:

```bash
source /opt/ros/jazzy/setup.bash
```

Para carregar automaticamente no terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Clone o Repositório:
```bash
git clone <URL_DO_REPOSITORIO>
cd zenite_ws
colcon build
```

Após compilar:

```bash
source install/setup.bash
```

## Executando os Nós

Abra terminais separados para cada nó:

acquisition_node:
```bash
ros2 run acquisition_node acquisition_node
```

calibration_node:
```bash
ros2 run calibration_node calibration_node
```

localization_node:
```bash
ros2 run localization_node localization_node
```

interface_node:
```bash
ros2 run interface_node interface_node
```

control_node:
```bash
ros2 run control_node control_node
```

transmission_node:
```bash
ros2 run transmission_node transmission_node
```


Ou utilize os Launchers:

Para calibração:
```bash
ros2 launch zenite_launch zenite_calibration.py
```

Para execução completa:
```bash
ros2 launch zenite_launch zenite_launch.py
```
