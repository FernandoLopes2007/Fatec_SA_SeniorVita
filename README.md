# Sênior Vita 📿👴

> Sistema embarcado vestível (*wearable*) em formato de bracelete inteligente para a detecção automatizada de acidentes por quedas em idosos.

---

## 📌 Sobre o Projeto

O **Sênior Vita** é uma solução de tecnologia assistiva voltada à segurança, autonomia e monitoramento preventivo da população idosa. Desenvolvido como Trabalho de Graduação (TG) no Curso Superior de Tecnologia em Sistemas Embarcados da **Fatec Santo André**, o projeto alinha-se ao **Objetivo de Desenvolvimento Sustentável 3 (ODS 3 - Saúde e Bem-Estar)** da ONU.

O sistema utiliza sensores inerciais para a captação de dados cinemáticos em tempo real, processando variações de aceleração e velocidade angular para identificar padrões de queda de forma rápida e acionar alertas visuais, sonoros e remotos.

---

## 🛠️ Tecnologias e Componentes

### **Hardware**
* **Microcontrolador:** ESP32 (processamento em 32 bits, baixo consumo e conectividade Wi-Fi/Bluetooth integrada)
* **Sensor Inercial (IMU):** MPU6050 / MPU 5060 (acelerômetro e giroscópio de 6 eixos via I2C)
* **Interface Visual:** Display OLED (exibição de status, bateria e conectividade via I2C)
* **Armazenamento de Dados (MVP):** Módulo leitor de cartão Micro SD (gravação em `.csv` via SPI)
* **Gerenciamento de Energia:** Módulo HW-373 / TP4056 USB-C (recarga e proteção contra sobretensão/sobredescarga)
* **Atuadores e Montagem:** Buzzer passivo 5V, LED vermelho, placa de circuito impresso perfurada e jumpers.

### **Software e Ferramentas**
* **Linguagem / IDE:** C/C++ via **Visual Studio Code (VS Code)**
* **Modelagem 3D:** **Autodesk Fusion 360** (design ergonômico do invólucro do bracelete)

---

## 📁 Estrutura do Repositório

```text
├── cad/          # Arquivos e modelos 3D do invólucro (.STL, .STEP)
├── docs/         # Documentação acadêmica, esquemáticos e manuais
├── firmware/     # Códigos-fonte do ESP32 (leitura dos sensores e lógica de alerta)
└── README.md     # Apresentação do repositório
