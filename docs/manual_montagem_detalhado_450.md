# Manual de Montagem Completo Passo a Passo — Quadricóptero 450 mm ESP32-S3 & Controle Caseiro

Este guia foi escrito para funcionar como um **manual de bancada à prova de erros**. Cada etapa termina com um **Portão de Validação (Passa / Não-Passa)** usando o seu multímetro: você nunca avança para o próximo passo sem ter certeza matemática e elétrica de que o anterior está 100% correto.

---

## Etapa 0: Preparação da Bancada e Inspeção Crítica das Peças

### 0.1 Ferramentas e Consumíveis Necessários
- **Elétrica:** Ferro de solda (30W–60W), estanho 60/40 (de preferência fino 0,8 mm p/ eletrônica) + pasta/fluxo de solda, multímetro digital, alicate de corte rente, desencapador (ou estilete), fios flexíveis (22–26 AWG p/ sinais; 14–16 AWG se precisar estender potência), tubos termorretráteis (2 mm, 4 mm, 6 mm) + isqueiro/soprador.
- **Mecânica:** Régua metálica 30 cm, esquadro escolar de 90°, lápis bem apontado, punção (ou prego + martelo p/ marcar furos), serrinha manual (arco de serra c/ lâmina de metal 24 dentes — corta madeira e alumínio liso!), furadeira manual, brocas **3,0 mm** (ou **3,2 mm**) e **8,0 mm**, chave Philips/Allen M3, chave de boca 5,5 mm (ou alicate de bico p/ porca M3), lixa 120/220 e cola Super Bonder (cianoacrilato líquido).

---

### 0.2 Inspeção e Ajuste Obrigatório da Antena Externa nos 2× ESP32-S3
Antes de soldar qualquer coisa, olhe para o módulo metálico no topo das suas duas placas **ESP32-S3**:

```
CASO A: Módulo ESP32-S3-WROOM-1U Original        CASO B: Placa Clone YD-ESP32-S3 N16R8
   +-----------------------+                        +-----------------------+
   |   (SEM trilha preta)  |                        |  |_|¯|_|¯|_ (Ant PCB) | <-- Tem antena impressa!
   |    (o) <- Só IPEX     |                        |  [1][2][3]   (o) IPEX | <-- Jumper de 3 pads!
   | +-------------------+ |                        | +-------------------+ |
   | | Blindagem Metálica| |                        | | Blindagem Metálica| |
```

- **Se for o CASO A (`WROOM-1U` puro):** O conector IPEX já está ativo de fábrica. Basta encaixar o cabo *pigtail* IPEX pressionando reto com a unha até ouvir um *"click"* firme.
- **Se for o CASO B (`YD-ESP32-S3` com antena impressa + IPEX):**
  1. Procure com lupa/câmera de celular com zoom um minúsculo resistor `0 Ω` logo ao lado do conector IPEX ligando o pino central do chip à trilha da antena impressa.
  2. Com a ponta fina do ferro de solda limpa, mova esse resistor (ou remova-o e faça uma micro-gota de solda) ligando o **pad central** ao **pad que vai para o conector IPEX** (verifique no multímetro em escala de continuidade: o pino central do conector IPEX **não** pode dar curto com a carcaça dourada externa do IPEX, a menos que a antena seja do tipo dipolo dobrado — meça antes de conectar a antena!).
- **Regra de Ouro de RF:** **NUNCA ligue o ESP32-S3 transmitindo ESP-NOW sem a antena rosqueada no pigtail SMA**, pois a potência refletida pode danificar o amplificador de RF interno do chip!

---

## Etapa 1: Construção Mecânica do Chassi Híbrido 450 mm

### 1.1 Gabarito Exato das 2 Chapas Centrais (`120 × 120 mm`, Compensado 4 mm)
O truque para ter precisão de máquina CNC usando ferramentas manuais é **furar as duas chapas juntas** e aproveitar que **as diagonais de um quadrado perfeito já se cruzam exatamente a 90°**!

1. Risque com esquadro e régua **2 quadrados perfeitos de `120 mm × 120 mm`** no compensado de `4 mm` e serre reto.
2. Junte as duas chapas uma sobre a outra, alinhe as bordas perfeitamente e enrole **fita crepe bem apertada** nas 4 bordas (elas agora viram um bloco único).
3. Na face de cima, trace com lápis fino as **duas diagonais de canto a canto** formando um grande `"X"`. O cruzamento exato é o centro `(0, 0)`.
4. A partir do centro `(0, 0)`, meça com a régua sobre **cada uma das 4 meias-diagonais** (em direção aos 4 cantos) e marque com um pontinho + furo guia de prego:
   - **Furo Interno de cada braço:** a exatos **`32,0 mm`** do centro `(0, 0)`.
   - **Furo Externo de cada braço:** a exatos **`68,0 mm`** do centro `(0, 0)`.
5. Marque também **4 furos auxiliares para fixar a Placa Controladora (`70 × 90 mm`)** alinhados com os lados retos do quadrado (a Frente do drone será o **lado reto** entre os braços Frente-Esquerda e Frente-Direita!):
   - Do centro `(0, 0)`, marque um retângulo centrado de **`60 mm` (largura) × `80 mm` (comprimento)** (`X = ±30 mm`, `Y = ±40 mm`).
6. Faça um furo central de passagem de fios no ponto `(0, 0)` (ou abra uma janelinha central de `25 × 25 mm` apenas na **chapa superior** para subir os fios de bateria/BEC da PDB se desejar, mantendo ambos os furos de `32 mm` intactos!).
7. **Furação:** Fure todos os 8 furos das diagonais e os 4 furos da controladora com **broca `3,0 mm` (ou `3,2 mm`)** mantendo a furadeira bem em pé (90°).
8. **Importante antes de tirar a fita crepe:** Risque com canetinha uma seta **`FRENTE ↑`** nas duas chapas simultaneamente! Assim, mesmo que sua mão tenha desviado 0,5 mm ao furar, a chapa de cima e a de baixo continuam sendo espelhos idênticos!
9. *(Opcional estético/peso):* Chanfre `10 mm` da pontinha viva dos 4 cantos e lixe as bordas.

```
          VISTA SUPERIOR DA CHAPA CENTRAL (120 x 120 mm)
                      [ FRENTE DO DRONE ↑ ]
       Canto FL (M4)                         Canto FR (M1)
            +---------------------------------------+
            |\  (68mm)                     (68mm)  /|
            | \   O                           O   / |
            |  \      o (-30,+40)   (+30,+40) o  /  |
            |   \       +-----[7x9 FC]-----+    /   |
            |    \  O   |                  | O /    |  <-- Furos internos a 32 mm do (0,0)
            |     \32mm |       (0,0)      |  /     |  <-- Furos externos a 68 mm do (0,0)
   ESQ      |           |         +        |        |      DIR
            |     /32mm |                  |  \     |
            |    /  O   +------------------+ O \    |
            |   /       o (-30,-40)   (+30,-40) o\  |
            |  /                                  \ |
            | /   O                           O   \ |
            |/  (68mm)                     (68mm)  \|
            +---------------------------------------+
       Canto RL (M3)                         Canto RR (M2)
```

---

### 1.2 Gabarito Exato dos 4 Braços (`225 mm` cada — Madeira Fase 1 / Alumínio Fase 2)
De uma barra de `1 metro` (`1000 mm`), você corta **4 braços idênticos de `225 mm` (`22,5 cm`)**.

```
   GABARITO DE CADA BRAÇO (225 mm de comprimento total — vista de topo no eixo central)

   |<-------------------- 225 mm -------------------->|
   +--------------------------------------------------+
   |    O          O                         O (Ø8) O |
   +--------------------------------------------------+
   0   12mm       48mm                    197mm 205 213mm   225mm
   ^    ^          ^                        ^    ^    ^
   |    |          |                        |    |    +-- Furo Motor Ext (M3: 213mm p/ base 16mm, ou 214,5mm p/ 19mm)
   |    |          |                        |    +------- Alívio trava do eixo (Ø 8 mm, 4 mm prof.) = Raio 225 mm do drone!
   |    |          |                        +------------ Furo Motor Int (M3: 197mm p/ base 16mm, ou 195,5mm p/ 19mm)
   |    |          +------------------------------------- Furo Chassi Externo (casa c/ 68 mm da chapa)
   |    +------------------------------------------------ Furo Chassi Interno (casa c/ 32 mm da chapa)
   +----------------------------------------------------- Ponta interna (fica a 20 mm do centro (0,0))
```

1. **Conferência dos furos na base do seu motor A2212:** Olhe embaixo do motor A2212 com a régua: ele tem 2 furos `M3` espaçados de **`16,0 mm`** e outros 2 espaçados de **`19,0 mm`**. Escolha o par de **`16,0 mm`** (furos em `197,0 mm` e `213,0 mm`) ou o de **`19,0 mm`** (`195,5 mm` e `214,5 mm`) — use o **mesmo padrão nos 4 braços**.
2. **Alívio central (`205,0 mm`):** Embaixo do motor A2212 sai a pontinha do eixo com um anel elástico (*circlip*). Faça um rebaixo/furo central com **broca `8,0 mm`** na marca de **`205,0 mm`** para o motor assentar 100% reto na madeira/alumínio sem ficar bambo sobre o eixo!
3. **Truque de Ouro para Endurecer os Furos da Madeira:** Pingue **2 gotas de Super Bonder líquido dentro de cada furo `3 mm` da madeira** (sem parafuso dentro!) e espere **15 minutos secar completamente**. A madeira absorve a resina e vira um plástico duro ao redor do furo — não esmaga nem cria folga com vibração!
4. *(Quando migrar para Alumínio `1/2"` na Fase 2):* Corte 4 toquinhos de `55 mm` de madeira lixada que entrem justos dentro da base do tubo de alumínio antes de furar os furos `12 mm` e `48 mm` — isso impede que o aperto do parafuso M3 amasse o tubo oco!

---

### 1.3 Fixação dos Motores A2212 (ATENÇÃO MÁXIMA AO COMPRIMENTO DO PARAFUSO!)
> [!CAUTION]
> **PERIGO DE CURTO NO MOTOR:** O parafuso `M3` que sobe por baixo do braço e entra na base metálica do motor A2212 deve entrar **apenas `3,0 mm a 3,5 mm` na rosca do motor**! Se usar um parafuso comprido demais, a ponta dele toca no enrolamento de cobre lá dentro, arranca o verniz e **queima o motor e o ESC instantaneamente**!
> - **Cálculo exato:** Se o seu braço de madeira tem `14 mm` + arruela `0,8 mm`, um parafuso `M3 × 18 mm` sobra `3,2 mm` de rosca (perfeito!). Se usar `M3 × 20 mm`, coloque **2 ou 3 arruelas M3** por baixo até sobrar exatos `3 mm` de ponta antes de rosquear no motor, e **sempre olhe por baixo das aberturas do motor com uma lanterna** para confirmar que o parafuso está longe do cobre!

### 1.4 Montagem do Chassi + Pés de Pouso
1. Monte o sanduíche (`Chapa Inferior` → `4 Braços` → `Chapa Superior`) usando **8× parafusos `M3 × 30 mm`**, arruelas nas duas faces e **porcas autotravantes `M3 Parlock`** embaixo.
2. Aperte de forma cruzada apoiando o chassi sobre uma **mesa perfeitamente plana** (piso de porcelanato ou tampo de vidro): os 4 braços devem tocar a mesa sem mancar!
3. Prenda **4 pedaços de `6 cm` de tubo de espuma isolante** (ou curvas de PVC fino) sob as pontas dos 4 braços com abraçadeiras de nylon — serão seus amortecedores de pouso.
4. Prenda a **cinta de Velcro da Bateria** passando por fendas na **Chapa Inferior** (a bateria vai embaixo da barriga, bem no centro `(0, 0)`, sobre um quadradinho de EVA antiderrapante).

> ✅ **PORTÃO DE VALIDAÇÃO 1 (Mecânica):**
> - Apoie o chassi na mesa plana: os 4 topos de eixo dos motores têm a **mesma altura até a mesa** (diferença `< 1,5 mm`)?
> - Meça com trena a distância entre motores vizinhos (`M1–M2`, `M2–M3`, `M3–M4`, `M4–M1`): todas estão em **`318 mm ± 3 mm`**?
> - Meça no multímetro (escala de continuidade/resistência) entre a **carcaça metálica de cada motor** e **cada um dos 3 fios do motor**: deve dar **Circuito Aberto (`OL` / infinito)**! *(Se apitar, um parafuso M3 encostou no cobre interno — recue imediatamente!).*

---

## Etapa 2: Montagem de Potência (Mini PDB-XT60 + 4× ESC 30A)

1. **Soldagem na Bancada (Fora do Drone é mais fácil!):**
   - Solde o conector **XT60 Macho** nos pads principais `+` e `-` da **Mini PDB-XT60** (ou use rabicho de fio silicone `14 AWG` de `8 cm` até o XT60 para facilitar plugar a bateria sem forçar a placa!).
   - Solde o **Capacitor `1000 µF 25V Low-ESR`** diretamente nos pads principais de entrada da bateria na PDB (**atenção redobrada à faixa branca negativa `-` do capacitor → pad `GND / -`!**).
2. **Instalação dos 4 ESCs nos Braços:**
   - Prenda cada **ESC 30A** com 2 abraçadeiras de nylon na lateral/topo do respectivo braço (no fluxo de ar da hélice para resfriar!), mas **sem deixar fio solto que possa subir na hélice**.
   - Solde os fios grossos **Vermelho (`+`)** e **Preto (`-`)** de cada um dos 4 ESCs nos 4 pares de pads `ESC + / -` da Mini PDB. Estanhe primeiro o pad, depois o fio, e funda os dois deixando a solda brilhante (nunca opaca/fria).
   - Solde **2 fios flexíveis (`22 AWG`, `12 cm`)** nos pads da saída regulada **`5V`** e **`GND`** do BEC da Mini PDB, e mais **1 fio fino (`26 AWG`, `12 cm`)** num pad **`VCC / BAT+`** (para o sensor de tensão da bateria na controladora).
3. **PASSO OBRIGATÓRIO NOS CABOS DE SINAL DOS 4 ESCs:**
   - Cada ESC tem um cabinho fino de 3 cores tipo servo (`Branco/Amarelo = Sinal`, `Vermelho = +5V BEC`, `Preto/Marrom = GND`).
   - Com a ponta de um alfinete ou estilete, **levante a travinha plástica do pino central VERMELHO (`+5V`) nos 4 conectores dos ESCs, puxe o terminal metálico para fora e encape cada um individualmente com tubo termorretrátil**! *(Não corte — apenas saque e isole; assim o ESC fica intacto se quiser usar em outro projeto, e nunca haverá briga de tensão com o 5 V da PDB!).*
4. **Fixação da PDB no Chassi:**
   - Fixe a Mini PDB no centro da **Chapa Superior** sobre espaçadores plásticos M3 (ou fita dupla-face isolante grossa — **garanta que nenhum ponto de solda embaixo da PDB encoste em cabeça de parafuso metálico!**).
   - Conecte os 3 conectores banana `3,5 mm` de cada ESC no respectivo motor (se depois na bancada um motor girar ao contrário, basta inverter **quaisquer 2 dos 3 fios banana** daquele motor!).

> ✅ **PORTÃO DE VALIDAÇÃO 2 (Potência — ANTES DE LIGAR BATERIA!):**
> 1. Multímetro em **Continuidade / Resistência (`Ω`)** nos pinos do **XT60**: meça entre `+` e `-`. *(O capacitor vai carregar subindo a resistência até `kΩ` ou `OL` — **JAMAIS pode dar `0 Ω` fixo / apito contínuo!**).*
> 2. **Primeira energização isolada (SEM a placa do ESP32 conectada!):** Plugue a LiPo 3S na PDB (você ouvirá os 3 bipes musicais dos motores indicando 3 células!).
> 3. Meça com o multímetro (escala `20V DC`) nos fios que saem do BEC `5V` e `GND` da PDB: deve ler **`4,95 V a 5,10 V` estáveis**. Desconecte a bateria!

---

## Etapa 3: Montagem da Placa Controladora do Drone (`7 × 9 cm` FR4)

### 3.1 Mapa Físico na Placa Perfurada `70 × 90 mm` (`26 colunas × 33 linhas`)
Posicione a placa com o lado de `90 mm` alinhado Frente-Trás:
- A **IMU (`ICM-42688-P` ou `BMI270`)** fica **exatamente no centro geométrico da placa** (e consequentemente no centro `(0, 0)` do drone!), com os pinos alinhados e você anotando para qual lado aponta o eixo `+X / +Y` serigrafado nela!
- O **`ESP32-S3-DevKitC-1`** vai encaixado sobre **2 barras de pinos fêmea de 22 pinos** deslocadas para um lado, com as portas USB-C apontando para a **Traseira** (fácil de plugar o cabo USB no computador sem bater em braços!).
- O **`SN74HCT125N`** vai num **soquete DIP-14** na lateral oposta (solde o soquete vazio; só encaixe o CI no final!).

```
             LAYOUT VISTA DE TOPO — PLACA PERFURADA 70 x 90 mm (DRONE)
                             [ ↑ FRENTE DO DRONE ↑ ]
   +-----------------------------------------------------------------------+
   | (O) Furo M3 Damper                             Furo M3 Damper (O)     |
   |                                                                       |
   |   [Header 6p GPS/Mag Fase 2]          [BMP280 I2C c/ espuma preta!]   |
   |   (5V, GND, TX17, RX18, SDA8, SCL9)                                   |
   |                                                                       |
   |   +--------------------+              +---------------------------+   |
   |   |                    |              |  [M4 FL] [M1 FR]          |   |
   |   |                    |   +------+   |  [M3 RL] [M2 RR] (4x 2p)  |   |
   |   |   ESP32-S3-WROOM   |   | IMU  |   |                           |   |
   |   |     DevKitC-1      |   | SPI  |   |   +-------------------+   |   |
   |   |  (2x Header 22p)   |   |(0,0)!|   |   | Soquete DIP-14    |   |   |
   |   |                    |   +------+   |   |   SN74HCT125N     |   |   |
   |   |   [Conector IPEX]  |              |   +-------------------+   |   |
   |   |                    |              |                           |   |
   |   |                    |              |  [Diodo 1N5819] [Buzzer]  |   |
   |   |                    |              |  [Divisor VBAT 33k / 10k] |   |
   |   |   [USB]   [UART]   |              |  [Borne/Header PDB 5V/BAT]|   |
   |   +--------------------+              +---------------------------+   |
   |                                                                       |
   | (O) Furo M3 Damper                             Furo M3 Damper (O)     |
   +-----------------------------------------------------------------------+
                             [ ↓ TRASEIRA DO DRONE ↓ ]
```

---

### 3.2 Tabela Mestre Ponto-a-Ponto de Soldagem (Conferir Linha por Linha!)

#### Bloco A — Alimentação e Sensor de Bateria
1. **Barramento `GND`:** Crie uma trilha contínua de estanho/fio nu por baixo da placa interligando: `GND da PDB` ↔ `Pinos GND do ESP32-S3` ↔ `Pino 7 do 74HCT125N` ↔ `GND dos 4 ESCs` ↔ `GND da IMU / BMP280 / Header GPS / Buzzer`.
2. **Entrada `+5V` Protegida (`+5V_SYS`):**
   - `Fio +5V da PDB` → **Ânodo** do diodo Schottky **`1N5819`** → **Cátodo (faixa prateada)** vai para a linha **`+5V_SYS`**.
   - Ligue **`+5V_SYS`** em: `Pino 5V (IN)` do ESP32-S3, `Pino 14 (VCC)` do `74HCT125N`, `VCC` do Buzzer 5V e `Pino 5V` do Header GPS.
   - Solde **1 capacitor cerâmico `100 nF` (104)** bem rente entre o **`Pino 14 (VCC)`** e o **`Pino 7 (GND)`** do soquete do `74HCT125N`!
3. **Barramento Limpo `+3,3V` (Sensores):**
   - Puxe do pino **`3V3`** do ESP32-S3 → solde um capacitor **`10 µF`** + **`100 nF`** para `GND` perto da IMU → alimente **`VCC/3V3` da IMU (`ICM-42688-P` / `BMI270`)** e **`VCC` do `BMP280`** *(NUNCA ligue 5 V nesses dois módulos!)*.
4. **Divisor de Tensão da Bateria 3S (`VBAT` → `GPIO1`):**
   - `Fio BAT+ (12,6 V máx da PDB)` → **Resistor `33 kΩ`** → nó central vai no **`GPIO1` do ESP32-S3** → do mesmo `GPIO1` vai **Resistor `10 kΩ`** em paralelo com **Capacitor `100 nF`** para **`GND`**.
   - *Fórmula de segurança:* Em carga máxima (`12,6 V`), $V_{GPIO1} = 12,6 \times \frac{10}{33 + 10} = \mathbf{2,93\text{ V}}$ (`< 3,1 V` limite linear do ADC1 com `ADC_11db`!).

#### Bloco B — Buffer `SN74HCT125N` (DIP-14, Chanfro `U` p/ Cima) e 4 ESCs
| Canal Motor | Posição Física & Giro | Saída ESP32-S3 | Resistor Pull-Down | Pino Entrada `74HCT125` | Pino Enable (`OE#`) | Pino Saída `74HCT125` (5 V) | Destino |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **Motor 1** | Frente-Direita (**FR, CCW ↺**) | **`GPIO4`** | `10 kΩ` → `GND` | **Pino 2 (`1A`)** | **Pino 1 (`1OE#`) → `GND`** | **Pino 3 (`1Y`)** | Sinal ESC 1 (+ `GND`) |
| **Motor 2** | Traseira-Direita (**RR, CW ↻**) | **`GPIO5`** | `10 kΩ` → `GND` | **Pino 5 (`2A`)** | **Pino 4 (`2OE#`) → `GND`** | **Pino 6 (`2Y`)** | Sinal ESC 2 (+ `GND`) |
| **Motor 3** | Traseira-Esquerda (**RL, CCW ↺**) | **`GPIO6`** | `10 kΩ` → `GND` | **Pino 9 (`3A`)** | **Pino 10 (`3OE#`) → `GND`** | **Pino 8 (`3Y`)** | Sinal ESC 3 (+ `GND`) |
| **Motor 4** | Frente-Esquerda (**FL, CW ↻**) | **`GPIO7`** | `10 kΩ` → `GND` | **Pino 12 (`4A`)** | **Pino 13 (`4OE#`) → `GND`** | **Pino 11 (`4Y`)** | Sinal ESC 4 (+ `GND`) |

*(Nota: Olhe com atenção os pinos do lado direito `8 a 13` do `74HCT125N` — a saída `Y` fica embaixo da entrada `A`! Pino 7 = `GND`, Pino 14 = `+5V_SYS`).*

#### Bloco C — Sensores SPI, I2C, GPS Reservado e Buzzer
- **IMU SPI (`ICM-42688-P` ou `BMI270`, fios curtos `< 4 cm`!):**
  - `VCC / 3V3` → `3V3` do ESP32-S3 | `GND` → `GND`
  - `CS` → **`GPIO10`**
  - `MOSI / SDI / SDA` → **`GPIO11`**
  - `SCK / SCLK / SCL` → **`GPIO12`**
  - `MISO / SDO / AD0` → **`GPIO13`**
  - `INT1 / INT` → **`GPIO14`** *(se o módulo tiver pino `FSYNC`, ligue `FSYNC` ao `GND`!)*
- **Barômetro `BMP280` (I2C):**
  - `VCC` → `3V3` | `GND` → `GND` | `SDA` → **`GPIO8`** | `SCL` → **`GPIO9`**
  - *(Se tiver pino `CSB`, deixe livre/ligado em `3V3`; ligue `SDO` ao `GND` para fixar o endereço I2C em `0x76`).*
  - **Truque essencial:** Cole um quadradinho de **espuma porosa escura** (tipo espuma de microfone ou esponja macia respirável — **sem cola tapando o furinho metálico do sensor!**) sobre o `BMP280` para bloquear rajadas de vento das hélices!
- **Header de 6 Pinos Reservado para Fase 2 (GPS u-blox M10 + Bússola QMC5883L):**
  - Pinos em fila: `1: +5V_SYS` | `2: GND` | `3: GPIO17 (TX→RX_GPS)` | `4: GPIO18 (RX←TX_GPS)` | `5: GPIO8 (SDA)` | `6: GPIO9 (SCL)`.
- **Buzzer Ativo:** `VCC` → `+5V_SYS` | `GND` → `GND` | `I/O` → **`GPIO21`**.

### 3.3 Isolamento Mecânico Anti-Vibração da Placa `7 × 9 cm`
- Instale os **4× Dampers M3 de borracha** nos 4 furos das quinas (`±30 mm, ±40 mm`) da Chapa Superior, encaixando a placa `7 × 9 cm` suspensa acima da Mini PDB! *(Se usar espuma `3M VHB` em vez de dampers M3, passe 2 abraçadeiras de nylon **frouxas** como "cinto de segurança" para a placa nunca soltar em voo).*
- Prenda a **Antena 2.4 GHz** apontando **verticalmente para cima ou para baixo a 90°** num tubinho plástico/suporte num braço traseiro, longe de fios de alta corrente.

> ✅ **PORTÃO DE VALIDAÇÃO 3 (Placa Controladora — COM OS CIs AINDA DESENCAIXADOS!):**
> 1. **Sem o ESP32-S3 e sem o `74HCT125N` nos soquetes**, verifique continuidade: há curto entre `+5V_SYS` e `GND`? Entre `3V3` e `GND`? Entre `3V3` e `+5V_SYS`? *(Todos devem dar circuito aberto!).*
> 2. Ligue a bateria na PDB (com os CIs ainda fora!):
>    - Pino `5V` do soquete do ESP32-S3 e Pino `14` do soquete DIP-14 medem **`~4,70 V a 4,85 V`**?
>    - Pino **`GPIO1`** do soquete do ESP32-S3 mede **`~2,5 V a 2,93 V`** (nunca acima de `3,1 V`!)?
>    - Pinos `VCC` da IMU e do BMP280 medem **`0 V`** (provando que não receberam 5 V por engano)?
> 3. Desligue a bateria, **encaixe o `ESP32-S3` (USB p/ trás!), o `74HCT125N` (chanfro `U` p/ frente!), a IMU e o BMP280**, conecte a antena externa e meça na energização se o pino `3V3` crava em **`3,28 V a 3,33 V`**.

---

## Etapa 4: Construção do Controle Caseiro (Estação Base + Ponte MAVLink)

```
       PAINEL FRONTAL DO CONTROLE CASEIRO (Caixa Patola ~15 x 10 x 5 cm)
    +-------------------------------------------------------------------+
    |  [SW1: ARM / KILL]        [Antena SMA ↑]       [SW2: MODO 3-POS]  |
    |   (Cima=DISARMED!)                              (Stab/Alt/GPS)    |
    |                        +------------------+                       |
    |      (Gimbal Esq.)     |  OLED 0.96" I2C  |      (Gimbal Dir.)    |
    |         ( | )          | BAT:11.8V RSSI:-62|         ( + )        |
    |       Throttle ↑↓      | MODE: STABILIZE  |        Pitch ↑↓       |
    |       (sem mola!)      +------------------+        Roll  ←→       |
    |         Yaw ←→              [Botão AUX]           (com mola)      |
    +-------------------------------------------------------------------+
                               [Saída USB-C OTG ↓ p/ Celular/Tablet]
```

### 4.1 Montagem Mecânica na Caixa Plástica
1. Faça os dois recortes circulares frontais (~`38–42 mm` conforme seu gimbal) com broca fina fazendo furinhos em círculo + estilete/lixa meia-cana, e os 4 furinhos `M2,5 / M3` de fixação de cada gimbal:
   - **Esquerda (Modo 2 padrão mundial):** Gimbal **SEM mola** no eixo vertical (**Throttle**) e com mola na horizontal (**Yaw**).
   - **Direita:** Gimbal **COM mola** nos dois eixos (**Pitch** vertical e **Roll** horizontal).
2. Instale no topo esquerdo (ao alcance imediato do dedo indicador esquerdo!) a chave alavanca **`SW1` (ARM / KILL-SWITCH)**, no topo direito a chave 3 posições **`SW2` (Modo de Voo)**, no centro a janelinha do **OLED `0,96"`**, na borda superior o conector **SMA da Antena 2.4 GHz**, e na borda inferior/traseira um rasgo alinhado com a porta **`USB` nativa** do ESP32-S3 para plugar o cabo OTG do celular/tablet!

### 4.2 Ligações Elétricas do Controle (`ground_tx`)
1. **Os 4 Potenciômetros dos Gimbals (Somente em `3,3 V`!):**
   - Cada potenciômetro tem 3 terminais: solde os terminais laterais de todos os 4 em **`3V3`** e **`GND`** do ESP32-S3.
   - Solde cada pino central (*wiper*) no respectivo pino ADC1, colocando **1 capacitor cerâmico `100 nF` entre cada pino ADC e `GND`** (elimina 99% do chiado elétrico de potenciômetro!):
     - Throttle → **`GPIO1`** | Yaw → **`GPIO2`** | Pitch → **`GPIO4`** | Roll → **`GPIO5`**.
2. **Chaves com Lógica *Fail-Safe* (Segurança):**
   - **Kill-Switch (`SW1`):** Ligue um terminal ao `GND` e o outro ao **`GPIO10`** (usaremos `INPUT_PULLUP` no código: circuito **aberto = DESARMADO**, fechado para `GND` = solicita armar — assim, se um fio quebrar por dentro do controle, o drone **desarma** em vez de ficar travado armado!).
   - **Chave 3 Posições (`SW2`):** Pino central no `GND`, pino lateral 1 no **`GPIO11`**, pino lateral 2 no **`GPIO12`**.
   - **Botão Auxiliar:** Entre **`GPIO13`** e `GND`.
   - **Buzzer:** Entre **`GPIO14`** e `GND` (+ `VCC` se ativo 3 pins).
3. **Display OLED `0,96"`:** `GND → GND`, `VCC → 3V3`, `SDA → GPIO8`, `SCL → GPIO9`.
4. **Alimentação + Proteção USB-OTG:**
   - Se usar **18650 + TP4056 + Boost 5V**: passe a saída `+5V` do Boost por **1 diodo Schottky `1N5819`** antes de entrar no pino `5V (IN)` do ESP32-S3 (assim, quando você plugar o celular/tablet via cabo OTG na porta `USB`, o celular e o módulo Boost não entram em curto!), e ligue o divisor `100 kΩ + 100 kΩ` do positivo da célula `18650` (`4,2 V`) no **`GPIO6`** para mostrar a porcentagem de bateria do controle no OLED.

> ✅ **PORTÃO DE VALIDAÇÃO 4 (Controle Caseiro):**
> - Com o controle ligado via USB, mova cada eixo dos gimbals devagar de ponta a ponta medindo a tensão no pino ADC com o multímetro: a tensão varia suavemente (sem saltos bruscos p/ `0 V` ou `3,3 V` no meio do caminho) e nunca ultrapassa `3,3 V`?

---

## Etapa 5: Construção da Bancada "Gangorra 1-Eixo" (Segurança Total no PID!)

> [!IMPORTANT]
> **NUNCA tente o primeiro voo livre de um código PID recém-escrito soltando um drone 450 mm direto do chão!** Se um sinal do giroscópio ou um par de motores estiver invertido, o drone dá um mortal violento em 0,2 segundo. Na **Gangorra 1-Eixo**, você descobre qualquer inversão e deixa o PID 90% calibrado com **zero risco**!

```
                  BANCADA GANGORRA 1-EIXO (Feita com sobras de madeira)

                     (M4)=========[ DRONE ]=========(M1)
                                     ||  <-- Preso c/ abraçadeiras sobre ripa giratória
                           ==========()==========  <-- Parafuso M4/M5 servindo de eixo livre!
                                    /  \
                                   /    \   <-- 2 Pilares de madeira (25 cm altura, livres das hélices!)
                        ========================  <-- Base pesada (tábua 40x40 cm ou presa na mesa c/ sargento)
```

1. Pregue/parafuse **2 colunas verticais de madeira (`25 cm` de altura)** afastadas `35 cm` uma da outra sobre uma base pesada (ou prenda a base na varanda/mesa externa com sargento/tijolos).
2. Passe uma barra/haste (ou 2 parafusos longos alinhados nas pontas de uma ripa central de `34 cm`) funcionando como um **eixo de gangorra sem atrito**, a `25 cm` do chão.
3. Prenda o centro do drone sobre essa ripa com abraçadeiras de nylon alinhando o eixo da gangorra primeiro com o eixo Frente-Trás (**para calibrar Roll: motores Esquerda vs. Direita**), limitando a inclinação máxima com um barbante frouxo em `±35°` para as hélices nunca tocarem na mesa!
4. Depois que o eixo **Roll** estiver travado no nível horizontal como uma rocha, gire o drone `90°` na ripa e repita para o eixo **Pitch**!

---

## Etapa 6: Os 4 Portões Finais Pré-Voo (Quando Carregarmos o Firmware)

Estes são os testes em sequência obrigatória que faremos juntos assim que gravarmos o código:

1. **Portão 5 — Orientação da IMU na Mão (Sem Bateria LiPo, só USB):**
   - Conectado ao **QGroundControl**, inclinar o nariz do drone para baixo mostra *Pitch negativo*? Inclinar para a direita mostra *Roll positivo*? Girar o nariz no sentido horário mostra *Yaw subindo*? *(90% das capotagens no 1º voo vêm de sinal invertido aqui!).*
2. **Portão 6 — Sentido e Ordem dos Motores SEM HÉLICES:**
   - Com as hélices **guardadas na gaveta**, ligar a LiPo e acionar pelo CLI de teste 5% de potência em cada motor individualmente (encostando um pedacinho de fita crepe no eixo para ver o lado que gira):
     - `Motor 1 (Frente-Direita)` gira **Anti-Horário (CCW ↺)**?
     - `Motor 2 (Traseira-Direita)` gira **Horário (CW ↻)**?
     - `Motor 3 (Traseira-Esquerda)` gira **Anti-Horário (CCW ↺)**?
     - `Motor 4 (Frente-Esquerda)` gira **Horário (CW ↻)**?
     - *(Se qualquer motor girar ao contrário, desligue a bateria e inverta 2 dos 3 plugues banana daquele motor!).*
3. **Portão 7 — Teste de Vibração na Mão + Kill-Switch + Failsafe (Ainda SEM HÉLICES):**
   - Acelerar a 40% olhando o gráfico do giroscópio; acionar o **Kill-Switch** (deve cortar instantaneamente); religar a 20% e **desligar a chave de energia do Controle Caseiro** (em `< 500 ms` os 4 motores devem cortar sozinhos em Failsafe e o buzzer do drone apitar!).
4. **Portão 8 — Sintonia na Gangorra 1-Eixo (Primeira vez COM Hélices, área ventilada e óculos de proteção):**
   - Hélices `1045 Nylon-Carbono` instaladas com as **letras/números impressos na pá SEMPRE voltados para o CÉU** (tanto nas CW quanto nas CCW!) e adaptadores apertados com chave.
   - Sintonizar `Rate P → Rate D → Rate I → Angle P` na gangorra antes de soltar para o primeiro voo livre!
