# Manual de Montagem Milimétrico Passo a Passo v2.1 — Quadricóptero 450 mm ESP32-S3 & Controle Long-Range

Este guia foi projetado como um **manual de bancada industrial à prova de erros**. Cada etapa termina com um **Portão de Validação (Passa / Não-Passa)** com valores numéricos exatos de multímetro e comandos de CLI: você nunca avança para o próximo passo sem comprovar fisicamente que o anterior está 100% seguro.

---

## Etapa 0: Preparação da Bancada e Inspeção Obrigatória Pré-Montagem

### 0.1 Ferramentas e Consumíveis Necessários
- **Elétrica:** Ferro de solda (`30W–60W` ponta fina), estanho `60/40` (`0,8 mm` c/ resina) + pasta/fluxo *no-clean*, multímetro digital, alicate de corte rente, desencapador, fios flexíveis de cobre estanhado/silicone (`24–26 AWG` p/ sinais; `22 AWG` p/ BEC `5V`; `14 AWG` p/ bateria XT60), tubos termorretráteis (`2 mm, 4 mm, 6 mm`).
- **Mecânica:** Régua metálica `30 cm`, esquadro de `90°`, lápis `0,5 mm`, punção (ou prego fino + martelo p/ marcar centro de furos), arco de serra manual c/ lâmina `24 dentes/pol` (corta madeira e alumínio reto), furadeira, brocas **`3,0 mm` (ou `3,2 mm`)** e **`8,0 mm`**, chave Philips/Allen `M3`, chave canhão/boca `5,5 mm` (porca `M3`), lixas `120` e `220` e cola cianoacrilato líquida (*Super Bonder* fluido, não em gel).

---

### 0.2 Os 5 Testes de Recebimento na Mesa (5 Minutos que Salvam Peças!)

#### 1. Conferência da Antena Externa IPEX nos 2× `ESP32-S3`
Olhe para a blindagem metálica no topo das suas duas placas `ESP32-S3`:

```text
CASO A: Módulo ESP32-S3-WROOM-1U Original        CASO B: Placa Clone YD-ESP32-S3 N16R8
   +-----------------------+                        +-----------------------+
   |   (SEM trilha preta)  |                        |  |_|¯|_|¯|_ (Ant PCB) | <-- Tem antena impressa!
   |    (o) <- Só IPEX     |                        |  [1][2][3]   (o) IPEX | <-- Jumper de 3 pads!
   | +-------------------+ |                        | +-------------------+ |
   | | Blindagem Metálica| |                        | | Blindagem Metálica| |
```

- **CASO A (`WROOM-1U` autêntico):** O conector IPEX já vem ativo de fábrica. Basta alinhar o *pigtail* IPEX1 perfeitamente plano e pressionar com a unha até ouvir um *"click"* firme.
- **CASO B (`YD-ESP32-S3` com antena PCB + IPEX):** Vem de fábrica ligada na antena impressa! Com lupa/câmera macro, localize o resistor minúsculo `0 Ω` ao lado do conector IPEX e mova-o (ou faça uma micro-ponte limpa de estanho) ligando o **pad central** do chip ao **pad do conector IPEX** (desconectando a antena PCB).
- **Casamento `SMA` vs `RP-SMA`:** Desrosqueie a antena do cabo *pigtail* e olhe dentro: **um lado DEVE ter pino central macho e o outro DEVE ter furinho fêmea**! **JAMAIS energize o ESP32-S3 transmitindo sem a antena rosqueada no SMA!**

#### 2. Conferência do LDO `3,3 V` na Placa `ESP32-S3`
- Confirme que o regulador de tensão perto das portas USB-C é o encapsulamento grande **`SOT-223` (`AMS1117-3.3`, `800 mA`)**.

#### 3. Teste de Polaridade Lógica do Buzzer Ativo (`NPN Active-HIGH`)
- Alimente provisoriamente o módulo buzzer (`GND → -`, `VCC → +5 V`):
  - Encoste o pino `I/O (S)` no `GND`: **DEVE FICAR 100% MUDO**.
  - Encoste o pino `I/O (S)` no `+3,3 V`: **DEVE APITAR FORTE**.
  - *(Se apitar quando `I/O = GND`, você recebeu um módulo PNP `Active-LOW`: não o solde direto! Dessolde o cápsula cilíndrica do buzzer da plaquinha e acione-a com um transistor `NPN BC547/2N2222` + resistor `1 kΩ` na base, ou compre o módulo `KY-012` correto).*

#### 4. Serigrafia do OLED `0,96"` (`SSD1306`) e Pré-Ajuste do `MT3608`
- Anote no papel a ordem exata impressa na frente do seu OLED (`GND - VCC - SCL - SDA` **ou** `VCC - GND - SCL - SDA`).
- Se for usar célula `18650` + módulo *step-up* `MT3608` no Controle: ligue a `18650` no `MT3608` **desconectado de tudo** e gire o trimpot anti-horário (~15 voltas) medindo a saída até cravar **`5,00 V`** exatos!

---

## Etapa 1: Construção Mecânica do Chassi Híbrido 450 mm (`v2.1`)

### 1.1 Gabarito Oficial das 2 Chapas Centrais (`120 × 120 mm`, Compensado Naval `4 mm`)
Para obter precisão de décimos de milímetro sem CNC, **furamos as duas chapas empilhadas juntas**:

1. Corte **2 quadrados exatos de `120 mm × 120 mm`** no compensado `4 mm`, alinhe as bordas e prenda-as firmemente juntas com fita crepe nas 4 laterais.
2. Risque as **2 diagonais de canto a canto** formando um `"X"` a `90°`. O cruzamento exato é a origem `(0, 0)`.
3. Escolha um lado reto entre dois cantos para ser a **`FRENTE ↑`** do drone (marque **`FRENTE ↑`** com caneta permanente nas duas chapas antes de furar!). Defina os eixos cartesianos a partir de `(0, 0)`:
   - **`X` = Largura Esquerda–Direita** (`-X` Esquerda, `+X` Direita).
   - **`Y` = Comprimento Traseira–Frente** (`+Y` Frente, `-Y` Traseira).
4. Sobre cada uma das **4 meias-diagonais** (rumo aos cantos `M1 FR`, `M2 RR`, `M3 RL`, `M4 FL`), marque com punção/prego:
   - **Furo Interno do Braço:** a exatos **`32,0 mm`** de `(0, 0)`.
   - **Furo Externo do Braço:** a exatos **`68,0 mm`** de `(0, 0)`.
5. **Gabarito v2.1 Anti-Colisão dos 4 Dampers `M3` da Controladora (`44 mm` Largura × `82 mm` Comprimento):**
   - Marque os 4 furos retangulares em **`X = ±22,0 mm`** (Esquerda/Direita) e **`Y = ±41,0 mm`** (Frente/Trás):
     - `(-22, +41)`, `(+22, +41)`, `(-22, -41)` e `(+22, -41)` mm.
   - *Por que essa medida exata é crítica:* a distância perpendicular até os braços diagonais é $d_\perp = \frac{41 - 22}{\sqrt{2}} = \mathbf{13,44\text{ mm}}$, garantindo folga livre real para as porcas/dampers `M3` sem jamais encostar nos braços de `15×15 mm` ou `12,7 mm`, e caindo a `4,0 mm` da borda superior/inferior e `13,0 mm` da borda lateral da placa `70×90 mm`!
6. Na **Chapa Superior apenas** (depois de furar todos os furos `M3` com **broca `3,0 mm` ou `3,2 mm`**), você pode abrir um furo central redondo de `Ø 16 mm` em `(0, 0)` para subir os fios de `5V/GND/VBAT` da PDB, mantendo intactos os furos de `32 mm`!
7. Chanfre `10 mm` nos 4 cantos externos e lixe com grão `220`.

```text
          VISTA SUPERIOR DA CHAPA CENTRAL v2.1 (120 x 120 mm)
                       [ ↑ FRENTE DO DRONE (+Y) ↑ ]
       Canto FL (M4 CW)                         Canto FR (M1 CCW)
            +---------------------------------------+
            |\  (68mm)                     (68mm)  /|
            | \   O     o(-22,+41) (+22,+41)o O   / | <-- Dampers M3 c/ folga real dos braços!
            |  \        +-----[70x90 FC]----+    /  |
            |   \       |                   |   /   |
            |    \  O   |                   | O/    | <-- Furos internos dos braços (r = 32 mm)
   ESQ (-X) |     \32mm |       (0,0)       | /32mm | DIR (+X)
            |           |         +         |       |
            |     /32mm |                   | \32mm |
            |    /  O   |                   | O\    |
            |   /       +-------------------+   \   |
            |  /        o(-22,-41) (+22,-41)o    \  |
            | /   O                           O   \ |
            |/  (68mm)                     (68mm)  \|
            +---------------------------------------+
       Canto RL (M3 CCW)                        Canto RR (M2 CW)
```

---

### 1.2 Gabarito Oficial dos 4 Braços (`225 mm` — Madeira `15×15 mm` ou Alumínio `1/2"`)
Corte **4 barras idênticas de `225,0 mm`** (com esquadro nas pontas):
- **Se usar Madeira (Fase 1):** Use obrigatoriamente seção mínima **`15 × 15 mm`** ($f_{n1} = 101,3\text{ Hz}$, evitando a ressonância de `76 Hz` de perfis finos de `13 mm`).
- **Se usar Tubo de Alumínio `1/2" × 1/2"` (`12,7 × 12,7 mm`, parede `1,0–1,2 mm`):** Antes de furar, lixe **4 buchas maciças de madeira dura de `55 mm` de comprimento** e insira sob pressão na extremidade interna (`0..55 mm`) de cada tubo (e opcionalmente um toquinho de `28 mm` na ponta do motor `192..220 mm`). Isso impede 100% que o aperto dos parafusos `M3` amasse a parede fina do alumínio!

```text
   GABARITO DE CADA BRAÇO (225 mm de comprimento — linha de centro da face superior)

   |<-------------------- 225,0 mm -------------------->|
   +----------------------------------------------------+
   |    O          O                         O  (Ø8)  O |
   +----------------------------------------------------+
   0  12,0mm     48,0mm                    197  205  213mm  225mm
   ^    ^          ^                        ^    ^    ^
   |    |          |                        |    |    +-- Furo Motor Ext M3 (213,0mm p/ base 16mm | 214,5mm p/ 19mm)
   |    |          |                        |    +------- Alívio trava eixo (Ø 8,0 mm, prof. 4 mm) = Raio 225 mm do drone!
   |    |          |                        +------------ Furo Motor Int M3 (197,0mm p/ base 16mm | 195,5mm p/ 19mm)
   |    |          +------------------------------------- Furo Chassi Externo M3 (casa c/ r = 68 mm da chapa)
   |    +------------------------------------------------ Furo Chassi Interno M3 (casa c/ r = 32 mm da chapa)
   +----------------------------------------------------- Ponta interna (fica a r = 20 mm do centro (0,0))
```

- **Endurecimento dos Furos na Madeira (Obrigatório):** Pingue **2 gotas de cianoacrilato líquido (*Super Bonder*)** na parede interna de todos os furos `M3` da madeira (sem parafuso!) e aguarde **15 minutos** curar antes da montagem.

### 1.3 Fixação Direta dos 4 Motores `A2212 1000KV` (Sem Cruzeta!)
> [!CAUTION]
> **REGRA DE SOBREVIVÊNCIA DO ENROLAMENTO:** A ponta rosqueada do parafuso `M3` que sobe por baixo do braço e entra na base metálica do `A2212` deve penetrar **entre `2,8 mm` e `3,5 mm` (MÁXIMO ABSOLUTO!)**. Se passar de `4 mm`, ela perfura o verniz das bobinas de cobre e queima motor + ESC na hora!
> - **Para braço de Madeira `15×15 mm`:** Parafuso `M3 × 18 mm` + `1 arruela lisa M3 (0,8 mm)` → penetração = $18,0 - 15,0 - 0,8 = \mathbf{2,2\text{ a }2,8\text{ mm}}$ (ou `M3 × 20 mm` + **3 arruelas M3** = `2,6 mm`).
> - **Para braço de Alumínio `12,7 mm`:** Parafuso `M3 × 16 mm` + `1 arruela M3` → penetração = $16,0 - 12,7 - 0,8 = \mathbf{2,5\text{ mm}}$.
> - **Inspeção visual e elétrica obrigatória:** Ilumine por baixo das janelas de ventilação do motor confirmando folga até o cobre, e valide no Portão 1 com o multímetro!

### 1.4 Fechamento do Sanduíche Central e Pés de Pouso
1. Monte `Chapa Inferior` → `4 Braços` → `Chapa Superior` com **8× parafusos `M3 × 30 mm`**, arruelas largas em cima e embaixo e **porcas autotravantes `M3 Parlock`** na face inferior. Aperte em estrela sobre vidro/porcelanato plano.
2. Instale **4 pés de espuma isolante tubular (`6 cm`)** sob a ponta de cada braço com 2 abraçadeiras de nylon cada, e passe a **cinta de Velcro + EVA antiderrapante** na barriga da `Chapa Inferior` centrada em `(0, 0)`.

> ✅ **PORTÃO DE VALIDAÇÃO 1 (Mecânica & Isolação de Estator):**
> | Medição | Instrumento | Critério de Aprovação |
> | :--- | :--- | :--- |
> | Nivelamento dos 4 eixos na mesa plana | Régua vertical | Diferença entre os 4 motores **`< 1,5 mm`** |
> | Distâncias laterais `M1–M2`, `M2–M3`, `M3–M4`, `M4–M1` | Trena | Todas em **`318 mm ± 2,5 mm`** (diagonal `450 mm ± 3 mm`) |
> | Isolação Carcaça do Motor ↔ cada um dos 3 fios banana | Multímetro (`Ω` / Bip) | **`OL` (Circuito Aberto Infinito)** nos 12 testes (`4 motores × 3 fases`)! |

---

## Etapa 2: Subsistema de Potência (`Mini PDB-XT60` + `4× ESC 30A`)

1. **Na Bancada:** Solde o rabicho **`14 AWG` (`8 cm`) + plugue `XT60 Macho`** e o **Capacitor `1000 µF 25V Low-ESR`** (faixa branca `-` no `GND`!) diretamente nos pads principais `BAT+ / BAT-` da `Mini PDB-XT60`.
2. **Nos Braços:** Prenda cada `ESC 30A` com abraçadeiras na lateral superior do braço e solde seus cabos grossos `Vermelho (+)` e `Preto (-)` nos 4 pares da PDB.
3. **Chicote p/ Controladora:** Solde `2× fios 22 AWG` nos pads regulados **`5V`** e **`GND`** do BEC da PDB, e `1× fio 26 AWG` num pad **`VCC / BAT+`** (leitura de bateria `3S`).
4. **REMOÇÃO OBRIGATÓRIA DOS FIOS VERMELHOS (`+5V`) DOS 4 ESCs:**
   - Levante com estilete fino a lingueta plástica do pino central **VERMELHO** no conector servo de 3 vias de **todos os 4 ESCs**, puxe o terminal metálico para trás e isole cada um individualmente com tubo termorretrátil! Somente `Sinal (Branco/Amarelo)` e `GND (Preto/Marrom)` vão para a placa controladora!
5. Monte a PDB sobre **4 espaçadores de nylon `M3 × 6 mm`** no centro da `Chapa Superior` (abaixo do vão onde ficará a placa `70×90 mm`).

> ✅ **PORTÃO DE VALIDAÇÃO 2 (Potência Isolada — SEM a Placa do ESP32-S3!):**
> 1. Multímetro em `Ω` no plugue `XT60`: resistência sobe carregando o capacitor até `> 10 kΩ / OL` (**nunca `0 Ω`!**).
> 2. Plugue a LiPo `3S` na PDB (ouça os 3 bipes musicais `♪ 1-2-3` dos motores) e meça a saída do BEC `5V`: deve marcar **`4,95 V a 5,10 V DC`**. Desconecte a LiPo!

---

## Etapa 3: Placa Controladora de Voo (`70 × 90 mm` FR4 — Revisão `v2.1`)

### 3.1 Disposição Física na Placa `70 × 90 mm`
1. **Furos dos 4 Dampers `M3`:** Marque e fure com broca `3,2 mm` nos mesmos **`X = ±22,0 mm, Y = ±41,0 mm`** a partir do centro da placa `70 × 90 mm` (ficam a `13,0 mm` das laterais e `4,0 mm` do topo/base!).
2. **IMU `ICM-42688-P` SPI no Centro `(0, 0)`:** Soquete fêmea bem no centro geométrico da placa (anote para qual lado aponta a seta `+X` da serigrafia da IMU — se ficar girada em `90°`, `180°` ou `270°`, você ajusta em 1 segundo via comando `imu_orient` sem refazer solda!).
3. **`ESP32-S3-DevKitC-1`:** Em **2 barras de pinos fêmea de 22 vias** na metade esquerda, com as **portas USB-C voltadas para a Traseira** (fácil acesso ao cabo USB).
4. **Soquete `DIP-14` (`SN74HCT125N`):** Na metade direita, ao lado das 4 barras de `2 pinos machos` (`Sinal + GND`) dos ESCs `M1..M4`.

```text
             LAYOUT VISTA DE TOPO — PLACA PERFURADA 70 x 90 mm (v2.1)
                             [ ↑ FRENTE DO DRONE (+Y) ↑ ]
   +-----------------------------------------------------------------------+
   |       (O) Damper (-22,+41)                 Damper (+22,+41) (O)       |
   |                                                                       |
   |   [Header 6p GPS/Mag Fase 2]          [BMP280 I2C c/ espuma porosa!]  |
   |   (+5V_SYS, GND, TX17, RX18, SDA8, SCL9)                              |
   |                                                                       |
   |   +--------------------+              +---------------------------+   |
   |   |                    |              |  [M4 FL] [M1 FR] (Sinal + |   |
   |   |                    |  +--------+  |  [M3 RL] [M2 RR]   GND)   |   |
   |   |   ESP32-S3-WROOM   |  |  ICM   |  |                           |   |
   |   |    DevKitC-1       |  |42688-P |  |   +-------------------+   |   |
   |   |  (2x Header 22p)   |  | (0,0)! |  |   | Soquete DIP-14    |   |   |
   |   |                    |  +--------+  |   |   SN74HCT125N     |   |   |
   |   |   [IPEX -> SMA]    |              |   +-------------------+   |   |
   |   |                    |              |                           |   |
   |   |                    |              |  [1N5819] [Buzzer KY-012] |   |
   |   |   [USB]   [UART]   |              |  [Divisor VBAT 39k / 10k] |   |
   |   +--------------------+              +---------------------------+   |
   |                                                                       |
   |       (O) Damper (-22,-41)                 Damper (+22,-41) (O)       |
   +-----------------------------------------------------------------------+
                             [ ↓ TRASEIRA DO DRONE (-Y) ↓ ]
```

---

### 3.2 Lista Mestre de Ligações Ponto-a-Ponto (`drone_fc` v2.1)

#### A) Alimentação Protegida e Divisor Linear `VBAT` (`39 kΩ / 10 kΩ`)
1. **Malha Estrela `GND`:** Una com fio rígido estanhado `GND da PDB` ↔ `Pinos GND do ESP32-S3` ↔ `Pino 7 do SN74HCT125N` ↔ `Pinos GND dos 4 ESCs` ↔ `GND da IMU, BMP280, Buzzer e Header GPS`.
2. **Barramento `+5V_SYS`:**
   - `Fio +5V BEC da PDB` → **Ânodo** do diodo Schottky **`1N5819`** → **Cátodo (faixa prateada)** forma a linha **`+5V_SYS`** (`~4,75 V`).
   - Ligue `+5V_SYS` em: pino **`5V (IN)`** do ESP32-S3, **`Pino 14 (VCC)`** do `SN74HCT125N` (+ capacitor cerâmico **`100 nF`** soldado direto entre Pino 14 e Pino 7!), **`VCC`** do Buzzer `KY-012` e pino `1` do Header GPS.
3. **Barramento Limpo `+3V3` (Sensores):**
   - Do pino **`3V3`** do ESP32-S3 → solde **`10 µF` + `100 nF`** p/ `GND` colados ao soquete da IMU → alimente **`VCC / 3V3`** do **`ICM-42688-P`** e **`VCC`** do **`BMP280`** (**NUNCA ligue 5 V nesses sensores!**).
4. **Divisor `VBAT` v2.1 (`39 kΩ / 10 kΩ` → `GPIO1`, Fator `4,90×`):**
   - `Fio BAT+ (12,6 V máx)` → **Resistor `39 kΩ 1%`** → nó central no **`GPIO1`** → do `GPIO1` p/ `GND`: **Resistor `10 kΩ 1%`** em paralelo com **Capacitor `100 nF`**.
   - Em `12,60 V` (3S 100% carregada): $V_{\text{GPIO1}} = 12,60 / 4,90 = \mathbf{2,57\text{ V}}$ (**folga ampla abaixo do teto de saturação de `2,90 V` do `ADC1` do ESP32-S3!**).

#### B) Buffer `SN74HCT125N` (`DIP-14`, meia-lua `U` p/ Frente) → 4 ESCs (`400 Hz`)
| Canal | Posição & Sentido | Pino ESP32-S3 | Pull-Down | Entrada `74HCT125N` | Enable (`OE#`) | Saída `5V` `74HCT125N` | Conector ESC |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **M1** | Frente-Direita (**FR, CCW ↺**) | **`GPIO4`** | `10 kΩ → GND` | **Pino 2 (`1A`)** | **Pino 1 → `GND`** | **Pino 3 (`1Y`)** | Sinal ESC 1 + `GND` |
| **M2** | Traseira-Direita (**RR, CW ↻**) | **`GPIO5`** | `10 kΩ → GND` | **Pino 5 (`2A`)** | **Pino 4 → `GND`** | **Pino 6 (`2Y`)** | Sinal ESC 2 + `GND` |
| **M3** | Traseira-Esquerda (**RL, CCW ↺**) | **`GPIO6`** | `10 kΩ → GND` | **Pino 9 (`3A`)** | **Pino 10 → `GND`** | **Pino 8 (`3Y`)** | Sinal ESC 3 + `GND` |
| **M4** | Frente-Esquerda (**FL, CW ↻**) | **`GPIO7`** | `10 kΩ → GND` | **Pino 12 (`4A`)** | **Pino 13 → `GND`** | **Pino 11 (`4Y`)** | Sinal ESC 4 + `GND` |

#### C) IMU `ICM-42688-P` SPI (`10 MHz`), `BMP280` I2C (`400 kHz`), Buzzer e LED RGB
- **`ICM-42688-P` (fios curtos `< 4 cm`!):**
  - `VCC/VDDIO` → `3V3` | `GND` → `GND` | `CS` → **`GPIO10`** | `MOSI (SDA/SDI)` → **`GPIO11`** | `SCK (SCL/SCLK)` → **`GPIO12`** | `MISO (SDO/AD0)` → **`GPIO13`** | `INT1` → **`GPIO14`** *(se houver pino `FSYNC` ou `INT2`, aterre `FSYNC → GND` e deixe `INT2` livre)*.
- **`BMP280` I2C:**
  - `VCC` → `3V3` | `GND` → `GND` | `SDA` → **`GPIO8`** | `SCL` → **`GPIO9`** | `SDO` → `GND` (`0x76`) | `CSB` → aberto/`3V3`.
  - Cubra o chip metálico com um quadradinho de **espuma preta de célula aberta** (presa apenas pelas bordas, **jamais obstruindo o furinho de ventilação com cola!**) para bloquear luz solar direta e o turbilhão das hélices.
- **Buzzer `KY-012` (`NPN Active-HIGH`):** `VCC` → `+5V_SYS` | `GND` → `GND` | `S (I/O)` → **`GPIO21`**.
- **Header 6 Pinos Fase 2 (Reservado):** `1:+5V_SYS | 2:GND | 3:GPIO17(TX) | 4:GPIO18(RX) | 5:GPIO8(SDA) | 6:GPIO9(SCL)`.
- **Código Oficial de Cores do LED RGB Onboard (`GPIO48` na v1.0/YD e `GPIO38` na v1.1 — acionados automaticamente):**
  - **Amarelo sólido:** Calibrando giroscópio no boot (`BOOT_CALIBRATING` — não toque no drone!).
  - **Azul piscando:** Desarmado, aguardando item de segurança (`PreArmFlags != 0`).
  - **Verde sólido:** Desarmado e **100% PRONTO PARA ARMAR (`PREARM_OK`)**.
  - **Ciano sólido:** Modo diagnóstico de bancada ativo (`BENCH_DIAGNOSTIC`).
  - **Vermelho forte sólido:** **PERIGO — MOTORES ARMADOS (`ARMED_IDLE` / `FLYING`)!**
  - **Roxo piscando + Sirene:** **FAILSAFE ATIVO (`FAILSAFE_DESCEND` / `FAILSAFE_LOCKED`)!**

> ✅ **PORTÃO DE VALIDAÇÃO 3 (Placa `70×90 mm` — COM TODOS OS MÓDULOS AINDA FORA DOS SOQUETES!):**
> 1. Continuidade sem energia: `+5V_SYS ↔ GND`, `3V3 ↔ GND` e `+5V_SYS ↔ 3V3` dão **Circuito Aberto (`OL`)**?
> 2. Plugue a LiPo `3S` na PDB (soquetes ainda vazios!) e meça:
>    - Pino `5V` do soquete ESP32-S3 e Pino `14` do DIP-14: **`4,68 V a 4,85 V`**?
>    - Pino **`GPIO1`** do soquete ESP32-S3: **`2,30 V a 2,60 V`** (exatamente `VBAT / 4,90`, sempre `< 2,90 V`)?
>    - Pinos `VCC` dos soquetes da IMU e do BMP280: **`0,00 V`** (confirmando que não estão na linha de `5 V`)?
> 3. Desconecte a LiPo, espete `ESP32-S3` (USB p/ trás), `SN74HCT125N` (meia-lua `U` p/ frente), `ICM-42688-P`, `BMP280` e **rosqueie a antena 2,4 GHz**! Instale sobre os 4 dampers `M3` (`±22, ±41 mm`) e fixe a antena vertical a `90°` longe da LiPo.

---

## Etapa 4: Construção do Controle Caseiro Long-Range (`ground_tx` v2.1)

```text
       PAINEL FRONTAL DO CONTROLE CASEIRO (Caixa Patola ~15 x 10 x 5 cm)
    +-------------------------------------------------------------------+
    |  [SW1: ARM / KILL]        [Antena SMA ↑]       [SW2: MODO 3-POS]  |
    |   (Cima=SAFE/OFF!)                             (STAB / ALTH / CINE)|
    |                        +------------------+                       |
    |      (Gimbal Esq.)     |  OLED 0.96" I2C  |      (Gimbal Dir.)    |
    |         ( | )          | DISARMED STAB -58|         ( + )        |
    |       Throttle ↑↓      | 3S:12.4V TX:4.1V |        Pitch ↑↓       |
    |       (sem mola!)      | OK P/ ARMAR      |        Roll  ←→       |
    |         Yaw ←→         +------------------+       (com mola)      |
    |                             [Botão AUX]                           |
    +-------------------------------------------------------------------+
                         [Porta USB-C Nativa ↓ OTG p/ QGroundControl]
```

### 4.1 Circuito Anti-Saturação `ADC1` dos 4 Potenciômetros (`Resistor Topo 560 Ω` — v2.1!)
Para que os potenciômetros nunca passem de `2,90 V` (onde o `ADC1` do ESP32-S3 satura perdendo `12%` de curso nos cantos):
1. Do pino **`3V3`** do ESP32-S3, solde **1 resistor de `560 Ω` em série** formando o barramento **`+V_POTS`**, e coloque **1 capacitor eletrolítico de `10 µF` entre `+V_POTS` e `GND`**.
2. Ligue os terminais positivos de todos os 4 potenciômetros em **`+V_POTS`** e os negativos em **`GND`**.
3. Ligue cada cursor central (*wiper*) ao seu pino `ADC1` com **1 capacitor cerâmico `100 nF` para `GND`** junto ao ESP32-S3:
   - **Throttle** (Esq. Vertical s/ mola) → **`GPIO1`**
   - **Yaw** (Esq. Horizontal c/ mola) → **`GPIO2`**
   - **Pitch** (Dir. Vertical c/ mola) → **`GPIO4`**
   - **Roll** (Dir. Horizontal c/ mola) → **`GPIO5`**

### 4.2 Chaves Fail-Safe, OLED, Bateria `18650` e Funções do Botão `[AUX]`
- **`SW1` (Kill-Switch 2 pos.):** Entre **`GPIO10`** e `GND` (`INPUT_PULLUP`: chave aberta = **`SAFE / DISARMED`**; fechada p/ `GND` = `ARM`).
- **`SW2` (Modos 3 pos. `ON-OFF-ON`):** Pino central no `GND`, lateral A no **`GPIO11`** (`STABILIZE`), meio aberto (`ALT_HOLD`), lateral B no **`GPIO12`** (`ALT_CINE` suave `±20°`).
- **Botão `[AUX]` (`GPIO13 → GND`):** Possui **dupla função automática no firmware v2.1**:
  1. **Segurar `[AUX]` ao ligar o Controle (com `SW1` em `SAFE`):** Abre na tela OLED o **Assistente Interativo de Calibração dos Gimbals** (Passo 1: mover os 2 sticks em todos os cantos e apertar `[AUX]`; Passo 2: soltar no centro c/ Throttle no zero e apertar `[AUX]` p/ gravar na `NVS`!).
  2. **Segurar `[AUX]` por `1,5 segundo` em operação normal (com drone desarmado):** Envia comando confirmado ao drone para **gravar permanentemente na Flash `NVS` os ganhos PID ajustados no QGroundControl**, exibindo `"SALVANDO NVS DRONE!"` no OLED + bipe de confirmação!
- **Buzzer `KY-012`:** `S → GPIO14`, `VCC → +5V_SYS / 3V3`, `GND → GND`.
- **OLED `0,96"`:** `SDA → GPIO8`, `SCL → GPIO9`, `VCC → 3V3`, `GND → GND` *(conferindo serigrafia!)*.
- **Bateria `18650` (se interna):** Positivo da `18650` (`4,2 V` antes do Boost) num divisor **`100 kΩ + 100 kΩ` (+ `100 nF`)** ligado ao **`GPIO6`**, e saída `5,00 V` do `MT3608` passando por **1 diodo `1N5819`** até o pino `5V (IN)` do ESP32-S3 (proteção total ao plugar o cabo OTG na porta **`USB` nativa**!).

> ✅ **PORTÃO DE VALIDAÇÃO 4 (Controle Caseiro + Calibração de Gimbals):**
> 1. Grave `pio run -e ground_tx -t upload` (com antena conectada!).
> 2. Ligue segurando `[AUX]` (chave `SW1` em `SAFE`), conclua os 2 passos do Wizard na tela OLED e verifique:
>    - Com Throttle todo abaixado, a barra mostra **`THR:  0%`**; no topo, **`THR:100%`** (sem travar antes de encostar no batente plástico!).
>    - Meça a tensão em `+V_POTS`: deve estar entre **`2,20 V e 2,75 V`** (100% dentro da faixa linear `< 2,90 V` do `ADC1`!).

---

## Etapa 5: Construção da Bancada "Gangorra 1-Eixo" (Zero Risco no Tuning!)

> [!IMPORTANT]
> **Jamais faça o primeiro teste com hélices soltando o drone direto do chão!** Na **Gangorra 1-Eixo**, você valida sinais, testa resposta a rajadas empurrando o braço com uma vareta e afina o PID com segurança absoluta.

```text
                  BANCADA GANGORRA 1-EIXO (Feita c/ retalhos de madeira)

                     (M4)=========[ DRONE ]=========(M1)
                                     ||  <-- Preso c/ abraçadeiras na ripa giratória
                           ==========()==========  <-- Haste/parafuso M5 como eixo livre s/ atrito!
                                    /  \
                                   /    \   <-- 2 Colunas de 25 cm (hélices longe da base!)
                        ========================  <-- Base pesada 40x40 cm presa c/ sargento/peso
```

- Duas colunas verticais de `25 cm` afastadas `36 cm` sustentam um eixo livre onde o centro `(0, 0)` do drone gira em apenas 1 eixo por vez (primeiro **Roll**, depois girado `90°` para **Pitch**), com cordão limitador de segurança em `±35°`.

---

## Etapa 6: Os 4 Portões Finais de Comissionamento de Software (Portões 5 a 8)

Grave o firmware do drone (`pio run -e drone_fc -t upload`) e abra o monitor serial na porta **`USB` nativa** (`115200 bps`):

### ✅ PORTÃO 5 — Saúde de Sensores e Convenção Estrita `FRD` (Só USB, Sem LiPo e Sem Hélices!)
1. Digite `status` no terminal:
   - `IMU SPI: ICM-42688-P (WHO_AM_I=0x47)` | `INT1=995..1005 Hz` | `Healthy=1` | `dt=1000 us` | `Baro BMP280: OK`.
2. Digite `imu_test` e mova o drone na mão conferindo os **3 sinais obrigatórios `FRD`**:
   - Inclinar asa **DIREITA para baixo** → **`Roll` POSITIVO (`+`)**.
   - Inclinar **NARIZ para cima** → **`Pitch` POSITIVO (`+`)**.
   - Girar **NARIZ no sentido HORÁRIO** (visto de cima) → **`GyroZ / Yaw` POSITIVO (`+`)**.
   - *(Se os eixos horizontais estiverem trocados pela posição na placa, execute `imu_orient 90`, `180` ou `270` e repita `imu_test` até os 3 sinais baterem 100%!).*

### ✅ PORTÃO 6 — Calibração de ESCs, Ordem e Sentido dos 4 Motores (**HÉLICES GUARDADAS LONGE!**)
1. Com hélices **removidas**, cole uma bandeirinha de fita crepe em cada eixo.
2. *(Na 1ª vez apenas):* Digite `bench on` → `esc_cal` → `CONFIRM`, conecte a LiPo `3S`, ouça os 2 bipes curtos `bi-bi` de topo (`2000 µs`) e pressione `[ENTER]` imediatamente para gravar o mínimo (`1000 µs`).
3. Teste cada motor individualmente por `3 s` (`bench on`):
   - `motor_test 1 8` → **Frente-Direita (`M1`)** gira **ANTI-HORÁRIO (`CCW ↺`)**?
   - `motor_test 2 8` → **Traseira-Direita (`M2`)** gira **HORÁRIO (`CW ↻`)**?
   - `motor_test 3 8` → **Traseira-Esquerda (`M3`)** gira **ANTI-HORÁRIO (`CCW ↺`)**?
   - `motor_test 4 8` → **Frente-Esquerda (`M4`)** gira **HORÁRIO (`CW ↻`)**?
   - *(Se algum motor girar invertido, desconecte a LiPo e troque 2 dos 3 conectores banana `3,5 mm` daquele motor).* Digite `bench off`.

### ✅ PORTÃO 7 — Vibração Dinâmica (FFT), Kill-Switch e Failsafe (**AINDA SEM HÉLICES!**)
1. Ligue o Controle Caseiro (confirme vínculo no OLED e LED **Verde sólido** no drone).
2. Arme pela chave `SW1` (LED muda p/ **Vermelho forte**), suba o acelerador a `40%` por `5 s`:
   - Vire **`SW1` p/ `SAFE`**: os 4 motores devem cortar instantaneamente (`< 15 ms`)!
   - Rearme a `25%` de acelerador e **desligue a chave geral de energia do Controle Caseiro**: em **`300 ms`** o drone deve entrar em **`FAILSAFE_DESCEND`** (LED **Roxo piscando** + sirene no buzzer)! Religando o controle com `SW1` em `SAFE`, tudo desarma limpo.
3. No terminal USB, rode `bb_dump_bin > teste_bancada.bin` e no PC execute `python3 tools/analyze_blackbox.py teste_bancada.bin` confirmando ruído filtrado RMS `< 3 deg/s` e `dt_us = 1000 µs`.

### ✅ PORTÃO 8 — Sintonia na Gangorra 1-Eixo e Primeiro Voo Livre (`STAB` → `ALTH` → `CINE`)
1. Prenda o drone na **Gangorra 1-Eixo**, coloque óculos de proteção e instale as hélices **`1045 Nylon-Carbono`** (**letras `10x4.5` impressas na pá SEMPRE voltadas para CIMA em todos os 4 motores!** Hélices `1045R / Pusher` nos motores `CW` `M2/M4`, e `1045` normais nos motores `CCW` `M1/M3`).
2. Em modo **`STAB`**, suba o acelerador até `~32%` (peso aliviado na gangorra):
   - O drone deve nivelar firme na horizontal e resistir quando você empurra um braço para baixo com uma vareta longa!
   - Se desejar refinar pelo celular/tablet conectado via cabo OTG no **QGroundControl**, ajuste `RATE_RP_KP` / `RATE_RP_KD` ao vivo e, após desarmar, segure **`[AUX]` por `1,5 s`** para gravar na `NVS`!
3. **Primeiro Voo Livre (Gramado aberto, sem vento forte, > 15 m de distância de pessoas):**
   - Decole decidido até `1,5 m` de altura em **`STAB`** (saindo rápido do colchão de ar rente ao chão `< 0,5 m`), valide *hover* estável e mude `SW2` para **`ALTH`** (centro) e **`CINE`** para experimentar a trava suave de altitude!
