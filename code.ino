#include <Wire.h>              // Biblioteca para enderecamento I2C
#include <LiquidCrystal_I2C.h> // Biblioteca para controle do display LCD
#include <U8g2lib.h>           // Biblioteca para controlar display OLED com baixo uso de memoria

// Resolucao dos displays
#define LARGURA_OLED 128
#define ALTURA_OLED 64

// Endereco I2C dos displays
#define ENDERECO_LCD 0x27
#define ENDERECO_OLED 0x3C

// Definicao da grade no modo de onda do oscilloscopio
#define GRADE_X 15    // Tamanho horizontal da grade
#define GRADE_Y 15    // Tamanho vertical da grade
#define GRADE_PASSO 5 // Passo de transparencia da grade

// Definicao da barra de espacamento no modo de onda
#define Y_INICIO_GRAFICO 12

// Inicializacao das bibliotecas dos displays
LiquidCrystal_I2C lcd(ENDERECO_LCD, 16, 2);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// Constantes de hardware
const int pinoLeitura = A0;   // Pino analogico para leitura de voltagem
const int pinoBuzzer = 12;    // Pino de saida para emissao de feedback sonoro
const int pinoDisplay = 8;    // Pino de entrada PULLUP para modos de display
const int pinoMedicao = 7;    // Pino de entrada PULLUP para modos de medicao
const int pinoFonte10k = 2;   // D2 -> resistor 10k -> no P (continuidade e resistencia)
const int pinoFonte3x10k = 3; // D3 -> 3x resistor 10k  -> no P (diodo)
const int bitsADC = 10;       // Resolucao ADC/DAC em bits (10 = 1023 etc.)

const float R_REF = 10000.0f; // Resistencia de referencia
const float VREF = 5.0f;      // Voltagem de referencia para medicao via ADC

// Variaveis para controle de estados
int modoDisplay = 0;    // 0: Padrao, 1: Osciloscopio, 2: Metricas
int modoMedicao = 0;    // 0: CONT, 1: DIODO, 2: BAT, 3: RES
int modoMedicaoAlt = 0; // Sincroniza o modo de medicao com o display

bool botaoDisplayAnterior = HIGH; // Estado anterior do botao para debounce
bool botaoMedicaoAnterior = HIGH; // Estado anterior do botao para debounce

// Variaveis de controle para testes internos com onda simulada
bool simulacaoOndaAtiva = true; // Alterne para false para ler o pino A0

int simulacaoTipoOnda = 0; // 0: Senoidal, 1: Quadrada, 2: Dente de serra, 3: Triangular

float simulacaoAmplitude = 2.5f;      // Amplitude da onda em volts, inicia no centro entre 0 e VREF
float simulacaoOffset = 0.0f;         // Deslocamento vertical da onda a partir do centro
float simulacaoCicloTrabalho = 50.0f; // Ciclo de trabalho para onda quadrada (PWM), mantenha entre 0.0f e 100.0f
float simulacaoFaseGraus = 0.0f;      // Deslocamento de fase da onda em graus
float simulacaoFreqHz = 1.0f;         // Frequencia da onda simulada (Hz)

// Acumuladores de tempo
unsigned long ultimoCliqueBotao = 0; // Acumula tempo entre apertos dos botoes

unsigned long tempoAcumuladoSimUs = 0; // Utilizados para desenhar as ondas de teste
unsigned long ultimoTempoSimUs = 0;    // Utilizados para desenhar as ondas de teste

unsigned long ultimoSerialMs = 0; // Evita spam nos logs

unsigned long tempoAltoUs = 0;   // Tempo em que o sinal permaneceu em HIGH
unsigned long ultimaBordaUs = 0; // Tempo entre ultima mudanca LOW -> HIGH

unsigned long ultimoDisplayMs = 0; // Usado para atualizar os displays sem bloqueio
unsigned long ultimaAmostraUs = 0; // Evita avancar a coordenada X do oscilloscopio indevidamente

// Memoria interna para o serialMonitor
char textoMedicao[16]; // O texto de log do serialMonitor e copiado para esta variavel
const char *nomesModo[4] = {"CONT", "DIODO", "BAT", "RES"};

// Variaveis de configuracao do oscilloscopio
float escalaTempoTela = 3.0f; // Segundos por largura total da tela (1.0 = 1 ciclo de 1Hz)

int eixoXGrafico = 0;                // Cursor horizontal para desenhar a linha do oscilloscopio
int eixoYAnterior = ALTURA_OLED - 1; // Usado para desenhar uma lina continua a partir deste ponto

bool lcdAtualizado = false; // Evita atualizar o display I2C a cada loop no modo de oscilloscopio

// Variaveis para analise de metricas
float vMin = VREF;          // Voltagem minima medida
float vMax = 0.0f;          // Voltagem maxima medida
float frequencia = 0.0f;    // Frequencia da onda em hertz
float tempoCiclo = 0.0f;    // Tempo entre uma oscilacao completa em millisegundos
float cicloTrabalho = 0.0f; // Ciclo de trabalho da onda, formato de porcentagem

bool bordaAnterior = false; // Ultimo estado do sinal medido, sendo HIGH ou LOW

byte passoAntiHarmonico = 0; // Altera a taxa de refrescamento da analise de onda para nao entrar em harmonia com a onda

// Variaveis pre-processadas, nao altere
long adcMax = (1L << bitsADC) - 1; // Resolucao do ADC, usa operador "left bit shift" (1L << 10 => 1024 - 1 => 1023)

unsigned long passoOscUs = (escalaTempoTela * 1000000.0f) / LARGURA_OLED;
unsigned long periodoSimUs = 1000000UL / simulacaoFreqHz;

float invPeriodoSimUs = simulacaoFreqHz / 1000000.0f;

float adcPorVolt = adcMax / VREF;
float voltsPorBit = VREF / adcMax;
float adcLimiteAberto = adcMax * 0.997f;
float centroSim = (VREF / 2.0f) + simulacaoOffset;
float fasesimulacaoOffset = (simulacaoFaseGraus / 360.0f);

void setup()
{
  // Inicializa inputs e outputs
  pinMode(pinoMedicao, INPUT_PULLUP);
  pinMode(pinoDisplay, INPUT_PULLUP);
  pinMode(pinoBuzzer, OUTPUT);

  Serial.begin(115200); // Inicio de logs via monitor serial

  configurarModoMedicao();

  // Inicializacao dos perifericos I2C
  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();

  oled.setI2CAddress(ENDERECO_OLED << 1);
  oled.begin();
  oled.setFont(u8g2_font_6x10_tr);
  oled.setFontPosTop();

  Wire.setClock(400000); // Define a frequencia I2C

  ultimoTempoSimUs = micros();
}

void loop()
{
  lerBotaoMedicao(); // Leia os botoes porfavor
  lerBotaoDisplay(); // Leia os botoes porfavor

  long leituraADC = obterLeituraSinal();   // Leitura de ADC real ou simulada
  float tensao = leituraADC * voltsPorBit; // Tensao em volts

  // Atualizamos as metricas imediatamente para exibir nos logs
  atualizarMetricasContinuas(tensao);

  modoMedicaoAlt = (modoDisplay == 0) ? modoMedicao : 2;

  // Selecao dos modos de medicao, equivalente a uma estrutura If - Else if
  switch (modoMedicaoAlt) // Evita emissao no pino D2 ou D3 no modo de display
  {
  case 0:
    executarTesteContinuidade(tensao);
    break;

  case 1:
    executarTesteDiodo(tensao);
    break;

  case 2:
    executarTesteBateria(tensao);
    break;

  case 3:
    executarTesteResistencia(leituraADC);
    break;
  }

  // Selecao dos modos de display, equivalente a uma corrente If - Else if
  switch (modoDisplay)
  {
  case 0:
    exibirModoPadrao();
    break;

  case 1:
    exibirModoOsciloscopio();
    break;

  case 2:
    exibirModoMetricas();
    break;
  }

  // Coloca um delay de 100 ms para evitar spam nos logs
  if (millis() - ultimoSerialMs >= 100)
  {
    ultimoSerialMs = millis();

    // Modo entre colchetes
    Serial.print(F("["));
    Serial.print(nomesModo[modoMedicaoAlt]);
    Serial.print(F("] "));

    Serial.print(textoMedicao); // Tensao ou OL

    // Analise de tensao
    Serial.print(F(" | Vmax:"));
    Serial.print(vMax, 1);

    Serial.print(F("V Vmin:"));
    Serial.print(vMin, 1);

    // Analise de onda
    Serial.print(F("V | F:"));
    Serial.print(frequencia, 1);

    Serial.print(F("Hz CT:"));
    Serial.print(cicloTrabalho, 0);

    Serial.print(F("% T:"));
    Serial.print(tempoCiclo, 0);
    Serial.println(F("ms"));
  }
}

// ------------------------------------------------------------------
// FUNCOES AUXILIARES
// ------------------------------------------------------------------

// Funcao para simular a onda e testar as capacidades do sistema
long obterLeituraSinal()
{
  // Faz a leitura regular se nao for desativada
  if (!simulacaoOndaAtiva)
  {
    return analogRead(pinoLeitura);
  }

  // Grava o tempo e periodo para desenhar a onda
  unsigned long agoraUs = micros();

  // Usa um acumulador que retorna ao 0 a cada periodo para gerar a onda
  tempoAcumuladoSimUs += (agoraUs - ultimoTempoSimUs);
  ultimoTempoSimUs = agoraUs;
  if (periodoSimUs > 0)
  {
    tempoAcumuladoSimUs %= periodoSimUs;
  }

  // Controle de fase da onda simulada
  float fase = (tempoAcumuladoSimUs * invPeriodoSimUs) + fasesimulacaoOffset;
  fase -= floor(fase);

  float forma = 0.0f;

  switch (simulacaoTipoOnda)
  {
  case 0:
    // Onda senoidal
    forma = sin(2.0f * PI * fase);
    break;

  case 1:
    // Onda quadrada com ciclo de trabalho
    forma = (fase < (simulacaoCicloTrabalho * 0.01f)) ? 1.0f : -1.0f;
    break;

  case 2:
    // Onda dente de serra
    forma = (fase * 2.0f) - 1.0f;
    break;

  case 3:
    // Onda triangular com tratamento especifico para fase
    if (fase < 0.25f)
      forma = fase * 4.0f;

    else if (fase < 0.75f)
      forma = 2.0f - (fase * 4.0f);

    else
      forma = (fase * 4.0f) - 4.0f;

    break;
  }

  // Valor final que denota a tensao da onda no passo de tempo atual
  float tensao = centroSim + (simulacaoAmplitude * forma);

  // Limita o output a um alcance legivel pelo ADC configurado
  long tensaoADC = round(tensao * adcPorVolt);
  return constrain(tensaoADC, 0L, adcMax);
}

// Configuracao para medicao com duas pontas de teste
void configurarModoMedicao()
{
  // Inicializa os pinos
  digitalWrite(pinoFonte10k, LOW);
  pinMode(pinoFonte10k, INPUT);

  digitalWrite(pinoFonte3x10k, LOW);
  pinMode(pinoFonte3x10k, INPUT);

  modoMedicaoAlt = (modoDisplay == 0) ? modoMedicao : 2;

  switch (modoMedicaoAlt) // Evita emissao no pino D2 ou D3 no modo de display
  {
  case 0:
    // Configura teste de continuidade
    pinMode(pinoFonte10k, OUTPUT);
    digitalWrite(pinoFonte10k, HIGH);
    break;

  case 1:
    // Configura medicao de diodo
    pinMode(pinoFonte3x10k, OUTPUT);
    digitalWrite(pinoFonte3x10k, HIGH);
    break;

  case 2:
    // Mantem a configuracao padrao, teste de bateria
    break;

  case 3:
    // Configura teste de resistencia
    pinMode(pinoFonte10k, OUTPUT);
    digitalWrite(pinoFonte10k, HIGH);
    break;
  }
}

// Funcao para atualizar as metricas de analise
void atualizarMetricasContinuas(float tensao)
{
  // Gravar o valor maximo de tensao e simples o suficiente
  if (tensao > vMax)
    vMax = tensao;

  if (tensao < vMin)
    vMin = tensao;

  // Se o sinal for fraco nao atualize as metricas seguintes
  if ((vMax - vMin) <= 0.1f)
    return;

  float pontoMedio = (vMax + vMin) / 2.0f; // O ponto entre vMax e vMin e crucial para determinar o ciclo de trabalho

  // Sinal e maior que o ponto medio?
  bool nivelAlto = (tensao > (bordaAnterior ? (pontoMedio - 0.02f) : (pontoMedio + 0.02f)));

  // Se nao houve mudanca HIGH -> LOW ou LOW -> HIGH no sinal, nao atualize as metrica seguintes
  if (nivelAlto == bordaAnterior)
    return;

  unsigned long agoraUs = micros(); // Necessario para determinar ciclo de trabalho

  if (nivelAlto)
  {
    if (ultimaBordaUs != 0)
    {
      unsigned long periodoLocalUs = agoraUs - ultimaBordaUs; // Tempo entre mudanca HIGH e LOW

      if (periodoLocalUs > 0)
      {
        tempoCiclo = periodoLocalUs * 0.001f; // microsegundo -> millisegundo
        frequencia = 1000.0f / tempoCiclo;    // millisegundo -> Hz

        float tempoLocal = tempoAltoUs;
        cicloTrabalho = constrain((tempoLocal / periodoLocalUs) * 100.0f, 0.0f, 100.0f);
      }
    }
    ultimaBordaUs = agoraUs;
  }
  else
  {
    tempoAltoUs = agoraUs - ultimaBordaUs;
  }

  bordaAnterior = nivelAlto;
}

// Controle direto do I2C para melhoria de performance, retorna o "step" de desenho ao inicio
void restaurarModoPaginaOLED()
{
  Wire.beginTransmission(ENDERECO_OLED);
  Wire.write(0x00);             // Coloca o display no modo "input de comandos"
  Wire.write(0x21);             // Comando para definir faixa de colunas de pixeis no eixo X
  Wire.write(0);                // Primeira coluna de pixeis
  Wire.write(LARGURA_OLED - 1); // Ultima coluna de pixeis
  Wire.write(0x22);             // Comando para definir "paginas" de pixeis no eixo Y
  Wire.write(0);                // Primeira pagina de pixeis
  Wire.write(7);                // Ultima pagina de pixeis
  Wire.write(0x20);             // Modo de enderecamento de memoria
  Wire.write(0x02);             // Sai do modo 0x00 e retorna ao padrao da biblioteca
  Wire.endTransmission();
}

// Funcao para desenhar a grade "transparente" do oscilloscopio, escreve diretamente para economizar memoria
void desenharColunaOsciloscopio(int x, int yAnterior, int yAtual)
{
  byte col[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Paginas de pixeis (8px por coluna)
  col[1] |= 0x04;                         // Linha que separa o cabecalho (y = 10)

  // Se a coordenada x estiver entre 4 pixeis da esquerda e direita da tela...
  if (x >= 4 && x <= 124)
  {
    // Se o resto for zero, desenhe a grade vertical nesta coordenada
    if ((x - 4) % GRADE_X == 0)
    {
      // Controle direto dos bits para desenhar no display
      col[1] |= 0x80;
      col[2] |= 0x10;
      col[3] |= 0x42;
      col[4] |= 0x08;
      col[5] |= 0x21;
      col[6] |= 0x84;
      col[7] |= 0x10;
    }

    // Se o resto for zero, desenhe a grade horizontal nesta coordenada
    else if ((x - 4) % GRADE_PASSO == 0)
    {
      // Controle direto dos bits para desenhar no display
      col[1] |= 0x80;
      col[3] |= 0x40;
      col[5] |= 0x20;
      col[7] |= 0x10;
    }
  }

  // Evita desenhar a grade sobre a onda se elas se cruzarem
  int yMin = min(yAnterior, yAtual);
  int yMax = max(yAnterior, yAtual);

  for (int y = yMin; y <= yMax; y++)
  {
    col[y >> 3] |= (1 << (y & 0x07));
  }

  Wire.beginTransmission(ENDERECO_OLED);
  Wire.write(0x00); // Coloca o display no modo "input de comandos"
  Wire.write(0x21); // Comando para definir faixa de colunas de pixeis no eixo X

  // Desenhe apenas nesta coordenada
  Wire.write(x);
  Wire.write(x);

  Wire.write(0x22); // Comando para definir "paginas" de pixeis no eixo Y

  // Desenhe no alcance inteiro de paginas
  Wire.write(1);
  Wire.write(7);

  Wire.endTransmission();

  Wire.beginTransmission(ENDERECO_OLED);
  Wire.write(0x40);       // Coloca o display no modo "input de dados"
  Wire.write(&col[1], 7); // Desenha o conteudo das colunas
  Wire.endTransmission();
}

// Leitura do botao do modo de medicao
void lerBotaoMedicao()
{
  bool botao = digitalRead(pinoMedicao);

  // Leitura do botao com debounce
  if (millis() - ultimoCliqueBotao > 50)
  {
    if (botao == LOW && botaoMedicaoAnterior == HIGH)
    {
      ultimoCliqueBotao = millis();

      modoMedicao = (modoMedicao + 1) % 4; // Operador modulo limita o valor maximo
      configurarModoMedicao();
      noTone(pinoBuzzer);

      ultimoDisplayMs = 0;
    }
    botaoMedicaoAnterior = botao;
  }
}

// Leitura do botao do modo de display
void lerBotaoDisplay()
{
  bool botao = digitalRead(pinoDisplay);

  // Leitura do botao com debounce
  if (millis() - ultimoCliqueBotao > 50)
  {
    if (botao == LOW && botaoDisplayAnterior == HIGH)
    {
      ultimoCliqueBotao = millis();

      modoDisplay = (modoDisplay + 1) % 3; // Operador modulo limita o valor maximo
      configurarModoMedicao();
      noTone(pinoBuzzer);

      restaurarModoPaginaOLED();
      lcd.clear();
      oled.clearDisplay();

      eixoXGrafico = 0;
      eixoYAnterior = ALTURA_OLED - 1;
      lcdAtualizado = false;
      ultimaAmostraUs = micros();
      ultimoDisplayMs = 0;

      // Resetamos as metricas
      vMin = VREF;
      vMax = 0.0f;
      frequencia = 0.0f;
      tempoCiclo = 0.0f;
      cicloTrabalho = 0.0f;
      ultimaBordaUs = 0;
      tempoAltoUs = 0;
      bordaAnterior = false;
    }
    botaoDisplayAnterior = botao;
  }
}

// ------------------------------------------------------------------
// BLOCO DE LOGICA DE MEDICAO
// ------------------------------------------------------------------

// Executa o teste de continuidade, emite som se a corrente medida for baixa
void executarTesteContinuidade(float tensao)
{
  if (tensao < 0.035f)
  {
    tone(pinoBuzzer, 600);
    strcpy(textoMedicao, "BEEP");
  }
  else
  {
    noTone(pinoBuzzer);
    strcpy(textoMedicao, "OL");
  }
}

// Executa o teste de diodo, exibe a voltagem ou circuito aberto
void executarTesteDiodo(float tensao)
{
  if (tensao > 3.0f)
  {
    strcpy(textoMedicao, "OL");
  }
  else
  {
    dtostrf(tensao, 1, 2, textoMedicao); // Double para string
    strcat(textoMedicao, " V");
  }
}

// Executa o teste de bateria, mostra a voltagem restante
void executarTesteBateria(float tensao)
{
  dtostrf(tensao, 1, 2, textoMedicao); // Double para string
  strcat(textoMedicao, " V");
}

// Executa o teste de resistencia, compara um resistor conhecido para retornar um valor em Ohm ou circuito aberto
void executarTesteResistencia(long leituraADC)
{
  if (leituraADC >= adcLimiteAberto)
  {
    strcpy(textoMedicao, "OL");
  }
  else
  {
    float rx = R_REF * leituraADC / (float)(adcMax - leituraADC);
    if (rx >= 1000000.0f)
    {
      dtostrf(rx / 1000000.0f, 1, 2, textoMedicao);
      strcat(textoMedicao, " Mohm");
    }
    else if (rx >= 1000.0f)
    {
      dtostrf(rx / 1000.0f, 1, 1, textoMedicao);
      strcat(textoMedicao, " kohm");
    }
    else
    {
      dtostrf(rx, 1, 0, textoMedicao);
      strcat(textoMedicao, " ohm");
    }
  }
}

// ------------------------------------------------------------------
// BLOCO DE LOGICA DOS DISPLAYS
// ------------------------------------------------------------------

void exibirModoPadrao()
{
  // Atualiza o display a cada 100 ms
  if (millis() - ultimoDisplayMs < 100)
    return;

  char linhaLCD[17]; // Espaco de 17 caracteres por linha do LCD

  // Este metodo de escrita evita usar lcd.clear() antes de escrever o texto novo
  // %- -> insere alinhado a esquerda
  // 10 -> quantidade de caracteres alocados
  // s  -> tipo de dado string
  snprintf(linhaLCD, sizeof(linhaLCD), "MODO: %-10s", nomesModo[modoMedicao]);
  lcd.setCursor(0, 0);
  lcd.print(linhaLCD);

  snprintf(linhaLCD, sizeof(linhaLCD), "%-16s", textoMedicao);
  lcd.setCursor(0, 1);
  lcd.print(linhaLCD);

  // Escrita do texto, while (oled.nextPage()) atualiza o display uma "pagina" de cada vez
  oled.firstPage();
  do
  {
    atualizarMetricasContinuas(obterLeituraSinal() * voltsPorBit);

    oled.setCursor(0, 4);
    oled.print(F("MODO: "));
    oled.print(nomesModo[modoMedicao]);

    oled.setCursor(0, 20);
    oled.print(F("MEDICAO:"));

    oled.setCursor(0, 36);
    oled.print(textoMedicao);
  } while (oled.nextPage());

  ultimoDisplayMs = millis();
}

void exibirModoOsciloscopio()
{
  // Evita atualizar o display LCD a cada chamada, o texto e fixo e nao precisa mudar
  if (!lcdAtualizado)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("Faixa:  0V - "));
    lcd.print(VREF, 0);
    lcd.print(F("V"));

    lcd.setCursor(0, 1);
    lcd.print(F("Escala: "));
    lcd.print(escalaTempoTela, 1);
    lcd.print(F("s     "));

    lcdAtualizado = true;
  }

  // Inicia o desenho do grafico e onda, evita redesenhar a tela inteira
  if (eixoXGrafico == 0)
  {
    // Retorna o display ao modo de desenho da biblioteca U8g2
    restaurarModoPaginaOLED();

    // Desenha o conteudo "fixo" da tela
    oled.firstPage();
    do
    {
      // Atualiza as metricas para a secao no topo da tela
      atualizarMetricasContinuas(obterLeituraSinal() * voltsPorBit);

      // Desenho do texto das metricas em apenas uma linha
      oled.setCursor(0, 0);
      oled.print(F("0-"));
      oled.print(VREF, 0);
      oled.print(F("V "));

      oled.print(escalaTempoTela, 1);
      oled.print(F("S "));
      oled.print(frequencia, 1);
      oled.print(F("HZ"));

      // Linha que separa a onda da secao de dados
      oled.drawHLine(0, Y_INICIO_GRAFICO - 2, LARGURA_OLED);

      // Desenho da grade "transparente" sem sobrepor intersecoes verticais e horizontais
      for (int graficoX = 4; graficoX <= 124; graficoX += GRADE_PASSO)
      {
        if ((graficoX - 4) % GRADE_X == 0)
        {
          for (int graficoY = 15; graficoY <= 60; graficoY += GRADE_PASSO)
            oled.drawPixel(graficoX, graficoY);
        }
        else
        {
          for (int graficoY = 15; graficoY <= 60; graficoY += GRADE_Y)
            oled.drawPixel(graficoX, graficoY);
        }
      }
    } while (oled.nextPage());

    // Controle direto do I2C para melhorar a performance
    Wire.beginTransmission(ENDERECO_OLED);
    Wire.write(0x00); // Input de comandos
    Wire.write(0x20); // Enderecamento de memoria
    Wire.write(0x00); // Enderecamento horizontal
    Wire.endTransmission();

    // Define o eixoY do sinal na tela, mapeando o valor de 0-1024 para a resolucao do display
    int eixoY = map(obterLeituraSinal(), 0, adcMax, ALTURA_OLED - 1, Y_INICIO_GRAFICO);
    desenharColunaOsciloscopio(0, eixoY, eixoY);
    eixoYAnterior = eixoY;

    eixoXGrafico = 1; // Avanca a coordenada X imediatamente
    ultimaAmostraUs = micros();

    return;
  }

  // Se o tempo entre a ultima medida nao for suficiente para avancar horizontalmente...
  if (micros() - ultimaAmostraUs < passoOscUs)
    return; // Nao faca nada

  // Avanca o tempo pelo passo entre o pixel anterior e atual
  ultimaAmostraUs += passoOscUs;

  // Se o tempo for suficiente ressincroniza o tempo para comparar na proxima chamada
  if (micros() - ultimaAmostraUs > passoOscUs)
    ultimaAmostraUs = micros();

  // Define o eixoY do sinal na tela, mapeando o valor direto do ADC para a resolucao do display
  int eixoY = map(obterLeituraSinal(), 0, adcMax, ALTURA_OLED - 1, Y_INICIO_GRAFICO);
  desenharColunaOsciloscopio(eixoXGrafico, eixoYAnterior, eixoY);
  eixoYAnterior = eixoY;

  // Avanca a coordenada X, esta secao e executada no loop desenhando apenas a onda
  eixoXGrafico++;

  // Reseta o display e inicia a onda da coordenada 0 novamente
  if (eixoXGrafico >= LARGURA_OLED)
  {
    eixoXGrafico = 0;
    restaurarModoPaginaOLED();
  }
}

void exibirModoMetricas()
{
  // Intervalo dinamico para analise de metricas, altera a cada ciclo para evitar harmonizar com a onda
  unsigned long intervaloDinamico = 180 + (((unsigned long)(tempoCiclo * 0.37f) + (passoAntiHarmonico * 43)) % 97);

  // Evita realizar uma analise antes do intervalo dinamico passar
  if (millis() - ultimoDisplayMs < intervaloDinamico)
    return;

  // Incrementa o byte de passo ate 255 e retorna ao 0 com overflow
  passoAntiHarmonico++;

  // Tensao local para analise
  float tensao = 0.0f;

  // Desenha o conteudo principal do display
  oled.firstPage();
  do
  {
    tensao = obterLeituraSinal() * voltsPorBit;
    atualizarMetricasContinuas(tensao);

    oled.setCursor(0, 1);
    oled.print(F("TENSAO"));

    oled.setCursor(72, 1);
    oled.print(F("V:"));
    oled.print(tensao, 2);
    oled.print(F("V"));

    oled.setCursor(0, 13);
    oled.print(F("Vmax:"));
    oled.print(vMax, 1);
    oled.print(F("V Vmin:"));
    oled.print(vMin, 1);
    oled.print(F("V"));

    oled.drawHLine(0, 26, LARGURA_OLED);

    oled.setCursor(0, 30);
    oled.print(F("ONDA"));

    oled.setCursor(0, 42);
    oled.print(F("F:"));
    oled.print(frequencia, 1);
    oled.print(F("Hz  CT:"));
    oled.print(cicloTrabalho, 0);
    oled.print(F("%"));

    oled.setCursor(0, 54);
    oled.print(F("T:"));
    oled.print(tempoCiclo, 0);
    oled.print(F("ms"));
  } while (oled.nextPage());

  // Atualiza o conteudo do LCD
  lcd.setCursor(0, 0);
  lcd.print(F("F:"));
  lcd.print(frequencia, 1);
  lcd.print(F("Hz CT:"));
  lcd.print(cicloTrabalho, 0);
  lcd.print(F("% "));

  lcd.setCursor(0, 1);
  lcd.print(F("V:"));
  lcd.print(tensao, 1);
  lcd.print(F(" +"));
  lcd.print(vMax, 1);
  lcd.print(F(" -"));
  lcd.print(vMin, 1);
  lcd.print(F(" "));

  ultimoDisplayMs = millis();
}
