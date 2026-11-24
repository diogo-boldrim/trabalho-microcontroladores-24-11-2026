// Constantes de hardware
const int pinoLeitura = A0;   // Pino analogico para leitura de voltagem
const int pinoBuzzer = 12;    // Pino de saida para emissao de feedback sonoro
const int pinoMedicao = 7;    // Pino de entrada PULLUP para modos de medicao
const int pinoFonte10k = 2;   // D2 -> resistor 10k -> no P (continuidade e resistencia)
const int pinoFonte3x10k = 3; // D3 -> 3x resistor 10k  -> no P (diodo)
const int bitsADC = 10;       // Resolucao ADC/DAC em bits (10 = 1023 etc.)

const float R_REF = 10000.0f; // Resistencia de referencia
const float VREF = 5.0f;      // Voltagem de referencia para medicao via ADC

// Variaveis para controle de estados
int modoMedicao = 0; // 0: CONT, 1: DIODO, 2: BAT, 3: RES

bool botaoMedicaoAnterior = HIGH; // Estado anterior do botao para debounce

// Variaveis de controle para testes internos com onda simulada
bool simulacaoOndaAtiva = false; // Alterne para false para ler o pino A0

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

// Memoria interna para o serialMonitor
char textoMedicao[16]; // O texto de log do serialMonitor e copiado para esta variavel
const char *nomesModo[4] = {"CONT", "DIODO", "BAT", "RES"};

// Variaveis para analise de metricas
float vMin = VREF;          // Voltagem minima medida
float vMax = 0.0f;          // Voltagem maxima medida
float frequencia = 0.0f;    // Frequencia da onda em hertz
float tempoCiclo = 0.0f;    // Tempo entre uma oscilacao completa em millisegundos
float cicloTrabalho = 0.0f; // Ciclo de trabalho da onda, formato de porcentagem

bool bordaAnterior = false; // Ultimo estado do sinal medido, sendo HIGH ou LOW

// Variaveis pre-processadas, nao altere
long adcMax = (1L << bitsADC) - 1; // Resolucao do ADC, usa operador "left bit shift" (1L << 10 => 1024 - 1 => 1023)

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
  pinMode(pinoBuzzer, OUTPUT);

  Serial.begin(115200); // Inicio de logs via monitor serial

  configurarModoMedicao();

  ultimoTempoSimUs = micros();
}

void loop()
{
  lerBotaoMedicao(); // Leia os botoes porfavor

  long leituraADC = obterLeituraSinal();   // Leitura de ADC real ou simulada
  float tensao = leituraADC * voltsPorBit; // Tensao em volts

  // Atualizamos as metricas imediatamente para exibir nos logs
  atualizarMetricasContinuas(tensao);

  // Selecao dos modos de medicao, equivalente a uma estrutura If - Else if
  switch (modoMedicao)
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

  // Coloca um delay de 100 ms para evitar spam nos logs
  if (millis() - ultimoSerialMs >= 100)
  {
    ultimoSerialMs = millis();

    // Modo entre colchetes
    Serial.print(F("["));
    Serial.print(nomesModo[modoMedicao]);
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

  switch (modoMedicao)
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
    }
    botaoMedicaoAnterior = botao;
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
