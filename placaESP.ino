//Definição das bibliotecas
  #define BLYNK_PRINT Serial
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <BlynkSimpleEsp32.h>
  #include <math.h>       //para usar a raiz quadrada
  #include "ZMPT101B.h"   //sensor de tensão
  #include "ACS712.h"     //sensor de corrente
  #include "DHT.h"        //sensor de temperatura e humidade

//Credênciais do BLYNK e do Wi-Fi
  char auth[] = "hcumPgO0LkGb9b_cnxkWQcU8k8RF4Wnu";
  //char ssid[] = "MEO-4D6DA0";
  //char pass[] = "c7a1553fdd";
  char ssid[] = "ZTE_CDA0A3";
  char pass[] = "45183030";

//Variáveis globais
  const unsigned int buttonPin = 12;    //botão está ligado a este pin
  const int relayPin = 2;              //Output está ligado a este pin
  const unsigned int greenLedPin = 27;  //LED verde está ligado a este pin
  const unsigned int redLedPin = 26;    //LED vermelho está ligado a este pin
  const unsigned int relayPin_V = V0;
  const unsigned int virtualPin_Temp = V1, virtualPin_Hum = V2;
  const unsigned int virtualPin_V = V3, virtualPin_I = V4, virtualPin_P = V5; 

//Variáveis para ligar e desligar a tomada
  int buttonState = 0;          //Estado inicial do botão
  int oldButtonState = LOW;
  int outState = LOW;           //Estado inicial do LED/Output = Desligado
  int lastDebounceTime = 0, debounceDelay = 100;

//Variáveis para sensor de temperatura e humidade
  #define DHTTYPE DHT22    
  #define DHTPIN 15     
  DHT dht(DHTPIN, DHTTYPE);
  unsigned long currentMillis;
  float temp_hum_val[2] = {0};      // humidade = temp_hum_val[0] ; temperatura = temp_hum_val[1]
  float old_temp = 0, old_hum = 0;
  unsigned long lastTime = 0;       //Última vez (tempo) em que existiu mudança de estado do LED/Output          
  unsigned long Delay = 1000;       //Tempo de delay
  int auxTemp = 0;
  int auxHum = 0;

//Variáveis para o sensor de tensão e de corrente
  #define SAMPLES 80      //número de amostras p/ leitura de sensores 
  #define ZMTP_pin 35     //pin do sensor de tensão na ESP32
  #define ACS_pin 34      //pin do sensor de corrente na ESP32
  unsigned long ReadingTime = 10000;  //10 segundos para ler valores V e I
  unsigned long Timer = 250;          //250 microsegundos entre cada amostra
  int lastSensorTime = -30000, sensorDelay = 30000;


BLYNK_CONNECTED() {     //Realizar a conexão o BLYNK (e aos pins virtuais )
  Blynk.syncVirtual(relayPin_V);
  Blynk.syncVirtual(virtualPin_Temp);
  Blynk.syncVirtual(virtualPin_Hum);
  Blynk.syncVirtual(virtualPin_V);
  Blynk.syncVirtual(virtualPin_I);
  Blynk.syncVirtual(virtualPin_P);
}

BLYNK_WRITE(V0){    //Caso exista uma mudança no estado da tomada, na aplicação BLYNK
  int virtual_pinValue = param.asInt();
  if (virtual_pinValue == 0){ //Se a tomada foi desligada
    setLeds(LOW);
    digitalWrite(relayPin, LOW);
  }
  if (virtual_pinValue == 1){ //Se a tomada foi ligada
    setLeds(HIGH);
    digitalWrite(relayPin, HIGH);
  }
}

void setLeds(bool estado){  //Função para ligar e desligar LEDs Green e Red conforme o estado do rele (estado = high se a tomada estiver ligada)
  digitalWrite(greenLedPin, estado); 
  digitalWrite(redLedPin, !estado); 
}

void setup(){
  Serial.begin(9600);
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);   //Realizar a ligação ao BLYNK

  //Para o sensor de temperatura e humidade
  Wire.begin();
  dht.begin();

  //Definição do tipo de Pins
  pinMode(relayPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  //Inicializar o rele como desligado e os LEDS Green e Red de acordo com isso
  digitalWrite(relayPin, LOW);
  digitalWrite(greenLedPin, LOW);   //desliga o led verde
  digitalWrite(redLedPin, HIGH);    //liga o led vermelho
}



void loop(){
  Blynk.run();
  
//LIGAR E DESLIGAR RELE 
  //Procura o estado atual do rele, e corrige o estado dos LEDs Green e Red se necessário
  Serial.println(digitalRead(relayPin));
    outState = digitalRead(relayPin);
    setLeds(outState);

  //Verificar se o botão está a ser pressionado ou não
    buttonState = digitalRead(buttonPin);

  //Garantir que passou tempo suficiente, e que todo o "barulho" e interferências são eliminadas
    if((millis() - lastDebounceTime) > debounceDelay){      //Garantir que passou tempo suficiente
      if (buttonState != oldButtonState && buttonState == HIGH){    //botão foi pressionado => mudar o estado do output
        outState = (outState == LOW ? HIGH : LOW);    //Ligar/Desligar LED
        digitalWrite(relayPin, outState);
        Blynk.virtualWrite(relayPin_V, outState);   //Liga/Desliga LED na aplicação BLYNK
        lastDebounceTime = millis();
      }
      setLeds(outState);
      oldButtonState = buttonState;
    }
//SENSOR DE TEMPERATURA E HUMIDADE
  //Garantir que passou tempo suficiente, e que todo o "barulho" e interferências são eliminadas
    currentMillis = millis();
    if((currentMillis - lastTime) > Delay){      //Garantir que passou tempo suficiente  
      if (dht.readTempAndHumidity(temp_hum_val) == 0) { //Se o sensor conseguiu realizar uma leitura (é 0 se sim, é -1 se não)
        if (abs(old_temp - temp_hum_val[1]) >= 0.01){
          auxTemp = 1;
          old_temp = temp_hum_val[1];
          Blynk.virtualWrite(virtualPin_Temp, temp_hum_val[1]);
        }
          if (abs(old_hum - temp_hum_val[0]) >= 1){
            auxHum = 1;
            old_hum = temp_hum_val[0];
            Blynk.virtualWrite(virtualPin_Hum, temp_hum_val[0]);
          }
      lastTime = millis();  //Quer tenha lido um novo valor da temperatura ou não, atualiza sempre o valor da variável lastTime
      }
    }
//ENVIAR NOTIFICAÇÕES PARA O BLYNK
  if(auxTemp == 1) { //Se já foi registado algum valor de temperatura
    if (old_temp > 38){ Blynk.logEvent("temperatura", "A temperatura ambiente está demasiado elevada!");}
    if (old_temp < 2){ Blynk.logEvent("temperatura", "A temperatura ambiente está demasiado baixa!");}
  }
  if(auxHum == 1) { //Se já foi registado algum valor de humidade
    if (old_hum > 70){  Blynk.logEvent("humidade", "A humidade relativa do ar está demasiado elevada!");}
    if (old_hum < 20){  Blynk.logEvent("humidade", "A humidade relativa do ar está demasiado baixa!");}
  }

//SENSORES DE TENSÃO E DE CORRENTE, E CÁLCULOS DE TENSÃO TMS, CORRENTE RMS E POTÊNCIA ATIVA
  //Declarar as variáveis 
    unsigned long startTimer;
    float V_lervalores[SAMPLES], I_lervalores[SAMPLES];
    float V_array[SAMPLES], I_array[SAMPLES], P_array[SAMPLES];
    float V_soma = 0, I_soma = 0, P_soma = 0;
    float V_rms, I_rms, P_ativa;

  if((millis() - lastSensorTime) > sensorDelay){      //Garantir que passou tempo suficiente desde que as últimas medições dos sensores foram realizadas
    //Ler e guardas os valores das amostras de tensão e de corrente
      for (int i = 0 ; i < SAMPLES ; i++){
        startTimer = micros();
        V_lervalores[i] = analogRead(ZMTP_pin);
        I_lervalores[i] = analogRead(ACS_pin);
        while ((micros() - startTimer) < Timer){ } //espera que passe tempo suficiente até ler a próxima amostra
      }
    //Alterar a escala de cada uma das amostras e calcular a potencia ativa de cada par de amostras
      for(int j=0; j<SAMPLES; j++){ 
        V_array[j] = ((((float(V_lervalores[j])- 2048) / 2048) * 1.65) * 239.99); //tensão deve encontrar-se entre (+/- 280*sqrt(2)), sendo esse valor o máximo (~=395.98), consegue medir até esse valor
        I_array[j] = ((float(I_lervalores[j])/4095*3.3 )- 1.65) / (0.185 *(5/0.66));
        P_array[j] = V_array[j] * I_array[j];
      }
    //Calcular os valores eficazes da tensão e da corrente -> elevar ao quadrado todos os valores
      for(int j=0; j<SAMPLES; j++){
        V_array[j] = V_array[j] * V_array[j];
        I_array[j] = I_array[j] * I_array[j];
      }
    //Calcular média desses valores -> primeiro a soma de todos e depois o cálculo da média de V e de I
      for(int j=0; j<SAMPLES; j++){
        V_soma += V_array[j];
        I_soma += I_array[j];
        P_soma += P_array[j];
      }
      V_rms = V_soma / SAMPLES;
      I_rms = I_soma / SAMPLES;
      P_ativa = P_soma / SAMPLES;
    //Calcular os valores finais de tensão e de corrente rms -> raiz quadrada dos valores médios de V e de I
      V_rms = sqrt(V_rms);
      I_rms = sqrt(I_rms);

    //Colocar a zero, caso os valores seja residuais
      if(V_rms < 180){
        V_rms = 0;
        I_rms = 0;
        P_ativa = 0;
      }

    //Enviar os valores finais para a aplicação BLYNK
      Blynk.virtualWrite(virtualPin_V, V_rms);
      Blynk.virtualWrite(virtualPin_I, I_rms);
      Blynk.virtualWrite(virtualPin_P, P_ativa);
  }
}