#include <NewPing.h>
#include <LiquidCrystal.h>
#include <IRremote.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <dht_nonblocking.h>
#define PING_PIN1 9
#define PING_PIN2 6  // Arduino pin tied to both trigger and echo pins on the ultrasonic sensor.
// Pin TX/RX
#define PIN_TX 8
#define PIN_RX 11
#define DHT_SENSOR_TYPE DHT_TYPE_11
#define MAX_DISTANCE 450
#define PING_INTERVAL 30  // Milliseconds between sensor pings (ora usato come delay)


unsigned long measurement_timestamp = 0;
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

String classe = "";
String comando = "";

float media_l;

// --- COSTANTI FISICHE (DA MISURARE!) ---
const float d = 100;  // ESEMPIO: Distanza REALE tra i sensori, in cm.
const float A = 62;   // a distanza 160cm di distanza
float Avar = 0;
const int triggerDist = 250;
int county = 0;  // Distanza (in cm) sotto la quale rileva l'oggetto
int count_macchine = 0;

// Contatori sensore 3
int count_tolte = 0;
int count_aggiunte = 0;
int count_rx = 0;
int last_count_tolte = 0;
int flag = -2;

// ==========================================================
// --- STATI (Logica come da PAPER III.C) ---
// ==========================================================
#define STATO_ATTESA 0  // Aspetta che l'auto entri in S1 (per t0)
#define STATO_IN_S1 1   // Auto in S1, aspetta che entri in S2
#define STATO_IN_S2 2   // Auto in S2, aspetta che ESCA da S2 (per t3)
// ==========================================================


// --- Setup Sensori (1-PIN) ---
NewPing sonar1(PING_PIN1, PING_PIN1, MAX_DISTANCE);
NewPing sonar2(PING_PIN2, PING_PIN2, MAX_DISTANCE);

// Setup wireless
SoftwareSerial HC12(PIN_TX, PIN_RX);


// --- Variabili Globali ---
double kaldist1;
double kaldist2;
// Array per le 3 distanze (Indici 0, 1, 2)
double distTemp1[] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };
double distTemp2[] = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE };
unsigned long tTemp1[] = { 0, 0, 0 };
unsigned long tTemp2[] = { 0, 0, 0 };
int number_cycles = 0;
float temperature = 0.0;
float humidity = 0.0;
float v_suono = 340;

// ==========================================================
// --- MODIFICA: Variabili Timer (standardizzate in microsecondi) ---
// ==========================================================
unsigned long t0_micros = 0;
unsigned long t1_micros = 0;
unsigned long t3_micros = 0;
unsigned long t2_micros = 0;
int statoRilevazione = STATO_ATTESA;
unsigned long startCiclo = 0;
unsigned long endCiclo = 0;

// Timer per i ping (ora usano millis() per l'intervallo)
unsigned long time_ping1 = 0;
unsigned long time_ping2 = PING_INTERVAL;  //sfasamento iniziale
// ==========================================================


// Variabili di calcolo
float v;  // velocità (cm/s)
float l;  // lunghezza (cm)
float l2;
float l3;
float l4;
// Variabili del telecomando
int RECV_PIN = 4;
int count = 0;

// CODA IN LISTA DINAMICA per gestione dei dati dei veicoli in coda
// struttura dinamica
typedef struct node {
  float velocity;
  float length;
  String classe;
  struct node* next;
} node;

typedef node* ptrNode;

// Inizializzazione lista dinamica
ptrNode coda = NULL;

static bool measure_environment(float* temperature, float* humidity) {


  /* Measure once every four seconds. */
  if (millis() - measurement_timestamp > 4000ul) {
    
    if (dht_sensor.measure(temperature, humidity)) {
        measurement_timestamp = millis();
      return (true);
    }
  }

  return (false);
}

// FUNZIONI DI GESTIONE DELLA CODA
// Ingresso in coda
ptrNode inserisci_in_coda_ric(ptrNode coda, float v, float l, String classe) {
  ptrNode temp;
  if (coda == NULL) {
    temp = (ptrNode)malloc(sizeof(node));
    temp->velocity = v;
    temp->length = l;
    temp->classe = classe;
    temp->next = NULL;
    return temp;
  } else {
    coda->next = inserisci_in_coda_ric(coda->next, v, l, classe);
    return coda;
  }
}

// Rimozione dalla testa
ptrNode rimuoviDaPila(ptrNode coda) {
  ptrNode vecchiatesta = coda;

  if (coda == NULL) {
    return NULL;
  } else {
    coda = coda->next;
    free(vecchiatesta);
    return coda;
  }
}

// Stampa coda
void stampa_coda(ptrNode coda) {
  if (coda == NULL) {
    Serial.println("NULL");
  }
  while (coda != NULL) {
    Serial.print("v = ");
    Serial.println((coda->velocity) * 3.6 / 100);
    Serial.print("l = ");
    Serial.println(coda->length);
    Serial.print("Classe = ");
    Serial.println(coda->classe);
    if (coda->next != NULL)
      Serial.println(" --> ");
    coda = coda->next;
  }
  printf(" --|\n");
}

//funzione per filtro kalman (INVARIATA)

double kalman1(double U) {
  static const double R = 10;
  static const double H = 1.00;
  static double Q = 15;
  static double P = 100;
  static double U_hat = 100;
  static double K = 0.5;
  K = P * H / (H * P * H + R);
  U_hat += +K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

double kalman2(double U) {
  static const double R2 = 10;
  static const double H2 = 1.00;
  static double Q2 = 15;
  static double P2 = 100;
  static double U_hat2 = 100;
  static double K2 = 0.5;
  K2 = P2 * H2 / (H2 * P2 * H2 + R2);
  U_hat2 += +K2 * (U - H2 * U_hat2);
  P2 = (1 - K2 * H2) * P2 + Q2;
  return U_hat2;
}


// ==========================================================
// --- CORREZIONE: Funzione Filtro ---
// Ora usa gli indici corretti (0, 1, 2) per trovare uno spike
// nel valore centrale (indice 1).
// ==========================================================

/**
 * Filtro Mediano a 3 campioni.
 * Sostituisce il valore in distTemp[1] con la mediana
 * dei tre valori [0], [1], e [2].
 * È la soluzione più robusta per eliminare gli spike.
 */
void Filtro(double distTemp[]) {
  double d0 = distTemp[0];
  double d1 = distTemp[1];
  double d2 = distTemp[2];
  double median;

  // Trova la mediana di d0, d1, d2 senza un array di supporto
  // Questa logica complessa è solo un modo veloce per ordinare 3 numeri
  if ((d0 <= d1 && d1 <= d2) || (d2 <= d1 && d1 <= d0)) {
    median = d1;
  } else if ((d1 <= d0 && d0 <= d2) || (d2 <= d0 && d0 <= d1)) {
    median = d0;
  } else {
    median = d2;
  }

  // Aggiorna il valore centrale con la mediana calcolata.
  // Il resto del codice userà questo valore "pulito".
  distTemp[1] = median;
}

void setup() {  // (INVARIATO)

  Serial.begin(9600);
  delay(1000);
  IrReceiver.begin(RECV_PIN, ENABLE_LED_FEEDBACK);
// -------------------------------------------------
// --------------------------------------------------
  Serial.println("Veicolo: Docker");
  Serial.print("Distanza tra sensori:");
  Serial.println(d);
  Serial.print("Distanza sensore-macchina: 180-210 cm");
//---------------------------------------------------
// -------------------------------------------------
  Serial.println("Avvio ricevitore IR...");
  Serial.println("Avvio sensore ad ultrasuoni...");
 Serial.println("Calcolo della temperatura in corso...");
  while(!measure_environment(&temperature, &humidity)){
    delay(1000); 
  }
  Serial.println("Temperatura di partenza: ");
  Serial.println(temperature);
  HC12.begin(9600);  // Serial port initialization to communicate with HC12
}


void loop() {

  // --- GESTIONE IR ON/OFF ---
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.decodedRawData == 0xB847FF00) {
      count++;
      Serial.println("STOP MISURA");
      delay(100);
    } else {
      Serial.print("Codice sconosciuto (HEX): 0x");
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    }
  }
  IrReceiver.resume();


  // Se 'count' è pari -> SISTEMA SPENTO (INVARIATO)
  if (count % 2 == 0) {

    statoRilevazione = STATO_ATTESA;
    t0_micros = 0;
    t1_micros = 0;
    t3_micros = 0;
    flag = -2;

    // Stampa coda
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n') {
        if (comando == "stampa") {
          stampa_coda(coda);
        }
        comando = "";  // reset per prossimo comando
      } else {
        comando += c;
      }
    }
    return;
  } else {
    flag = -3;
  }




  // ==========================================================
  // --- CORREZIONE: Logica di Lettura Sensori ---
  // Usiamo millis() per l'intervallo.
  // Usiamo gli indici corretti [0, 1, 2].
  // Usiamo = per l'assegnazione.
  // ==========================================================

  // --- Sensore 1 ---
  // PING_INTERVAL è in millisecondi, quindi usiamo millis()
  if (millis() - time_ping1 > PING_INTERVAL) {

    // 1. Scorri i vecchi valori
    distTemp1[0] = distTemp1[1];
    distTemp1[1] = distTemp1[2];
    tTemp1[0] = tTemp1[1];
    tTemp1[1] = tTemp1[2];

    // 2. Leggi il nuovo valore e salvalo nell'INDICE 2
    distTemp1[2] = sonar1.ping();
    if (measure_environment(&temperature, &humidity) == true) {
      v_suono = (331.45 + (0.62 * temperature));
      distTemp1[2] = ((v_suono * distTemp1[2]) / (10000.0)) / 2;
    } else {
      distTemp1[2] = ((v_suono * distTemp1[2]) / (10000.0)) / 2;
    }
    // Salva il timestamp in microsecondi
    time_ping1 = millis();  // Resetta il timer per il prossimo ping
    tTemp1[2] = micros();   // Salva il timestamp effettivo
  }

  // --- Sensore 2 ---
  // CORREZIONE: Controlla time_ping2 e legge da sonar2
  if (millis() - time_ping2 > PING_INTERVAL) {

    // 1. Scorri i vecchi valori
    distTemp2[0] = distTemp2[1];
    distTemp2[1] = distTemp2[2];
    tTemp2[0] = tTemp2[1];
    tTemp2[1] = tTemp2[2];

    // 2. Leggi il nuovo valore da SONAR 2 e salvalo nell'INDICE 2
    distTemp2[2] = sonar2.ping();  // CORRETTO
    if (measure_environment(&temperature, &humidity) == true) {
      v_suono = (331.45 + (0.62 * temperature));
      distTemp2[2] = ((v_suono * distTemp2[2]) / (10000.0)) / 2;
    } else {
      distTemp2[2] = ((v_suono * distTemp2[2]) / (10000.0)) / 2;
    }
    // Salva il timestamp in microsecondi
    time_ping2 = millis();  // Resetta il timer per il prossimo ping
    tTemp2[2] = micros();   // Salva il timestamp effettivo
  }


  // --- SISTEMA ACCESO ---
  if (number_cycles > 2) {  // Aspetta che gli array abbiano almeno 3 dati

    startCiclo = micros();  // Per debugging

    // Legge Sonar 1 (Usa l'indice [1] che è stato filtrato)
    Filtro(distTemp1);
    if (distTemp1[1] == 0) {
      kaldist1 = MAX_DISTANCE;
    } else kaldist1 = kalman1(distTemp1[1]);

    // Legge Sonar 2 (Usa l'indice [1] che è stato filtrato)
    Filtro(distTemp2);
    if (distTemp2[1] == 0) {
      kaldist2 = MAX_DISTANCE;
    } else kaldist2 = kalman2(distTemp2[1]);


    // ================================================================
    // --- CORREZIONE: Macchina a Stati (Timeout e Nomi Variabili) ---
    // ================================================================
    switch (statoRilevazione) {

      case STATO_ATTESA:  // Stato 0
        if (kaldist1 < triggerDist && kaldist1 > 0) {
          t0_micros = tTemp1[1];  // Salva t_0 (è in micros)
          statoRilevazione = STATO_IN_S1;
          Serial.println("Ingresso in S1 (t0)");
        }
        break;

      case STATO_IN_S1:  // Stato 1
        // Aspetta ingresso in S2
        if (kaldist2 < triggerDist && kaldist2 > 0) {
          statoRilevazione = STATO_IN_S2;
          if(Avar==0){
               Avar = 0.17632698 * kaldist2;
          } 
          t2_micros = tTemp2[1];
          Serial.println("Ingresso in S2");
        }

        // CORREZIONE: Timeout S1 (5 sec = 5'000'000 micros)
        // Spostato prima del 'break'
        if (t0_micros > 0 && (micros() - t0_micros > 5000000)) {  // 5 sec
          statoRilevazione = STATO_ATTESA;                        // Reset
          Serial.println("Timeout S1");
          t0_micros = 0;
        }
        break;  // Fine STATO_IN_S1


      case STATO_IN_S2:  // Stato 3
        // Aspetta uscita da S1
        if (kaldist1 > triggerDist && t1_micros == 0) {
          t1_micros = tTemp1[1];  // Salva t_1 (è in micros)
          Serial.println("Uscito da 1 (t1)");
        }

        // CORREZIONE: Timeout S2 Entry (5 sec = 5'000'000 micros)
        // Time out spostato da STATO_IN_S1 a STATO_IN_S2
        if (t1_micros > 0 && (micros() - t1_micros > 5000000)) {  // 5 sec
          statoRilevazione = STATO_ATTESA;
          Serial.println("Timeout S2 Entry");
          t0_micros = 0;
          t1_micros = 0;
        }

        // Aspetta uscita da S2
        if (kaldist2 > triggerDist && t1_micros != 0) {  // t1 deve essere già stato salvato
          t3_micros = tTemp2[1];                         // Salva t_3 (è in micros)
          Serial.println("Uscita da S2 (t3) - CALCOLO");

          // --- CALCOLI (ora coerenti in microsecondi) ---
          // 1000000.0 per convertire microsecondi in secondi
          float delta_t_31 = (t3_micros - t1_micros) / 1000000.0;  // (t3 - t1) [s]
          float delta_t_10 = (t1_micros - t0_micros) / 1000000.0;  // (t1 - t0) [s]
          float delta_t_32 = (t3_micros - t2_micros) / 1000000.0;
          float delta_t_20 = (t2_micros - t0_micros) / 1000000.0;

          // Calcola v (Velocità)
          if (delta_t_31 > 0.0000001) {  // Evita divisione per zero
            v = d / delta_t_31;          // v = d / (t3 - t1)
          } else {
            v = 0;
          }

          // Calcola L (Lunghezza)
          if (v > 0) {
           
            l = (v * delta_t_10) - Avar;  // L = v * (t1 - t0) - A
            if (delta_t_20 > 0.000001) {
              l2 = d * (delta_t_10) / (delta_t_20)-Avar;
            } else {
              l2 = -1;  //errore tempo al denominatore
            }

            l3 = ((v * (delta_t_10) + v * delta_t_32) / 2) - Avar;
            float vmix = (d / delta_t_20 + d / delta_t_31) / 2;
            l4 = vmix * delta_t_10 - Avar;
            if (l < 0) l = 0;
          } else {
            l = 0;
          }

          media_l = (l+l2+l3+l4) / 4;

          // Classificazione veicolo
          if (media_l<200){
            classe = "MOTO";
          }
          if (media_l>200 && media_l<400){
            classe = "AUTO PICCOLA";
          }
          if (media_l>400 && media_l<600){
            classe = "AUTO GRANDE";
          }
          if (media_l>600){
            classe = "CAMION";
          }
          

          // Inserimento dei dati del veicolo nella coda
          coda = inserisci_in_coda_ric(coda, v, media_l, classe);
          count_aggiunte++;
          Serial.print("Valore salvato in coda");

          // --- STAMPA RISULTATI ---
          
          Serial.print("Avar (cm): ");
          Serial.println(Avar);
          Serial.print("T (C): ");
          Serial.println(temperature);
          Serial.print("\t V (km/h): ");
          Serial.print((v / 100.0) * 3.6);  // v è in cm/s
          Serial.print("\t L1 (cm): ");
          Serial.println(l);
          Serial.print("\t L2 (cm): ");
          Serial.println(l2);
          Serial.print("\t L3 (cm): ");
          Serial.println(l3);
          Serial.print("\t L4 (cm): ");
          Serial.println(l4);
          Serial.print("\t Media_l (cm): ");
          Serial.println(media_l);
          Serial.print("\t Classe:");
          Serial.println(classe);


          // Resetta tutto per la prossima auto
          t0_micros = 0;
          t1_micros = 0;
          t3_micros = 0;
          Avar = 0; 
          statoRilevazione = STATO_ATTESA;
        }
        break;
    }

    endCiclo = micros();  // Per debugging

    // --- STAMPA DI DEBUG ---
    //Serial.print("t_inizio");
    //Serial.print(startCiclo);
    //Serial.print(";");
    //Serial.print("t_fine");
    //Serial.print(endCiclo);
    //Serial.print(";");
    Serial.print(tTemp1[1]);
    Serial.print(",");
    // Serial.print("t2:");
    Serial.print(tTemp2[1]);
    Serial.print(",");
    //Serial.print("dist1:");
    Serial.print(kaldist1);
    Serial.print(",");
    //Serial.print("dist2:");
    Serial.println(kaldist2);

    county = county + 1;
    startCiclo = 0;
    endCiclo = 0;
  }

  // COMUNICAZIONE CON IL TERZO SENSORE
  // Cancellazione testa della coda se riceve un segnale dal HC-12

  // In the HC12.available() section:µµ
  if (HC12.available()) {
    int received_count = HC12.read();
    if (received_count > count_rx) {
      int vehicles_to_remove = received_count - count_rx;
      for (int i = 0; i < vehicles_to_remove && coda != NULL; i++) {
        coda = rimuoviDaPila(coda);
      }
      count_tolte += vehicles_to_remove;
      count_rx = received_count;
      Serial.println("messaggio ricevuto");
    }
  }

  // se ha tolto la macchina manda il feedback
  if (count_tolte != last_count_tolte) {
    HC12.write(count_tolte);  // Invia il feedback
    Serial.println("messaggio feedback inviato");
  }

  last_count_tolte = count_tolte;


  //

  number_cycles++;
  delay(5);  // Piccolo delay per stabilità, non bloccare il loop
}
