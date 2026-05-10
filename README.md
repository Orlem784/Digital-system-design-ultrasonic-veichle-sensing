# Sistema di Monitoraggio e Classificazione Traffico (Arduino)

## Descrizione del Progetto
Questo progetto implementa un sistema avanzato di rilevamento veicoli utilizzando una coppia di sensori a ultrasuoni gestiti da una scheda Arduino. Il sistema non si limita a rilevare la presenza di un veicolo, ma ne calcola con precisione la **velocità** e la **lunghezza**, classificandolo in diverse categorie (Moto, Auto, Camion).

Il codice integra diverse tecniche di elaborazione del segnale per garantire affidabilità in ambienti esterni, tra cui la compensazione della velocità del suono in base alla temperatura e filtri digitali per la pulizia dei dati.

## Caratteristiche Principali
- **Macchina a Stati (FSM):** Gestione robusta del transito veicolare attraverso gli stati `STATO_ATTESA`, `STATO_IN_S1` e `STATO_IN_S2`.
- **Compensazione Ambientale:** Integrazione con sensore **DHT11** per regolare il calcolo della velocità del suono (v = 331.45 + 0.62 * T).
- **Filtrazione Dati:** - **Filtro Mediano:** Per l'eliminazione degli spike (errori di lettura isolati).
  - **Filtro di Kalman:** Per stabilizzare le letture della distanza in tempo reale.
- **Gestione Code Dinamiche:** Utilizzo di una lista concatenata (`struct node`) per memorizzare i dati dei veicoli in transito.
- **Comunicazione Wireless:** Supporto per moduli **HC-12** per la sincronizzazione dei dati con altri dispositivi lungo la strada.
- **Interfaccia Utente:** Controllo tramite telecomando a infrarossi (IR) per avvio/arresto delle misurazioni.

## Hardware Richiesto
- **Microcontrollore:** Arduino (Uno/Mega o compatibili).
- **Sensori Ultrasuoni:** 2x HC-SR04 (collegati ai pin 9 e 6).
- **Sensore Ambiente:** 1x DHT11 (pin 2).
- **Comunicazione:** Modulo HC-12 (pin 8 TX, 11 RX).
- **Ricevitore IR:** TSOP382 o simili (pin 4).
- **Display:** LiquidCrystal (opzionale, supportato dalle librerie).

## Librerie Necessarie
Assicurati di installare le seguenti librerie tramite l'Arduino Library Manager:
- `NewPing`
- `LiquidCrystal`
- `IRremote`
- `SoftwareSerial`
- `dht_nonblocking`

##  Logica di Calcolo
Il sistema utilizza la distanza nota (d) tra i due sensori e i timestamp microsecondali per calcolare:
1. **Velocità (v):** Calcolata come rapporto tra la distanza inter-sensore e il tempo di volo tra le uscite dai sensori.
2. **Lunghezza (l):** Calcolata moltiplicando la velocità per il tempo di occupazione del sensore, sottraendo un fattore di correzione geometrica (Avar).

### Classificazione Veicoli:
- **MOTO:** < 200 cm
- **AUTO PICCOLA:** 200 - 400 cm
- **AUTO GRANDE:** 400 - 600 cm
- **CAMION:** > 600 cm

## Comandi Seriali e IR
- **Telecomando IR:** Pulsante specifico (codice `0xB847FF00`) per alternare lo stato di misura.
- **Monitor Seriale (9600 baud):** Inviare il comando `stampa` per visualizzare la coda dei veicoli memorizzati.

---
*Progetto sviluppato per sistemi di Smart City e monitoraggio del traffico urbano.*
