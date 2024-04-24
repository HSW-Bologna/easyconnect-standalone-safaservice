# Easyconnect Standalone

Firmware per la scheda periferica-master semplificata del sistema Easyconnect.

## Compilazione

Il firmware dovrebbe essere compilato con i seguenti strumenti:
 - ESP-IDF v4.4.x (testato con v4.4.6)

## Struttura del Progetto

Il progetto e' strutturato secondo il paradigma Model-View-Controller (o almeno la mia interpretazione).

Il Model contiene tutti i dati dell'applicazione in un'unica `struct`; non ha dipendenze e gestisce soltanto i dati.

La View gestisce tutto cio' che e' correlato al display: e' divisa in pagine che si impilano durante l'esecuzione.
Ogni pagina e' costituita da callback che ne descrivono il ciclo di vita (creazione, apertura, gestione degli eventi, chiusura e distruzione) durante il quale possono mostrare informazioni sullo schermo, modificare il modello o inviare messaggi al Controller in risposta a interazioni dell'utente.

Il Controller gestisce tutto il resto; la colla tra i componenti e l'interazione con l'hardware.

## Notes
