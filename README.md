# Pick and Delivery (Progetto LABIAGI 2021)

Progetto di Gabriele Iannola(1849732), consistente nel realizzare il "Pick and Delivery" tra due utenti in una mappa data.

## Contenuto del package ed uso

Il package consta di due nodi ROS, server e client. I due nodi possono essere avviati facilmente con i seguenti comandi:

```bash
roscore pickdelivery server

roscore pickdelivery client
```

Possono essere connessi solo un sender ed un reciever alla volta. Appena il server troverà qualcuno con i ruoli appropriati, inizierà la comunicazione tra i due utenti connessi.
Coloro che si connetteranno durante la comunicazione saranno messi in attesa.

### Credenziali per il login

Al momento dell'accesso del client, il sistema richiede all'utente uno Username e una password. Ciò è necessario per la localizzazione dell'utente nella mappa. Esempi di credenziali possono essere 
nel file *data.csv* presente nella cartella **data**. Li riporto di seguito:


| Username  | Password |
| ------------- | ------------- |
| marco  | pass1  |
| giovanni  | pass2  |

E' inoltre necessario specificare il ruolo scelto per la comunicazione.I due ruoli possibili sono:
- SENDER: colui che vuole inviare il pacco attraverso il robot al reciever
- RECIEVER: colui che vuole ricevere il pacco dal sender

## Requisiti

Sono ovviamente richiesti i package srrg2_labiagi per il funzionamento del sistema.


