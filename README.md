# Pick and Delivery (Progetto LABIAGI 2021)

Progetto di Gabriele Iannola(1849732), consistente nel realizzare il "Pick and Delivery" tra due utenti in una mappa data.

## Contenuto del package ed uso

Il package consta di due nodi ROS, server e client. Il server può essere semplicemente avviato con

```bash
roscore pickdelivery server
```
Per il client è invece necessario specificare il RUOLO scelto.

I due ruoli possibili sono:
- sender(s): colui che vuole inviare il pacco attraverso il robot al reciever
- reciever(r): colui che vuole ricevere il pacco dal sender

```bash
#su due terminali distinti, avvia il sender
roscore pickdelivery client s

#e il reciever
roscore pickdelivery client r
```

Possono essere connessi solo un sender ed un reciever alla volta. Appena il server troverà qualcuno con i ruoli appropriati, inizierà la comunicazione tra i due utenti connessi.

### Credenziali per il login

Al momento dell'accesso del client, il sistema richiede all'utente uno Username e una password. Ciò è necessario per la localizzazione dell'utente nella mappa. Esempi di credenziali possono essere 
nel file *data.csv* presente nella cartella **data**. Li riporto di seguito:


USERNAME PASSWORD
marco pass1
giovanni pass2


| First Header  | Second Header |
| ------------- | ------------- |
| Content Cell  | Content Cell  |
| Content Cell  | Content Cell  |

## Requisiti

Sono ovviamente richiesti i package srrg2_labiagi per il funzionamento del sistema.


