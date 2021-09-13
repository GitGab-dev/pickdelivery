# Pick and Delivery (Progetto LABIAGI 2021)

Progetto di Gabriele Iannola(1849732), consistente nel realizzare il "Pick and Delivery" tra due utenti in una mappa data.

## Contenuto del package ed uso

Il package consta di due nodi ROS, server e client. I due nodi possono essere avviati facilmente con i seguenti comandi:

```bash
roscore pickdelivery server

roscore pickdelivery client
```


### Credenziali per il login

Al momento dell'accesso al client, il sistema richiederà all'utente uno Username e una password. Ciò è necessario per la localizzazione dell'utente nella mappa. Esempi di credenziali possono essere 
nel file *data.csv* presente nella cartella *data*, riportati di seguito:


| Username  | Password |
| ------------- | ------------- |
| marco  | pass1  |
| giovanni  | pass2  |
| luca  | pass3  |

Si possono liberamente altre voci al file, seguendo sempre la struttura USERNAME;PASSWORD;X;Y, dove X e Y sono le coordinate in cui si desidera ricevere il robot.

### Funzionamento generale

Effettuato il login, l'utente può scegliere tra tre opzioni:
- inviare un pacco a qualcuno
- ricevere un pacco da qualcuno
- fare il logout

Nei primi due casi sarà inoltre necessario specificare a/da chi inviare/ricevere il pacco.

L'utente sarà quindi messo in coda, in attesa che il robot sia utilizzabile (si noti che è necessario che entrambi gli utenti interessati siano in coda affinchè inizi la comunicazione tra i due).

Durante il viaggio del robot l'utente dovrà inoltre confermare l'avvenuta consegna e ricezione del pacco.ù

Terminato il viaggio, l'utente può effettuare una nuova scelta, e il robot torna alla sua base per una nuova missione.

## Requisiti

Il sistema è stato costruito per un sistema Linux, pertanto potrebbe non funzionare su altre piattaforme.
Inoltre è necessario aver scaricato ROS e i package srrg2_labiagi del corso.


