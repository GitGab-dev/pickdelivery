#Pick and Delivery(Progetto LABIAGI 2021)

Progetto di Gabriele Iannola(1849732), consistente nel realizzare il "Pick and Delivery" tra due utenti in una mappa data.

Il package consta di due nodi ROS, server e client. Mentre il primo può essere avviato normalmente, per il secondo è necessario specificare il RUOLO scelto.
I due ruoli possibili sono
- sender(s): colui che vuole inviare il pacco attraverso il robot
- reciever(r): colui che vuole ricevere il pacco
Possono essere connessi solo un sender ed un reciever alla volta. Appena il server troverà qualcuno con i ruoli appropriati, inizierà la comunicazione.

USO: $rosrun pickdelivery server
USO: $rosrun pickdelivery client r(eciever) oppure $rosrun pickdelivery client s(ender)

All'avvio del client, verrano chiesti un nome utente e una password, i quali servono fondamentalmente ad identificare la posizione dell'utente nella mappa(si noti il file data.csv presente nella cartella data).

Nel momento in cui il server inizia la comunicazione, questo invia il robot al SENDER(fornendo continuamente informazioni sulla posizione del robot). All'arrivo, viene chiesto all'utente di porre il pacco sul robot, e confermare sul suo client. Alla ricezione di una conferma, il pacco viene inviato al RECIEVER, al quale verrà chiesto poi di confermare l'avvenuta ricezione. Il robot torna infine al punto di partenza(base).


