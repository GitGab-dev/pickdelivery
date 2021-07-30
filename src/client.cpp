//#include <thread>
#include <iostream>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
//#include <string.h>

using namespace std;


#define PORT 8080

string login(string role){
    string data;
    string temp;
    cout << "Stai per accedere al sistema come " << (role=="s"?"SENDER":"RECIEVER") << endl;
    cout << "Username: ";
    cin >> temp;
    data = (temp+";");
    cout << "Password: ";
    cin >> temp;
    data += (temp+";");
    return data + role; //username;password;role
}

int main(int argc, char** argv){

    if(argc < 2){
        cerr << "USAGE: client [s(ender):r(eciever)]";
        exit(-1);
    }
    string role = (argv[1] == "sender" ? "s" : (argv[1] == "reciever" ? "r" : argv[1] ));
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    string presentation, msg;

    char buffer[1024] = {0};

    while(true){
        presentation = login(role);
        //string presentation = role;

        
        

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            printf("\n Socket creation error \n");
            return -1;
        }
    
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        
        // Convert IPv4 and IPv6 addresses from text to binary form
        if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
        {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }
    
        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            printf("\nConnection Failed \n");
            return -1;
        }

        //Sending info to server
        //cout << presentation.c_str();
        
        send(sock , presentation.c_str() , presentation.size() , 0 );

        valread = read( sock , buffer, 1024);
        msg = string(buffer);

        if(msg == "ERR_1"){//LOGIN FAIL
            cout << "ERRORE: login fallito. Controllare le credenziali e riprovare.\n";
            continue;
        }else if(msg == "ERR_2"){//SYSTEM ALREADY IN USE
            cout << "ERRORE: il sistema è al momento in uso da un altro utente con lo stesso ruolo. Riprovare più tardi.\n";
            return 0;
        }else break;
    }
    printf("%s\n",buffer );


    //Da qui inizia la comunicazione
    valread = read( sock , buffer, 1024);

    return 0;
}