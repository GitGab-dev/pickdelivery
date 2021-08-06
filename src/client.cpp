//#include <thread>
#include <iostream>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

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

    if(argc < 2 || !(strcmp(argv[1],"r")==0 && strcmp(argv[1],"s")==0 && strcmp(argv[1],"reciever")==0 && strcmp(argv[1],"sender")==0)){
        cerr << "USAGE: client [s(ender):r(eciever)]";
        exit(-1);
    }
    string role = (argv[1] == "sender" ? "s" : (argv[1] == "reciever" ? "r" : argv[1] ));
    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    string presentation, msg;
    string confirm;

    char buffer[1024] = {0};

    while(true){
        presentation = login(role);
        //string presentation = role;

        
        

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
        {
            cout << "\n Socket creation error \n";
            return -1;
        }
    
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);
        
        // Convert IPv4 and IPv6 addresses from text to binary form
        if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0) 
        {
            cout << "\nInvalid address/ Address not supported \n";
            return -1;
        }
    
        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            cout << "\nConnection Failed \n";
            return -1;
        }

        //Sending info to server
        //cout << presentation.c_str();
        
        send(sock , presentation.c_str() , presentation.size() , 0 );

        valread = read( sock , buffer, 1024);
        msg = string(buffer);

        if(msg == "ERR_1"){//LOGIN FAIL
            cout << "ERROR: login failed. Check your username and password and retry.\n";
            continue;
        }else if(msg == "ERR_2"){//SYSTEM ALREADY IN USE
            cout << "ERROR: the system is already in use by someone else with this role. Retry later.\n";
            return 0;
        }else break;
    }
    std::printf("%s\n",buffer );


    //Da qui inizia la comunicazione
    while(true){
        memset(buffer,0,1024);
        valread = read( sock , buffer, 1024);
        msg = string(buffer);
        if(msg.back() == '\n') msg.pop_back();

        //cout << "||" << msg <<"||";

        if(msg == "CMD_1"){ //E' stato chiesto di mettere il pacco
            
            cout << "Il robot è arrivato. Metti il pacco nell'apposito spazio nei prossimi \
             60 secondi.\nScrivi OK quando sei pronto: ";
            
            while(true){
                cin >> confirm;
                if(confirm == "OK"){
                    cout << "Invio la risposta al server...";
                    send(sock , confirm.c_str() , confirm.size() , 0 );
                    break;
                }else cout << "Scrivi OK: ";
            }
        }else if(msg == "CMD_2"){ //E' stato chiesto di prendere il pacco
            cout << "Il robot è arrivato. Metti il pacco nell'apposito spazio nei prossimi \
             60 secondi.\nScrivi OK quando sei pronto: ";
            
            while(true){
                cin >> confirm;
                if(confirm == "OK"){
                    cout << "Invio la risposta al server...";
                    send(sock , confirm.c_str() , confirm.size() , 0 );
                    break;
                }else cout << "Scrivi OK: ";
            }
        }else if(msg == "CMD_EXIT"){
            cout << "Il viaggio si è concluso con successo. Il robot torna alla base!\n";
            break;
        }else{
            std::printf("%s\n",buffer );
        }

        
    }

    close(sock);

    return 0;
}