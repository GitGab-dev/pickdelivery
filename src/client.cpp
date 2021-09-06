#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <signal.h>
using namespace std;


#define PORT 8080

string user_id;
pthread_t tid;
string confirm = "KO";
int T = 10; //timeout(in seconds)


string login(){
    string data("--LOGIN;");
    string temp;

    //data += (user_id + ";");
    
    
    cout << "Username: ";
    cin >> temp;
    data += (temp+";");
    cout << "Password: ";
    cin >> temp;

    
    data += temp;
    /*
    cout << "\nPlease select one of the following option (selecting any other number will close the system):\n" << endl;
    cout << "1. I want to SEND a package to someone." << endl;
    cout << "2. I want to RECIEVE a package." << endl;
    cout << "\n> ";
    cin >> temp;
    cout << "\n";
    if(temp!="1" && temp!="2") return string("EXIT");
    
    if(temp=="1"){
        data+="s;";
        cout << "\n Please specify the username of the reciever: ";
        cin >> temp;
        data+=temp;
    }else data+="r";*/
    
    return data;
}

string get_action(){

    string temp;

    cout << "\nPlease select one of the following option:\n" << endl;
    cout << "0. LOGOUT." << endl;
    cout << "1. SEND a package to someone." << endl;
    cout << "2. RECIEVE a package." << endl;
    cout << "\n> ";
    cin >> temp;
    cout << "\n";
    if(temp!="0" && temp!="1" && temp!="2") return string("");
    
    if(temp=="1"){ //sender
        cout << "\n Please specify the username of the reciever: ";
        cin >> temp;
        return "--JOIN;0;" + temp; 
    }else if(temp=="2"){ //reciever
        cout << "\n Please specify the username of the reciever: ";
        cin >> temp;

        return "--JOIN;1;" + temp;
    }else{//logout
        return "--LOGOUT";
    }
    
}

void* confirmation(void *arg)
{
    while(true){
        cin >> confirm;
        if(confirm == "OK"){
            cout << "Sending confirm to the server..." << endl;
            break;
        }else cout << "Write OK: ";
    }

    return nullptr;
}

void alarm_handler(int a)
{
    if(confirm != "OK"){
        cout << "\nYou took too much time!" <<endl;
        pthread_cancel(tid);    // terminate thread
    }   
}

int main(int argc, char** argv){

    int sock = 0, valread;
    struct sockaddr_in serv_addr;
    string presentation, msg, action;
    

    char buffer[1024] = {0};

    //connecting to server
    cout << "Connecting to server...\n";
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

    memset(buffer,0,1024);

    /*
    valread = read( sock , buffer, 1024);
    
    user_id = string(buffer);
    cout << "Assigned to client n° " << user_id << endl;*/
    cout << "Welcome to the Pick and Delivery system. Please login.\n" << endl;
    fflush(stdout);

    while(true){
        presentation = login();
        
        //Sending info to server
        
        send(sock , presentation.c_str() , presentation.size() , 0 );


        memset(buffer,0,1024);
        valread = read( sock , buffer, 1024);
        msg = string(buffer);

        if(msg == "ERR_1"){//LOGIN FAIL
            cout << "ERROR: login failed. Check your username and password and retry.\n";
            continue;
        }else{
            cout << "You have logged in successfully. Welcome to the system.\n";
            break;
        }
    }
    
    while(true){

        action = "";
        while(action == "") action = get_action();
        send(sock , action.c_str() , action.size() , 0 );

        if(action == "--LOGOUT") break;

        //Communication start
        while(true){
            memset(buffer,0,1024);
            valread = read( sock , buffer, 1024);
            msg = string(buffer);
            
            if(msg.back() == '\n') msg.pop_back();
            //cout << "[" << msg << "]" << endl;

            if(msg == "CMD_1"){ //Put the package

                confirm = "KO";
                cout << "The robot arrived. Please put your package on the robot.\nWrite OK to send the package(you have 30 seconds to do so): ";
                pthread_create(&tid, nullptr, confirmation, nullptr);

                signal(SIGALRM, alarm_handler);
                alarm(T);   // Run alarm_handler after 3 seconds, and terminate thread in it

                pthread_join(tid, nullptr); // Wait for thread finish
                send(sock , confirm.c_str() , confirm.size() , 0 );
                
                
            }else if(msg == "CMD_2"){ //Take the package

                confirm = "KO";
                cout << "The robot arrived. Please take your package from the robot.\nWrite OK to send the robot back(you have 30 seconds to do so): ";
                
                pthread_create(&tid, nullptr, confirmation, nullptr);

                signal(SIGALRM, alarm_handler);
                alarm(T);   // Run alarm_handler after 3 seconds, and terminate thread in it

                pthread_join(tid, nullptr); // Wait for thread finish
                send(sock , confirm.c_str() , confirm.size() , 0 );

            }else if(msg == "CMD_EXIT"){
                cout << "The dispatch has been a success! The robot is going home now." << endl;
                break;
            }else if(msg == "MSG_ERR"){//some message had been recieved wrongly
                cout << "Something went wrong with the communication! Abort!" << endl;
                break;
            }else{
                cout << msg << endl;
            }      
        }

    }

    cout << "Thank you for using the Pick and Delivery system!" << endl;
    close(sock);

    return 0;
}