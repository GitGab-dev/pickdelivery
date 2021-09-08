#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <signal.h>
#include <sys/time.h>

using namespace std;


#define PORT 8080

string user_id;
string confirm = "KO";
int sock = 0;
int T = 10; //timeout(in seconds)

void my_handler(int s){
    printf("[INFO] You are leaving the system...\n",s);
    string exit_cmd = "--FORCE";
    send(sock , exit_cmd.c_str() , exit_cmd.size() , 0 );
    sleep(5);
    close(sock);
    exit(1); 
}

string login(){
    string data("--LOGIN;");
    string temp;

    cout << "Username: ";
    getline(std::cin, temp);
    data += (temp+";");
    cout << "Password: ";
    getline(std::cin, temp);

    
    data += temp;
        
    return data;
}

string get_action(){

    string temp="vuoto", check="";

    cout << "\nPlease select one of the following option:\n" << endl;
    cout << "0. LOGOUT." << endl;
    cout << "1. SEND a package to someone." << endl;
    cout << "2. RECIEVE a package." << endl;
    cout << "\n> ";
    getline(std::cin, temp);

    if(temp!="0" && temp!="1" && temp!="2"){
        cout << "\nCannot accept an option '" << temp <<"'\n\n"; 
        return string("");
    } 
    
    if(temp=="1"){ //sender
        cout << "\n Please specify the username of the reciever: ";
        getline(std::cin, temp);

        while(check!="Y" && check!="N"){
            cout << "\n\n Are you sure to sent a package to " << temp << "? (Y/N) ";
            getline(std::cin, check);
        }
        return check=="Y" ? "--JOIN;0;" + temp : ""; 
    }else if(temp=="2"){ //reciever

        cout << "\n Please specify the username of the sender: ";
        getline(std::cin, temp);
        while(check!="Y" && check!="N"){
            cout << "\n\n Are you sure to recieve a package from " << temp << "? (Y/N) ";
            getline(std::cin, check);
        }
        return check=="Y" ? "--JOIN;1;" + temp : ""; 

    }else{//logout
        while(check!="Y" && check!="N"){
            cout << "\n\n Are you sure you want to logout? (Y/N) ";
            getline(std::cin, check);
        }
        return check=="Y" ? "--LOGOUT" : "";
    }
    
}

string timedInput(int seconds){

    string res = "KO";
    fd_set s_rd;
    struct timeval tv;
    tv.tv_sec = seconds;
    tv.tv_usec = 0;

    FD_ZERO(&s_rd);
    FD_SET(fileno(stdin), &s_rd);

    select(fileno(stdin)+1, &s_rd, NULL, NULL, &tv);
    if(FD_ISSET(fileno(stdin),&s_rd)) getline(std::cin,res);
    else cout << "\nYou took too much time!\n";

    return res;
}

int main(int argc, char** argv){

    int valread;
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

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

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
        while(action.empty()) action = get_action();
        send(sock , action.c_str() , action.size() , 0 );

        if(action == "--LOGOUT") break;

        cout << "You can always leave the queue by pressing CTRL+C.\n";

        //Communication start
        while(true){
            memset(buffer,0,1024);
            valread = read( sock , buffer, 1024);
            msg = string(buffer);
            
            if(msg.back() == '\n') msg.pop_back();
            //cout << "[" << msg << "]" << endl;

            if(msg == "CMD_1"){ //Put the package

                //confirm = "KO";
                cout << "The robot arrived. Please put your package on the robot.\nWrite OK to send the package(you have 30 seconds to do so): ";
                fflush(stdout);

                confirm = timedInput(T);
                
                send(sock , confirm.c_str() , confirm.size() , 0 );
                cout << endl;
                
            }else if(msg == "CMD_2"){ //Take the package

                //confirm = "KO";
                cout << "The robot arrived. Please take your package from the robot.\nWrite OK to send the robot back(you have 30 seconds to do so): ";
                fflush(stdout);
                
                confirm = timedInput(T);
                send(sock , confirm.c_str() , confirm.size() , 0 );
                cout << endl;

            }else if(msg == "CMD_EXIT"){
                cout << "The dispatch has concluded! The robot is going home now." << endl;
                break;
            }else if(msg == "MSG_ERR"){//some message had been recieved wrongly
                cout << "Something went wrong with the communication! Abort!" << endl;
                break;
            }else if(msg == "USER_NOT_FOUND"){//when you want to reach someone who doesn't exist
                cout << "This user doesn't exist. Please check the username." << endl;
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