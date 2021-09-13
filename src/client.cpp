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
int zone = 0; //variable to keep track of the user action(0 = LOGIN, 1 = ACTION, 2 = QUEUE, 3 = DISPATCH)

enum Color {
    RED      = 31,
    GREEN    = 32,
    YELLOW = 33,
    BLUE     = 34,
    DEFAULT  = 39
};   

string coloring(string text,Color c, bool bright = false){
    string opt = (bright ? ";1m" : "m");
    return "\033[" + to_string(c) + opt + text + "\033[0m";
}


string serviceTag = "[" + coloring("PICKDELIVERY",Color::BLUE, true) + "] ";
string infoTag = "[" + coloring("INFO",Color::YELLOW, true) + "] ";
string errorTag = "[" + coloring("ERROR",Color::RED, true) + "] ";

void my_handler(int s)
{
    string exit_cmd;
    switch(zone){
        case 0:{
            cout << "\n" << infoTag << "You are leaving the system...\n";
            exit_cmd = "--DISCONNECT";
            send(sock, exit_cmd.c_str(), exit_cmd.size(), 0);
            sleep(3);
            close(sock);
            exit(1);
        }
        case 1:{
            cout << "\n" << infoTag << "You are leaving the system...\n";
            exit_cmd = "--LOGOUT";
            send(sock, exit_cmd.c_str(), exit_cmd.size(), 0);
            sleep(3);
            close(sock);
            exit(1);
        }
        case 2:{
            cout << "\n" << infoTag << "You are leaving the queue, please wait...\n";
            string exit_cmd = "--LEAVE";
            send(sock, exit_cmd.c_str(), exit_cmd.size(), 0);
            sleep(3);
            break;
        }
        case 3:{
            
            cout << "\n" << infoTag << "You are leaving the system...\n";
            string exit_cmd = "--FORCE";
            send(sock, exit_cmd.c_str(), exit_cmd.size(), 0);
            sleep(3);
            close(sock);
            exit(1);

        }
    }
    
    
}

string login()
{
    string data("--LOGIN;");
    string temp;

    cout << "──────────────────────────────" << endl;
    cout << "Username: ";
    getline(std::cin, temp);
    data += (temp + ";");
    cout << "Password: ";
    getline(std::cin, temp);
    cout << "──────────────────────────────" << endl;

    data += temp;

    return data;
}

string get_action()
{

    string temp = "vuoto", check = "";

    cout << "\n" << serviceTag << "Please select one of the following option:\n"
         << endl;
    cout << "┌──────────────────────────────┐" << endl;
    cout << "|0. LOGOUT.                    |" << endl;
    cout << "|1. SEND a package to someone. |" << endl;
    cout << "|2. RECIEVE a package.         |" << endl;
    cout << "└──────────────────────────────┘" << endl;
    cout << "\n> ";
    getline(std::cin, temp);

    if (temp != "0" && temp != "1" && temp != "2")
    {
        cout << "\n" << infoTag << "Cannot accept an option '" << temp << "'\n\n";
        return string("");
    }

    if (temp == "1")
    { //sender
        cout << "\n" << serviceTag << "Please specify the username of the reciever: ";
        getline(std::cin, temp);

        while (check != "Y" && check != "N")
        {
            cout << "\n" << serviceTag << "Are you sure to sent a package to " << temp << "? (Y/N) ";
            getline(std::cin, check);
        }
        return check == "Y" ? "--JOIN;0;" + temp : "";
    }
    else if (temp == "2")
    { //reciever

        cout << "\n" << serviceTag << "Please specify the username of the sender: ";
        getline(std::cin, temp);
        while (check != "Y" && check != "N")
        {
            cout << "\n" << serviceTag << "Are you sure to recieve a package from " << temp << "? (Y/N) ";
            getline(std::cin, check);
        }
        return check == "Y" ? "--JOIN;1;" + temp : "";
    }
    else
    { //logout
        while (check != "Y" && check != "N")
        {
            cout << "\n" << serviceTag << "Are you sure you want to logout? (Y/N) ";
            getline(std::cin, check);
        }
        return check == "Y" ? "--LOGOUT" : "";
    }
}

string timedInput(int seconds)
{

    string res = "KO";
    fd_set s_rd;
    struct timeval tv;
    tv.tv_sec = seconds;
    tv.tv_usec = 0;

    FD_ZERO(&s_rd);
    FD_SET(fileno(stdin), &s_rd);

    select(fileno(stdin) + 1, &s_rd, NULL, NULL, &tv);
    if (FD_ISSET(fileno(stdin), &s_rd))
        getline(std::cin, res);
    else
        cout << "\n" << infoTag << "You took too much time!\n";

    return res;
}

int main(int argc, char **argv)
{

    int valread;
    struct sockaddr_in serv_addr;
    string presentation, msg, action;

    char buffer[1024] = {0};

    //connecting to server
    cout << infoTag <<"Connecting to server...\n";
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        cout << errorTag << "\n Socket creation error \n";
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0)
    {
        cout << errorTag << "\nInvalid address/ Address not supported \n";
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        cout << "\n" << errorTag <<"Connection Failed \n";
        return -1;
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    cout << "\n" << serviceTag << "Welcome to the Pick and Delivery system. Please login.\n"
         << endl;
    fflush(stdout);

    while (true)
    {
        presentation = login();

        //Sending info to server

        send(sock, presentation.c_str(), presentation.size(), 0);

        memset(buffer, 0, 1024);
        valread = read(sock, buffer, 1024);
        msg = string(buffer);

        if (msg == "ERR_1")
        { //LOGIN FAIL
            cout << "\n" << errorTag << "Login failed. Check your username and password and retry.\n\n";
            continue;
        }
        else if (msg == "ERR_2")
        { //ALREADY LOGGED IN
            cout << "\n" << errorTag << "You are already logged on the server. Retry later...\n\n";
            exit(EXIT_FAILURE);
        }else
        {
            cout << "\n" << serviceTag << "You have logged in successfully. Welcome to the system.\n";
            break;
        }
    }

    while (true)
    {
        zone = 1;
        action = "";
        while (action.empty())
            action = get_action();
        send(sock, action.c_str(), action.size(), 0);

        if (action == "--LOGOUT")
            break;

        memset(buffer, 0, 1024); //successfully in queue message
        valread = read(sock, buffer, 1024);
        msg = string(buffer);

        cout << "\n" << serviceTag << msg;
        cout << "\n" << serviceTag << "You can always leave the queue by pressing CTRL+C.\n";

        zone = 2;    

        msg = "";
        while(msg == ""){ //consuming empty message
            memset(buffer, 0, 1024);
            valread = read(sock, buffer, 1024);
            msg = string(buffer);
        }

        
        if(msg == "--STOP"){
            cout << "\n" << infoTag << "You left the queue.\n";
            continue;
        }
        
        zone = 3; 

        //Communication start
        while (true)
        {
            memset(buffer, 0, 1024);
            valread = read(sock, buffer, 1024);
            msg = string(buffer);    

            if (msg == "CMD_1")
            { //Put the package

                //confirm = "KO";
                cout << "\n" << serviceTag << "The robot arrived. Please put your package on the robot.\nWrite OK to send the package(you have 30 seconds to do so): ";
                fflush(stdout);

                confirm = timedInput(T);

                send(sock, confirm.c_str(), confirm.size(), 0);
                
            }
            else if (msg == "CMD_2")
            { //Take the package

                //confirm = "KO";
                cout << "\n" << serviceTag << "The robot arrived. Please take your package from the robot.\nWrite OK to send the robot back(you have 30 seconds to do so): ";
                fflush(stdout);

                confirm = timedInput(T);
                send(sock, confirm.c_str(), confirm.size(), 0);
                
            }
            else if (msg == "CMD_EXIT")
            {
                cout << "\n" << serviceTag << "The dispatch has concluded! The robot is going home now." << endl;
                break;
            }
            else if (msg == "MSG_ERR" || msg == "MSG_ERR\n")
            { //some message had been recieved wrongly
                cout << "\n" << errorTag << "Something went wrong with the communication! Abort!" << endl;
                break;
            }
            else if (msg == "USER_NOT_FOUND")
            { //when you want to reach someone who doesn't exist
                cout << "\n" << infoTag << "This user doesn't exist. Please check the username." << endl;
                break;
            }
            else
            {
                cout << serviceTag << msg << endl;
            }
        }
    }

    cout << serviceTag << "Thank you for using the Pick and Delivery system!" << endl;
    close(sock);

    return 0;
}