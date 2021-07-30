#include <thread>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <vector>
#include <string.h>

#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

#define PORT 8080
//string fileName = "data.csv"; //old
string fileName = "data.csv";
string path;
pthread_t thread_id;


enum role {SENDER,RECIEVER,NONE};

struct Client{
    struct sockaddr_in address;
    int addrLen;
    int fd;
    string name;
    role client_role;
    string room;
    int coords[2];
    
    Client():client_role(NONE){}

    string getRoleStr(){
        return client_role == SENDER ? "sender" : "reciever";
    }
    
    string toString(){
        return name + " [Luogo: " +room+" ("+ to_string(coords[0]) +" "+ to_string(coords[1])+")]";
    }
};

struct args{
    Client sender;
    Client reciever;
};



int sendtoClient(int cfd,string msg){
    return send(cfd,msg.c_str(),msg.size(),0);
}


vector<string> tokenize(string s, char separator){
    

   // work
   std::istringstream split(s);
   std::vector<std::string> tokens;
   for (std::string each; std::getline(split, each, separator); tokens.push_back(each));

    //cout << "[" << tokens[0] << "|" << tokens[1] << "|" << tokens[2] << "]" << endl;

    return tokens;
}

string verifyUser(string userData)
{
  
    
    // File pointer
    fstream fin;
  
    // Open an existing file
    
    fin.open(path.c_str(), ios::in);
    if (!fin.is_open()){
        std::cout << "Error opening file\n";
    }
  
    bool found = false;
  
    // Read the Data from the file
    // as String Vector
    vector<string> row;
    string line, temp;

    vector<string> refRow = tokenize(userData,';');

  
    while (getline(fin, line)) {
  
        row.clear();
  
        // read an entire row and
        // store it in a string variable 'line'
        
  
        row = tokenize(line,',');
        cout << refRow[0] << " " << refRow[1] << endl;
        cout << row[0] << " " << row[1] << endl;
        fflush(stdout);
  
        // convert string to integer for comparision
        if((row[0] == refRow[0]) && (row[1] == refRow[1])){
            found = true;
            fin.close();
            return row[2]+ " " + row[3]+ " " +row[4]; //Aula x y
        }
    }
    fin.close();
    if(!found) return "";
}

void* subthread(void* arg){

    Client sender = ((args*)arg)->sender;
    Client reciever = ((args*)arg)->reciever;
    cout << "Iniziato l'invio da " << sender.name << " a " << reciever.name << endl;



    return NULL;
}

int main(int argc, char** argv){

    int serverSock = 0, clientSock, valread, opt=1;
    struct sockaddr_in address;
    struct sockaddr_in cAdd;
    int addrlen = sizeof(address);
    int cAddLen;

    Client sender,reciever;

    char buffer[1024] = {0};

    ros::init(argc,argv,"pickdelivery");
    path = ros::package::getPath("pickdelivery") + "/data/" + fileName;
    //cout << path << endl;

    if ((serverSock = socket(AF_INET, SOCK_STREAM, 0)) <= 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    if (setsockopt(serverSock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                  &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(serverSock, (struct sockaddr *)&address, 
                                 sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(serverSock, 2) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    
    string msg;

    while(true){
        if ((clientSock = accept(serverSock, (struct sockaddr *)&cAdd, 
                            (socklen_t*)&cAddLen))<0)
            {
                perror("accept persa");
                continue;
            }
        
        //identifica il ruolo

        cout << "Qualcuno si è connesso! Identificazione in corso...\n";
        fflush(stdout);

        memset(buffer,0,1024);
        valread = read(clientSock , buffer, 1024);
        
        vector<string> clientPosition;
        vector<string> clientData = tokenize(string(buffer),';'); //[nome,pass,role]
        string login = verifyUser(string(buffer));
        //cout << clientData[0] << " " << clientData[1] << " " << clientData[2] << endl;
        //cout << "//" << login << "//";
        fflush(stdout);

        if(login == ""){
            msg="ERR_1";
            sendtoClient(clientSock,msg);
            continue;
        }

        clientPosition = tokenize(login,' ');//[aula,x,y]
        
        //cout << string(buffer) << endl;
        //cout << clientData[0] << " " << clientData[1] << " " << clientData[2] << endl;
        //cout << clientPosition[0] << " " << clientPosition[1] << " " << clientPosition[2] << endl;
        //fflush(stdout);
        
        if(clientData[2]=="s" && sender.client_role==NONE){
            sender.address = cAdd;
            sender.addrLen = cAddLen;
            sender.name = clientData[0];
            sender.client_role = SENDER;
            sender.fd = clientSock;
            sender.room = clientPosition[0];
            sender.coords[0] = stoi(clientPosition[1]);
            sender.coords[1] = stoi(clientPosition[2]);
            
            msg="Connessione al sistema come SENDER avvenuta con successo.";
            sendtoClient(sender.fd,msg);
            cout << "Si è connesso un sender!\n";
            cout << sender.toString() << endl;
            fflush(stdout);

        }else if(clientData[2]=="r" && reciever.client_role==NONE){
            reciever.address = cAdd;
            reciever.addrLen = cAddLen;
            reciever.name = clientData[0];
            reciever.client_role = RECIEVER;
            reciever.fd = clientSock;
            reciever.room = clientPosition[0];
            reciever.coords[0] = stoi(clientPosition[1]);
            reciever.coords[1] = stoi(clientPosition[2]);

            msg="Connessione al sistema come RECIEVER avvenuta con successo.";
            sendtoClient(reciever.fd,msg);
            cout << "Si è connesso un reciever!\n";
            cout << reciever.toString() << endl;
            fflush(stdout);
        }else{
            msg="Il sistema è gia utilizzato con questo ruolo. Riprovare più tardi.";
            sendtoClient(clientSock,msg);
            close(clientSock);
        }


              
        if(sender.client_role!=NONE && reciever.client_role!=NONE){

            cout << "SENDER E RECIEVER TROVATI. Facciamo cose...";
            fflush(stdout);
            //fai roba con il sender e il reciever

            args* arg = (args*)malloc(sizeof(args));
            arg->reciever = reciever;
            arg->sender = sender;

            if((valread = pthread_create(&thread_id,NULL,subthread,(void*)arg))!=0){
                cout << "Error creating thread";
            }
            
            
            if((valread = pthread_join(thread_id,NULL))!=0){
                cout << "Error joining thread";
            }
            cout << "Fatto!\n";
            fflush(stdout);
            

            close(sender.fd); close(reciever.fd);
            free(arg);
            sender = Client();
            reciever = Client();
            continue;
        }
        
    }
    
    return 0;

}