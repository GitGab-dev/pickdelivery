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
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

#define PORT 8080
//string fileName = "data.csv"; //old
string fileName = "data.csv";
string path;
char buffer[1024] = {0}; int valread;


pthread_t thread_id;
tf2_ros::Buffer tfBuffer;
ros::Publisher pubGoal;

geometry_msgs::PoseStamped goal_msg;
int seq_num = 0;

float robot_start_position[2];
float robot_position[2];
float freq = 5.0; //frequenza di invio dei messaggi sulla pos. del robot attuale
float old_distance = 0;



enum role {SENDER,RECIEVER,NONE};

struct Client{
    struct sockaddr_in address;
    int addrLen;
    int fd;
    string name;
    role client_role;
    string room;
    float coords[2];
    
    Client():client_role(NONE){}

    string getRoleStr(){
        return client_role == SENDER ? "sender" : "reciever";
    }

    void callRobot(){
        
        ros::Time temp;

        cout << "Inviando il robot a " + this->name;

        goal_msg.header.seq = seq_num;
        seq_num++;

        temp = ros::Time::now();
        while(!(temp.isValid())){
            //cout << "Aspettando che si attivi il clock\n";
            temp = ros::Time::now();
        };
        goal_msg.header.stamp = temp;
        goal_msg.header.frame_id = "map";

        goal_msg.pose.position.x = coords[0];
        goal_msg.pose.position.y = coords[1];
        goal_msg.pose.position.z = 0;
        
        goal_msg.pose.orientation.x = 0;
        goal_msg.pose.orientation.y = 0;
        goal_msg.pose.orientation.z = 0;
        goal_msg.pose.orientation.w = 0;

        pubGoal.publish(goal_msg);
        cout << "GOAL INVIATO\n";
        fflush(stdout);
    }
        
    string toString(){
        return name + " [Luogo: " +room+" ("+ to_string(coords[0]) +" "+ to_string(coords[1])+")]";
    }
};

struct args{
    Client sender;
    Client reciever;
};

Client sender,reciever;



int sendtoClient(int cfd,string msg){
    return send(cfd,msg.c_str(),msg.size(),0);
}

float distance(float* c1,float* c2){
    float res = sqrt(pow(c1[0]-c2[0],2) + pow(c1[0]-c2[0],2));
    return res;
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
            return row[2]+ ";" + row[3]+ ";" +row[4]; //Aula x y
        }
    }
    fin.close();
    if(!found) return "";
}


void tfCallBack(const tf2_msgs::TFMessage& tf){

    int transform_ok = tfBuffer.canTransform("map","base_link",ros::Time(0));

    if(transform_ok){
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map","base_link",ros::Time(0));

        robot_position[0] = transformStamped.transform.translation.x;
        robot_position[1] = transformStamped.transform.translation.y;
        //cout <<"[["<< robot_position[0] << robot_position[1] <<"]]"<< endl;
        if(robot_start_position[0]==0 && robot_start_position[1]==0){
            robot_start_position[0] = robot_position[0];
            robot_start_position[1] = robot_position[1];
        }
        //cout << "Location saved!";
    }
    else{
        cout << "Error transforming!!";
    }
}

void sendRobotHome(){

        ros::Time temp;
        goal_msg.header.seq = seq_num;
        seq_num++;

        temp = ros::Time::now();
        while(!(temp.isValid())){
            //cout << "Aspettando che si attivi il clock\n";
            temp = ros::Time::now();
        };
        goal_msg.header.stamp = temp;
        goal_msg.header.frame_id = "map";

        //cout <<"[["<< robot_start_position[0] << robot_start_position[0] <<"]]"<< endl;
        goal_msg.pose.position.x = robot_start_position[0];
        goal_msg.pose.position.y = robot_start_position[1];
        goal_msg.pose.position.z = 0;
        
        goal_msg.pose.orientation.x = 0;
        goal_msg.pose.orientation.y = 0;
        goal_msg.pose.orientation.z = 0;
        goal_msg.pose.orientation.w = 0;

        pubGoal.publish(goal_msg);

        fflush(stdout);
}

void timer1Callback(const ros::TimerEvent& e){
    float dist = distance(robot_position,sender.coords);
    
    if(dist == old_distance){
        cout << "Il robot non si muove... riinvio il goal!" << endl;
        sender.callRobot();
    } 
    old_distance = dist;
    sendtoClient(sender.fd,"Il robot si trova a circa " + to_string((int)dist) + " metri da te.\n");
    cout << "Sent info\n" << endl;
}

void timer2Callback(const ros::TimerEvent& e){
    float dist = distance(robot_position,reciever.coords);
    
    if(dist == old_distance){
        cout << "Il robot non si muove... riinvio il goal!" << endl;
        reciever.callRobot();
    } 
    old_distance = dist;
    sendtoClient(reciever.fd,"Il robot si trova a circa " + to_string((int)dist) + " metri da te.\n");
    cout << "Sent info\n" << endl;
}

void* subthread(void* arg){

    bool onLoop = true;
    /*
    Client sender = ((args*)arg)->sender;
    Client reciever = ((args*)arg)->reciever;*/
    cout << "Iniziato l'invio da " << sender.name << " a " << reciever.name << endl;

    ros::NodeHandle nh;

    pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Subscriber sub_tf = nh.subscribe("tf",1000,tfCallBack);

    //invia il robot al sender, assieme ad alcune info
    sendtoClient(reciever.fd,"Sto inviando il robot a " + sender.name + "\n");
    sendtoClient(sender.fd,"Il robot è in arrivo, attendi...\n");

    //salvo lo stato iniziale
    
    robot_start_position[0] = 0;
    robot_start_position[1] = 0;
    ros::Rate loopRate(1);


    sender.callRobot();

    //set a timer to sent info on robot position to the sender
    ros::Timer timer1 = nh.createTimer(ros::Duration(freq),timer1Callback); 

    while(onLoop){
        ros::spinOnce();
        //pubGoal.publish(goal_msg);
        if(distance(robot_position,sender.coords) < 0.5) break;

        loopRate.sleep();
    }

    timer1.stop();

    sendtoClient(reciever.fd,"Il robot è arrivato da " + sender.name + "\n");
    sendtoClient(sender.fd,"CMD_1");

    memset(buffer,0,1024);
    valread = read(sender.fd, buffer, 1024);

    cout << "Ricevuta conferma dal sender.\n";
    fflush(stdout);


    

    sendtoClient(sender.fd,"Sto inviando il robot a " + reciever.name + "\n");
    sendtoClient(reciever.fd,"Il robot è in arrivo con il pacco, attendi...\n");

    reciever.callRobot();

    //set a timer to sent info on robot position to the sender
    ros::Timer timer2= nh.createTimer(ros::Duration(freq),timer2Callback); 

    while(onLoop){
        ros::spinOnce();
        //pubGoal.publish(goal_msg);
        if(distance(robot_position,reciever.coords) < 1.5) break;

        loopRate.sleep();
    }

    timer2.stop();

    sendtoClient(sender.fd,"Il robot è arrivato da " + reciever.name + "\n");
    sendtoClient(reciever.fd,"CMD_2");

    memset(buffer,0,1024);
    valread = read(reciever.fd, buffer, 1024);

    cout << "Ricevuta conferma dal reciever. Il robot torna alla base\n";
    fflush(stdout);

    sendtoClient(sender.fd,"CMD_EXIT");
    sendtoClient(reciever.fd,"CMD_EXIT");

    

    while(onLoop){
        ros::spinOnce();
        //pubGoal.publish(goal_msg);
        sendRobotHome();
        if(distance(robot_position,robot_start_position) < 1.5) break;

        loopRate.sleep();
    }
    
    return NULL;
}

int main(int argc, char** argv){

    int serverSock = 0, clientSock, opt=1;
    struct sockaddr_in address;
    struct sockaddr_in cAdd;
    int addrlen = sizeof(address);
    int cAddLen;


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
        cout << "Accettando nuovi utenti..." << endl;
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

        clientPosition = tokenize(login,';');//[aula,x,y]
        
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

            /*
            args* arg = (args*)malloc(sizeof(args));
            arg->reciever = reciever;
            arg->sender = sender;

            if((valread = pthread_create(&thread_id,NULL,subthread,(void*)arg))!=0){
                cout << "Error creating thread";
            }*/
            if((valread = pthread_create(&thread_id,NULL,subthread,NULL))!=0){
                cout << "Error creating thread";
            }
            
            if((valread = pthread_join(thread_id,NULL))!=0){
                cout << "Error joining thread";
            }
            cout << "Fatto!\n";
            fflush(stdout);
            

            close(sender.fd); close(reciever.fd);
            //free(arg);
            sender = Client();
            reciever = Client();
            continue;
        }
        
    }
    
    return 0;

}