#include <thread>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <vector>
#include <string.h>
#include <queue>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <srrg2_core_ros/PlannerStatusMessage.h>

using namespace std;

#define PORT 8080

//macros to define robot states
#define GOAL_REACHED "GoalReached"
#define CRUISING "Cruising"
#define GLOBAL_PLANNING "GlobalPlanning"

//input utilities
string fileName = "data.csv";
string path;

int max_sd;
int currentSender, currentReciever;
fd_set currentsd, readsd;
pthread_t commsId, selectorId;
struct timeval tv;

//ros variables

tf2_ros::Buffer tfBuffer;
ros::Publisher pubGoal;
geometry_msgs::PoseStamped goal_msg;
int seq_num = 0;
float freq = 5.0; //frequency of message sent to the client(during robot cruising)


//robot variables
bool robot_in_use = false;
float robot_position[2];

float dist;
string robot_status;
float old_dist = 0;


//the client structure
int sendtoClient(int cfd,string msg){
    return send(cfd,msg.c_str(),msg.size(),0);
}

struct Client{
    int fd;
    string name;
    string interest = "";
    float coords[2];
    int role = 0; //0 none, 1 sender, 2 reciever

    Client(){}
    
    Client(int fd_, string name_, float x, float y){
        fd = fd_;
        name = name_;
        coords[0] = x;
        coords[1] = y;
    }


    void callRobot(ros::NodeHandle& nh){
        sendGoal();

        //set a timer to sent info on robot position to the sender
        ros::Rate loopRate(1);
        ros::Timer timer = nh.createTimer(ros::Duration(freq),std::bind(&Client::timerCallback, this));

        while(robot_status != GLOBAL_PLANNING && robot_status != CRUISING){
            ros::spinOnce();
            loopRate.sleep();
        }
        while(true){
            ros::spinOnce();
            if(robot_status == GOAL_REACHED) break;
            loopRate.sleep();
        }

        timer.stop();
    }

    void sendGoal(){
        
        ros::Time temp;

        cout << "Sending robot to " + this->name << endl;

        goal_msg.header.seq = seq_num;
        seq_num++;

        temp = ros::Time::now();
        while(!(temp.isValid())){
            temp = ros::Time::now();
        };
        cout << "Clock is working!\n";

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
        cout << "Goal sent!\n";
        fflush(stdout);
    }

    void timerCallback(){
        if(dist == old_dist){
            cout << "The robot stopped. Goal sent again!" << endl;
            sendGoal();
        } 
        old_dist = dist;
        
        if(dist >= 1) sendtoClient(fd,"The robot is about " + to_string((int)dist) + " meters from you.\n");
        cout << "Sent info\n" << endl;
        
    }
    
    string toString(){
        return "User: " + name + " [Location: ("+ to_string(coords[0]) +" "+ to_string(coords[1])+")]";
    }
};

Client server;
std::queue<Client*> sendersQueue;
std::unordered_map<std::string,Client*> recieversMap;
std::unordered_map<int,Client*> loggedUsers;




vector<string> tokenize(string s, char separator){

    // Tokenize s with the separator chosen
    std::istringstream split(s);
    std::vector<std::string> tokens;
    for (std::string each; std::getline(split, each, separator); tokens.push_back(each));
    return tokens;

}

bool isUserOnline(string userName){
    if(loggedUsers.size() > 0){
        for(auto user : loggedUsers){
            if(user.second->name == userName) return true;
        } 
    }
    return false;
}

bool existUser(string userName){

    fstream fin;
      
    fin.open(path.c_str(), ios::in);
    if (!fin.is_open()){
        std::cout << "Error opening file\n";
    }
    
    vector<string> row;
    string line;
   
  
    while (getline(fin, line)) {
  
        row.clear();  
        row = tokenize(line,';');
        fflush(stdout);
  
        if(row[0] == userName){
            fin.close();
            return true;
        }
    }
    fin.close();
    return false;
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

    vector<string> row;
    string line;

    vector<string> refRow = tokenize(userData,';'); // [--LOGIN,name,password]

    //is the user already online?
    if(isUserOnline(refRow[1])) return "";

    
  
    while (getline(fin, line)) {
  
        row.clear();  
        row = tokenize(line,';');
        fflush(stdout);
  
        if((row[0] == refRow[1]) && (row[1] == refRow[2])){
            found = true;
            fin.close();
            return row[2]+ ";" + row[3]; //x;y
        }
    }
    fin.close();
    if(!found) return "";
}


void tfCallback(const tf2_msgs::TFMessage& tf){

    int transform_ok = tfBuffer.canTransform("map","base_link",ros::Time(0));

    if(transform_ok){
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("map","base_link",ros::Time(0));

        robot_position[0] = transformStamped.transform.translation.x;
        robot_position[1] = transformStamped.transform.translation.y;
        
        if(server.coords[0]==0 && server.coords[1]==0){
            server.coords[0] = robot_position[0];
            server.coords[1] = robot_position[1];
        }
    }
    else{
        cout << "Error transforming!!";
    }
}

void plannerCallback(const srrg2_core_ros::PlannerStatusMessage& msg){
    robot_status = msg.status;
    dist = msg.distance_to_global_goal;
}

void* commsThread(void* arg){

    string cmd;
    ros::NodeHandle nh;
    Client sender,reciever;
    char buffer[1024] = {0};
    int valread;

    cout << "[COMMS] Thread started!\n";

    while(true){

        while(true){
            if(!sendersQueue.empty() && !recieversMap.empty()){
                sender = *sendersQueue.front();
                if(sender.role == 0){ //Sender left while in queue
                    sendersQueue.pop();
                    continue;
                }
                std::unordered_map<std::string,Client*>::const_iterator match = recieversMap.find(sender.interest);
                if (match != recieversMap.end() && match->second->interest == sender.name){
                    cout << "[COMMS] Match found!\n"; 
                    reciever = *(match->second);
                    sendersQueue.pop();
                    recieversMap.erase(reciever.name);

                    currentSender = sender.fd;
                    currentReciever = reciever.fd;

                    break;
                }
                else{
                    cout << "[COMMS] Match missed!\n"; 
                    sendersQueue.push(sendersQueue.front());
                    sendersQueue.pop();
                }
            }
            sleep(1);
        }

        
        robot_in_use = true;
        cout << "[COMMS] Starting the dispatch from " << sender.name << " to " << reciever.name << endl;        

        pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        
        tf2_ros::TransformListener tfListener(tfBuffer);

        ros::Subscriber sub_tf = nh.subscribe("tf",1000,tfCallback);
        ros::Subscriber sub_planner = nh.subscribe("planner_status",1000,plannerCallback);

        //send robot to SENDER, wtih some robot info
        sendtoClient(reciever.fd,"Sending robot to " + sender.name + "\n");
        sendtoClient(sender.fd,"The robot is coming, please wait...\n");

        //saving initial position of the robot
        
        server.coords[0] = 0;
        server.coords[1] = 0;
        

        sender.callRobot(nh);
        sleep(2);
        sendtoClient(reciever.fd,"The robot has reached " + sender.name + "\n");
        //asking the sender to place the package on the robot
        sendtoClient(sender.fd,"CMD_1");
        memset(buffer,0,1024);
        valread = read(sender.fd, buffer, 1024);
        cmd = string(buffer);


        if(cmd == "KO"){
            cout << "The sender wasn't ready. Let's go home..." << endl;
            fflush(stdout);
            sendtoClient(sender.fd,"You didn't confirm the action in time. The robot will go back home!");
            sendtoClient(reciever.fd,sender.name+" was too slow accepting. The robot is going back home!");
            sleep(2);
            sendtoClient(sender.fd,"CMD_EXIT");
            sendtoClient(reciever.fd,"CMD_EXIT");

            currentSender = -1;
            currentReciever = -1;
            server.callRobot(nh);
            robot_in_use = false;
            cout << "[COMMS] The communication is over!\n";
            continue;

        }else if(cmd != "OK"){
            cout << "Error recieving message from sender!" << endl;
            fflush(stdout);

            sendtoClient(sender.fd,"MSG_ERR");
            sendtoClient(reciever.fd,"MSG_ERR");

            currentSender = -1;
            currentReciever = -1;
            server.callRobot(nh);
            robot_in_use = false;
            cout << "[COMMS] The communication is over!\n";
            continue;
        }

        cout << "[COMMS] The sender is ready!" << endl;
        fflush(stdout);

        sendtoClient(sender.fd,"Sending robot to " + reciever.name + "\n");
        sendtoClient(reciever.fd,"The robot is coming with the package, please wait...\n");

        reciever.callRobot(nh);
        sleep(2);
        sendtoClient(sender.fd,"The robot has reached " + reciever.name + "\n");
        sendtoClient(reciever.fd,"CMD_2");

        memset(buffer,0,1024);
        valread = read(reciever.fd, buffer, 1024);
        cmd = string(buffer);

        if(cmd == "OK"){
            cout << "The reciever got the package! Sending the robot home..." << endl;
            fflush(stdout);

            //closing clients
            sendtoClient(sender.fd,"CMD_EXIT");
            sendtoClient(reciever.fd,"CMD_EXIT");
            
            currentSender = -1;
            currentReciever = -1;
            server.callRobot(nh);
            robot_in_use = false;
            cout << "[COMMS] The communication is over!\n";
            continue;

        }else if(cmd != "KO"){
            cout << "Error recieving message from reciever!" << endl;
            fflush(stdout);

            sendtoClient(sender.fd,"MSG_ERR");
            sendtoClient(reciever.fd,"MSG_ERR");


            currentSender = -1;
            currentReciever = -1;
            server.callRobot(nh);
            robot_in_use = false;
            cout << "[COMMS] The communication is over!\n";
            continue;
        }

        cout << "[COMMS] The reciever didn't get the package. I'm sending it back..." << endl;
        fflush(stdout);

        sendtoClient(reciever.fd,"Sending robot back to " + sender.name + "\n");
        sleep(1);
        sendtoClient(reciever.fd,"CMD_EXIT");

        currentReciever = -1;
        cout << "[COMMS] The reciever is free!\n";

        sendtoClient(sender.fd,reciever.name + " didn't accept the package. The robot is coming back to you, please wait...\n");

        sender.callRobot(nh);
        sleep(2);
        sendtoClient(sender.fd,"CMD_2");

        memset(buffer,0,1024);
        valread = read(sender.fd, buffer, 1024);
        cmd = string(buffer);

        if(cmd == "OK"){
            cout << "The sender got the package back! Sending the robot home..." << endl;
            fflush(stdout);
            sendtoClient(sender.fd,"Thank you for getting the package!\n");
        }else if(cmd == "KO"){
            cout << "The sender didn't get the package back! Sending the robot home with the package..." << endl;
            fflush(stdout);
            sendtoClient(sender.fd,"You didn't get the package. The robot is bringing it home, you can get it there. Thank you.\n");
        }else{
            cout << "Error recieving message from sender!" << endl;
            fflush(stdout);

            currentSender = -1;
            currentReciever = -1;
            server.callRobot(nh);
            robot_in_use = false;
            cout << "[COMMS] The communication is over!\n";
            continue;
        }

        sleep(2);

        //closing clients
        sendtoClient(sender.fd,"CMD_EXIT");

        currentSender = -1;
        server.callRobot(nh);
        robot_in_use = false;
        cout << "[COMMS] The communication is over!\n";

    }

    return NULL;
}

void* selectorThread(void* arg){

    int activity,valread;
    char buffer[1024] = {0};
    string msg;

    vector<string> clientMsg;

    //login variables
    vector<string> clientPosition;
    Client* tempClient;


    cout << "[SELECTOR] Thread started!\n";

    while(true){
        readsd = currentsd;

        tv.tv_sec = 2;
        tv.tv_usec = 0;
        activity = select( max_sd + 1 , &readsd , NULL , NULL , &tv);
          
        
        if ((activity < 0) && (errno!=EINTR))  
        {  
            printf("[SELECTOR] select error");  
        }
        for(int i = 0; i< max_sd+1; i++){
            if(i!=currentSender && i!=currentReciever && FD_ISSET(i,&readsd)){
                memset(buffer,0,1024);
                valread = read(i , buffer, 1024);
                if(valread == 0) continue;
                clientMsg = tokenize(string(buffer),';'); //[--cmd;(args...)]
                tempClient = nullptr;

                cout << "[SELECTOR] Recieved command from client " << i << " : " << string(buffer) << endl;


                if(clientMsg[0] == string("--LOGIN")){ //[--LOGIN;name;pass]
                    string login = verifyUser(string(buffer));
                    
                    if(login.empty()){//ERR_1: login failed
                        msg="ERR_1";
                        sendtoClient(i,msg);
                        continue;
                    }else{//Create client

                        clientPosition = tokenize(login,';');
                        tempClient = new Client(i,clientMsg[1],stoi(clientPosition[0]),
                            stoi(clientPosition[1]));

                        loggedUsers.insert({i,tempClient});
                        
                        msg = "OK";
                        sendtoClient(i,msg);                      
                    }                           
                
                }else if(clientMsg[0] == string("--JOIN")){ //[--JOIN;action;interest]

                    tempClient = loggedUsers.find(i)->second;
                    if(!existUser(clientMsg[2])){
                        msg="USER_NOT_FOUND";
                        sendtoClient(tempClient->fd,msg); 
                        continue;
                    }
                    tempClient->interest = clientMsg[2];
                    if(clientMsg[1]=="0"){ //sender
                        tempClient->role = 1;
                        sendersQueue.push(tempClient);
                        msg="Request accepted. Waiting in queue...";
                        sendtoClient(tempClient->fd,msg);
                        cout << "[SELECTOR] The following user is a sender to "<<clientMsg[2]<<": " << endl;
                        cout << (*tempClient).toString() << endl;

                    }else{ //reciever
                        tempClient->role = 2;
                        recieversMap.insert({tempClient->name,tempClient});
                        msg="Request accepted. Waiting for the sender...";
                        sendtoClient(tempClient->fd,msg);
                        cout << "[SELECTOR] The following user is a reciever to "<<clientMsg[2]<<": " << endl;
                        cout << (*tempClient).toString() << endl;
                        
                    }
                
                }else if(clientMsg[0] == string("--LOGOUT")){

                    tempClient = loggedUsers.find(i)->second;
                    cout << "[SELECTOR] The user "<<tempClient->name<<" has logged out!" << endl;
                    close(i);
                    delete(tempClient);
                    loggedUsers.erase(i);
                    FD_CLR(i,&currentsd);
                    

                }else if(clientMsg[0] == string("--FORCE")){

                    tempClient = loggedUsers.find(i)->second;
                    cout << "[SELECTOR] The user "<<tempClient->name<<" has left the system forcibly!" << endl;
                    close(i);          
                    if(tempClient->role == 2) recieversMap.erase(tempClient->name);
                    else tempClient->role = 0;
                    delete(tempClient);
                    loggedUsers.erase(i);
                    FD_CLR(i,&currentsd);
                }                            
            }
        }
        
    }
    
}

int main(int argc, char** argv){

    int serverSock = 0, clientSock, opt=1, valread;
    char buffer[1024] = {0};
    
    struct sockaddr_in address;
    struct sockaddr_in cAdd;
    int addrlen = sizeof(address);
    int cAddLen;

    Client tempClient;
    

    ros::init(argc,argv,"pickdelivery");
    path = ros::package::getPath("pickdelivery") + "/data/" + fileName;
    FD_ZERO(&readsd);

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

    //initializing Client struct for the server
    server = Client(serverSock,"base",0,0);
    
    //creating comms thread

    
    if((valread = pthread_create(&commsId,NULL,commsThread,NULL))!=0){
        cout << "[MAIN] Error creating thread";
    }
    
    if((valread = pthread_detach(commsId))!=0){
        cout << "[MAIN] Error detatching thread";
    }

    
    if((valread = pthread_create(&selectorId,NULL,selectorThread,NULL))!=0){
        cout << "[MAIN] Error creating thread";
    }
    
    if((valread = pthread_detach(selectorId))!=0){
        cout << "[MAIN] Error detatching thread";
    }
    

    while(true){
        cout << "[MAIN] Waiting for users..." << endl;
        if ((clientSock = accept(serverSock, (struct sockaddr *)&cAdd, 
                            (socklen_t*)&cAddLen))<0)
            {
                perror("[MAIN] accept lost");
                continue;
            }
        
        //identifica il ruolo

        cout << "[MAIN] Someone connected. Starting identification..." << endl;
        FD_SET(clientSock, &currentsd);  
        max_sd = clientSock;
        
    }

    close(serverSock);
    
    return 0;

}