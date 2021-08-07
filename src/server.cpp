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
char buffer[1024] = {0}; int valread;

//ros variables
pthread_t thread_id;
tf2_ros::Buffer tfBuffer;
ros::Publisher pubGoal;
geometry_msgs::PoseStamped goal_msg;
int seq_num = 0;
float freq = 5.0; //frequency of message sent to the client(during robot cruising)


//robot variables
float robot_position[2];

float dist;
string robot_status;
float old_dist = 0;


//the client structure
int sendtoClient(int cfd,string msg){
    return send(cfd,msg.c_str(),msg.size(),0);
}

enum role {SENDER,RECIEVER,NONE};

struct Client{
    struct sockaddr_in address;
    int addrLen;
    int fd;
    string name;
    role client_role;
    float coords[2];
    
    Client():client_role(NONE){}

    string getRoleStr(){
        return client_role == SENDER ? "sender" : "reciever";
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
        if(client_role != NONE) sendtoClient(fd,"The robot is about " + to_string((int)dist) + " meters from you.\n");
        cout << "Sent info\n" << endl;
    }
    
    string toString(){
        return "User: " + name + " [Location: ("+ to_string(coords[0]) +" "+ to_string(coords[1])+")]";
    }
};

Client sender,reciever,server;





vector<string> tokenize(string s, char separator){

    // Tokenize s with the separator chosen
    std::istringstream split(s);
    std::vector<std::string> tokens;
    for (std::string each; std::getline(split, each, separator); tokens.push_back(each));
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
        row = tokenize(line,';');
        fflush(stdout);
  
        // convert string to integer for comparision
        if((row[0] == refRow[0]) && (row[1] == refRow[1])){
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


void* subthread(void* arg){

    string cmd;
    ros::NodeHandle nh;

    cout << "Starting the dispatch from " << sender.name << " to " << reciever.name << endl;

    

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
    sendtoClient(reciever.fd,"The robot has reached " + sender.name + "\n");
    //asking the sender to place the package on the robot
    sendtoClient(sender.fd,"CMD_1");
    memset(buffer,0,1024);
    valread = read(sender.fd, buffer, 1024);
    cmd = string(buffer);


    if(cmd == "KO"){
        cout << "The sender wasn't ready. Let's go home..." << endl;
        fflush(stdout);
        server.callRobot(nh);
        sendtoClient(sender.fd,"You didn't confirm the action in time. The robot will go back home!");
        sendtoClient(reciever.fd,sender.name+" was too slow accepting. The robot is going back home!");
        sendtoClient(sender.fd,"CMD_EXIT");
        sendtoClient(reciever.fd,"CMD_EXIT");
        
        return NULL;

    }else if(cmd != "OK"){
        cout << "Error recieving message from sender!" << endl;
        fflush(stdout);

        sendtoClient(sender.fd,"ERR_MSG");
        sendtoClient(reciever.fd,"ERR_MSG");

        return NULL;
    }

    cout << "The sender is ready!" << endl;
    fflush(stdout);

    sendtoClient(sender.fd,"Sending robot to " + reciever.name + "\n");
    sendtoClient(reciever.fd,"The robot is coming with the package, please wait...\n");

    reciever.callRobot(nh);

    sendtoClient(sender.fd,"The robot has reached" + reciever.name + "\n");
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
        

        server.callRobot(nh);
        
        return NULL;

    }else if(cmd != "KO"){
        cout << "Error recieving message from reciever!" << endl;
        fflush(stdout);

        sendtoClient(sender.fd,"ERR_MSG");
        sendtoClient(reciever.fd,"ERR_MSG");

        return NULL;
    }

    cout << "The sender didn't get the package. I'm sending it back..." << endl;
    fflush(stdout);

    sendtoClient(reciever.fd,"Sending robot back to " + sender.name + "\n");
    sendtoClient(sender.fd,reciever.name + " didn't accept the package. The robot is coming back to you, please wait...\n");

    sender.callRobot(nh);

    sendtoClient(sender.fd,"The robot has reached again" + sender.name + "\n");
    sendtoClient(reciever.fd,"CMD_2");

    memset(buffer,0,1024);
    valread = read(reciever.fd, buffer, 1024);
    cmd = string(buffer);

    if(cmd == "OK"){
        cout << "The sender got the package back! Sending the robot home..." << endl;
    }else if(cmd == "KO"){
        cout << "The sender didn't get the package back! Sending the robot home with the package..." << endl;
    }else{
        cout << "Error recieving message from sender!" << endl;
        fflush(stdout);

        sendtoClient(sender.fd,"ERR_MSG");
        sendtoClient(reciever.fd,"ERR_MSG");

        return NULL;
    }

    fflush(stdout);

    //closing clients
    sendtoClient(sender.fd,"CMD_EXIT");
    sendtoClient(reciever.fd,"CMD_EXIT");

    server.callRobot(nh);

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

    //initializing Client struct for the server
    server.address = address;
    server.addrLen = addrlen;
    server.name = "base";
    server.fd = serverSock;
    

    

    while(true){
        cout << "Waiting for users..." << endl;
        if ((clientSock = accept(serverSock, (struct sockaddr *)&cAdd, 
                            (socklen_t*)&cAddLen))<0)
            {
                perror("accept persa");
                continue;
            }
        
        //identifica il ruolo

        cout << "Someone connected. Starting identification..." << endl;
        fflush(stdout);

        memset(buffer,0,1024);
        valread = read(clientSock , buffer, 1024);
        
        vector<string> clientPosition;
        vector<string> clientData = tokenize(string(buffer),';'); //[nome;pass;role]
        string login = verifyUser(string(buffer));
        
        fflush(stdout);


        //ERR_1: login failed
        if(login.empty()){
            msg="ERR_1";
            sendtoClient(clientSock,msg);
            continue;
        }

        clientPosition = tokenize(login,';');//[x;y]
        
        
        
        if(clientData[2]=="s" && sender.client_role==NONE){
            sender.address = cAdd;
            sender.addrLen = cAddLen;
            sender.name = clientData[0];
            sender.client_role = SENDER;
            sender.fd = clientSock;
            sender.coords[0] = stoi(clientPosition[0]);
            sender.coords[1] = stoi(clientPosition[1]);
            
            msg="Connected succesfully as a SENDER.";
            sendtoClient(sender.fd,msg);
            cout << "The following user is the sender: " << endl;
            cout << sender.toString() << endl;
            fflush(stdout);

        }else if(clientData[2]=="r" && reciever.client_role==NONE){
            reciever.address = cAdd;
            reciever.addrLen = cAddLen;
            reciever.name = clientData[0];
            reciever.client_role = RECIEVER;
            reciever.fd = clientSock;
            reciever.coords[0] = stoi(clientPosition[0]);
            reciever.coords[1] = stoi(clientPosition[1]);

            msg="Connected succesfully as a RECIEVER.";
            sendtoClient(reciever.fd,msg);
            cout << "The following user is the reciever: " << endl;
            cout << reciever.toString() << endl;
            fflush(stdout);

        }else{
            //ERR_2: the role chosen is already used by another user
            msg="ERR_2";
            sendtoClient(clientSock,msg);
            close(clientSock);
        }


              
        if(sender.client_role!=NONE && reciever.client_role!=NONE){

            cout << "Sender and reciever found! Starting..." << endl;
            fflush(stdout);

            //start thread to do things with clients...

            if((valread = pthread_create(&thread_id,NULL,subthread,NULL))!=0){
                cout << "Error creating thread";
            }
            
            if((valread = pthread_join(thread_id,NULL))!=0){
                cout << "Error joining thread";
            }
            cout << "Done!\n";
            fflush(stdout);
            

            close(sender.fd); close(reciever.fd);
            
            sender = Client();
            reciever = Client();
            continue;
        }
        
    }

    close(serverSock);
    
    return 0;

}