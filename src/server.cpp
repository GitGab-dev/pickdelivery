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
float robot_start_position[2];
float robot_position[2];

float dist;
string robot_status;
float old_dist = 0;


//the client structure
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

    void callRobot(){
        
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
        
    string toString(){
        return "User: " + name + " [Location: ("+ to_string(coords[0]) +" "+ to_string(coords[1])+")]";
    }
};

Client sender,reciever;



int sendtoClient(int cfd,string msg){
    return send(cfd,msg.c_str(),msg.size(),0);
}

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
        
        if(robot_start_position[0]==0 && robot_start_position[1]==0){
            robot_start_position[0] = robot_position[0];
            robot_start_position[1] = robot_position[1];
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

void sendRobotHome(){

        ros::Time temp;
        goal_msg.header.seq = seq_num;
        seq_num++;

        temp = ros::Time::now();
        while(!(temp.isValid())){
            temp = ros::Time::now();
        };
        cout << "Clock is working!\n";

        goal_msg.header.stamp = temp;
        goal_msg.header.frame_id = "map";

        
        goal_msg.pose.position.x = robot_start_position[0];
        goal_msg.pose.position.y = robot_start_position[1];
        goal_msg.pose.position.z = 0;
        
        goal_msg.pose.orientation.x = 0;
        goal_msg.pose.orientation.y = 0;
        goal_msg.pose.orientation.z = 0;
        goal_msg.pose.orientation.w = 0;

        pubGoal.publish(goal_msg);
        cout << "Goal sent!\n";

        fflush(stdout);
}

void timer1Callback(const ros::TimerEvent& e){
        
    if(dist == old_dist){
        cout << "Il robot non si muove... riinvio il goal!" << endl;
        sender.callRobot();
    } 
    old_dist = dist;
    sendtoClient(sender.fd,"Il robot si trova a circa " + to_string((int)dist) + " metri da te.\n");
    cout << "Sent info\n" << endl;
}

void timer2Callback(const ros::TimerEvent& e){
        
    if(dist == old_dist){
        cout << "Il robot non si muove... riinvio il goal!" << endl;
        reciever.callRobot();
    } 
    old_dist = dist;
    sendtoClient(reciever.fd,"Il robot si trova a circa " + to_string((int)dist) + " metri da te.\n");
    cout << "Sent info\n" << endl;
}

void timer3Callback(const ros::TimerEvent& e){
        
    if(dist == old_dist){
        cout << "Il robot non si muove... riinvio il goal!" << endl;
        sendRobotHome();
    } 
    old_dist = dist;
    cout << "Il robot si trova a circa " + to_string((int)dist) + " metri da te.\n";
}

void* subthread(void* arg){

    cout << "Starting the dispatch from " << sender.name << " to " << reciever.name << endl;

    ros::NodeHandle nh;

    pubGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Subscriber sub_tf = nh.subscribe("tf",1000,tfCallback);
    ros::Subscriber sub_planner = nh.subscribe("planner_status",1000,plannerCallback);

    //send robot to SENDER, wtih some robot info
    sendtoClient(reciever.fd,"Sending robot to " + sender.name + "\n");
    sendtoClient(sender.fd,"The robot is coming, please wait...\n");

    //saving initial position of the robot
    
    robot_start_position[0] = 0;
    robot_start_position[1] = 0;
    ros::Rate loopRate(1);


    sender.callRobot();

    //set a timer to sent info on robot position to the sender
    ros::Timer timer1 = nh.createTimer(ros::Duration(freq),timer1Callback);
     

    while(robot_status != GLOBAL_PLANNING && robot_status != CRUISING){
        ros::spinOnce();
        loopRate.sleep();
    }
    while(true){
        ros::spinOnce();
        if(robot_status == GOAL_REACHED) break;
        loopRate.sleep();
    }

    timer1.stop();



    sendtoClient(reciever.fd,"The robot has reached " + sender.name + "\n");

    //asking the sender to place the package on the robot
    sendtoClient(sender.fd,"CMD_1");
    memset(buffer,0,1024);
    valread = read(sender.fd, buffer, 1024);

    cout << "The sender is ready!" << endl;
    fflush(stdout);


    

    sendtoClient(sender.fd,"Sending robot to " + reciever.name + "\n");
    sendtoClient(reciever.fd,"The robot is coming with the package, please wait...\n");

    reciever.callRobot();

    //set a timer to sent info on robot position to the sender
    ros::Timer timer2= nh.createTimer(ros::Duration(freq),timer2Callback); 

    while(robot_status != GLOBAL_PLANNING && robot_status != CRUISING){
        ros::spinOnce();
        loopRate.sleep();
    }
    while(true){
        ros::spinOnce();
        if(robot_status == GOAL_REACHED) break;
        loopRate.sleep();
    }

    timer2.stop();

    sendtoClient(sender.fd,"The robot has reached" + reciever.name + "\n");
    sendtoClient(reciever.fd,"CMD_2");

    memset(buffer,0,1024);
    valread = read(reciever.fd, buffer, 1024);

    cout << "The reciever got the package! Sending the robot home..." << endl;
    fflush(stdout);

    //closing clients
    sendtoClient(sender.fd,"CMD_EXIT");
    sendtoClient(reciever.fd,"CMD_EXIT");

    sendRobotHome();

    ros::Timer timer3= nh.createTimer(ros::Duration(freq),timer3Callback); 
    
    while(robot_status != GLOBAL_PLANNING && robot_status != CRUISING){
        ros::spinOnce();
        loopRate.sleep();
    }
    while(true){
        ros::spinOnce();
        if(robot_status == GOAL_REACHED) break;
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