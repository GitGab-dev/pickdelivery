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


//RICEVI ED INVIA MESSAGGI

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

//INVIA E RICEVI ANCORA

    sendRobotHome();
    //parametrizzare callRobot()

    ros::Timer timer3= nh.createTimer(ros::Duration(freq),timer3Callback);
    //ros::Timer timerX= nh.createTimer(ros::Duration(freq),timer3Callback);
    
    while(robot_status != GLOBAL_PLANNING && robot_status != CRUISING){
        ros::spinOnce();
        loopRate.sleep();
    }
    while(true){
        ros::spinOnce();
        if(robot_status == GOAL_REACHED) break;
        loopRate.sleep();
    }

    timer3.stop();