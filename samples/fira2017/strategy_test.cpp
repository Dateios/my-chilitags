// 机器人正对球的角度 - 180
// double angleDiff =  getAngle(ourRobotOne.g_position, ourRobotOne.front, b_position) - 180.0f;

// 机器人与球之间的距离
// double rob2ball = getDistance(ourRobotOne.g_position, b_position);

// 获取目标点
// cv::Point ball_target = getTarget(outputImage);

// 机器人、球、目标点之间的夹角 - 180
// double angleTargetDiff = getAngle(b_position, ourRobotOne.g_position, ball_target) - 180.0f;

// std::cout <<  obstacle_One.g_position << " " << obstacle_Two.g_position << std::endl;
// std::cout <<  rob2ball << std::endl;
// std::cout <<  angleDiff << "  " << angleTargetDiff << "  " << rob2ball << std::endl;


// std::cout << dir0[12][5] << std::endl;
/*
int situation = 4; 
int robot_num = 0;
switch(situation)
{
    case 1:
    {
        
        static bool kicktime = false;
        if(rob2ball >= 15 && !kicktime)
        {
            if(angleDiff > -175.0f && angleDiff <= 0)
            {
                std::cout << "turn right" << std::endl;
                send(robot_num , 6 , 1);          
            }
            else if(angleDiff < 175.0f && angleDiff >= 0)
            {
                std::cout << "turn left" << std::endl;
                send(robot_num , 5 , 1); 
            }
            else
            {
                std::cout << "stop" << std::endl;
                send(robot_num , 1 , 1);
            }
        }
        else
        {

            if(angleTargetDiff >= -180.0f && angleTargetDiff <= -5.0f && !kicktime)
            {
                std::cout << "move right" << std::endl;
                send(robot_num , 9 , 1);
            }
            else if(angleTargetDiff <= 180.0f && angleTargetDiff >= 5.0f && !kicktime)
            {
                std::cout << "move left" << std::endl;
                send(robot_num , 7 , 1);
            }
            else
            {
                if(angleDiff > -175.0f && angleDiff <= 0 && !kicktime)
                {
                    std::cout << "--- turn right" << std::endl;
                    send(robot_num , 6 , 1);          
                }
                else if(angleDiff < 175.0f && angleDiff >= 0 && !kicktime)
                {
                    std::cout << "--- turn left" << std::endl;
                    send(robot_num , 5 , 1); 
                }
                else
                {
                    kicktime = true;
                }
            }

            if(rob2ball >= 10)
            {
                std::cout << "--- go ahead ---" << std::endl;
                send(robot_num , 11 , 2);
            }
            else
            {
                kicktime = false;
            }
        }
        
        break;
    }
    case 2:
    {
        // 情景二
        // 1.调整到正方向
        cv::Point2f posit(ourRobotOne.position.x + 10.0f , ourRobotOne.position.y);
        double positDiff =  getAngle(ourRobotOne.position, ourRobotOne.front, posit) - 180.0f;
        // std::cout << positDiff << std::endl;
        if(positDiff > -175.0f && positDiff <= 0)
        {
            std::cout << "turn right" << std::endl;
            send(0 , 5 , 1);          
        }
        else if(positDiff < 175.0f && positDiff >= 0)
        {
            std::cout << "turn left" << std::endl;
            send(0 , 4 , 1); 
        }
        else
        {
            // 左右平移正对小球
            // std::cout << "direction right" << std::endl;
            if(ourRobotOne.position.y > b_center.y + 5.0f)
            {
                std::cout << "left move" << std::endl;
                send(0 , 6 , 1); 
            }
            else if(ourRobotOne.position.y < b_center.y - 5.0f)
            {
                std::cout << "right move" << std::endl;
                send(0 , 8 , 1); 
            }
            else
            {
                double dx = ourRobotOne.position.x - b_center.x;
                double dy = ourRobotOne.position.y - b_center.y;
                double distance = sqrt(dx * dx + dy * dy);
                std::cout << distance << std::endl;
                if(distance < 25)
                {
                    std::cout << "kkk...kick!!!" << std::endl;
                    send(0 , 13 , 1); 
                }
                else
                {
                    std::cout << "go ahead" << std::endl;
                    send(0 , 0 , 1);
                }
            }
        }
        
        break;
    }
    case 3:
    {
        // 情景三
        float angle_r2b = getAngle(ourRobotOne.g_position, ourRobotOne.front, b_position);
        
        static bool kicktime = false;
        static bool kick = false;
        std::cout <<  angleDiff  << "  " << angleTargetDiff << "  " << angle_r2b << "  " << rob2ball << "  " << kicktime << "  " << kick << std::endl;
        
        // std::cout << b_position << std::endl;
        if(rob2ball >= 15 && !kicktime)
        {
            if(angleDiff > -175.0f && angleDiff <= 0)
            {
                std::cout << "turn right" << std::endl;
                send(robot_num , 4 , 2);          
            }
            else if(angleDiff < 175.0f && angleDiff >= 0)
            {
                std::cout << "turn left" << std::endl;
                send(robot_num , 3 , 2); 
            }
            else
            {
                std::cout << "go ahead" << std::endl;
                send(robot_num , 11 , 2);
            }
        }
        else
        {

            if(angleTargetDiff >= -180.0f && angleTargetDiff <= -5.0f && !kicktime)
            {
                std::cout << "move right" << std::endl;
                send(robot_num , 6 , 2);
            }
            else if(angleTargetDiff <= 180.0f && angleTargetDiff >= 5.0f && !kicktime)
            {
                std::cout << "move left" << std::endl;
                send(robot_num , 5 , 2);
            }
            else
            {
                if(angle_r2b > 95.0f && angle_r2b <= 270 && !kicktime)
                {
                    std::cout << "??????turn right" << std::endl;
                    send(robot_num , 4 , 2);          
                }
                else if(angle_r2b < 85.0f || angle_r2b > 270.0f && !kicktime)
                {
                    std::cout << "??????turn left" << std::endl;
                    send(robot_num , 3 , 2); 
                }
                else
                {
                    send(robot_num , 0 , 2);
                    kicktime = true;
                }

                
                if(rob2ball >= 10)
                {
                    std::cout << "--- go ahead ---" << std::endl;
                    send(robot_num , 11 , 2);
                }
                else
                {
                    kicktime = false;
                }*/

                /*
                if(rob2ball >= 10 && !kick)
                {
                    std::cout << "--- go ahead ---" << std::endl;
                    // send(robot_num , 11 , 2);
                }

                
                if(!kick)
                {
                    if(angleDiff > -175.0f && angleDiff <= 0 )
                    {
                        std::cout << "--- turn right" << std::endl;
                        // send(robot_num , 4 , 2);          
                    }
                    else if(angleDiff < 175.0f && angleDiff >= 0 )
                    {
                        std::cout << "--- turn left" << std::endl;
                        // send(robot_num , 3 , 2); 
                    }
                }

                if(angleDiff > 0)
                {
                    kick = true;    
                    std::cout << "--- left kick ---" << std::endl;
                    send(robot_num , 7 , 2);//左踢
                }
                else if(angleDiff < 0)
                {
                    kick = true;    
                    std::cout << "--- right kick ---" << std::endl;
                    send(robot_num , 8 , 2);//右踢
                } 
                else
                {
                    kick = false;
                }
                
                
                // kicktime = false;
            }
        }
        
        if(kicktime)
        {
            // if(rob2ball >= 10)
            // {
            //     kicktime = false;
            // }
            // else
            // {
                if(angleDiff > 160.0f && !kick)
                {
                    std::cout << "--- left kick ---" << std::endl;
                    send(robot_num , 7 , 2);//左踢
                }
                else if(angleDiff < -160.0f && !kick)
                {  
                    std::cout << "--- right kick ---" << std::endl;
                    send(robot_num , 8 , 2);//右踢
                }
                else
                {
                    // send(robot_num , 0 , 2);
                    // kick = true;
                }
            // }
        }
        


        break;
    }
    default:
    {
        // static cv::Point lastball = b_position;
        
        // std::cout << "test" << std::endl;
        // send(1 , 11 , 2);
        // 踢球测试
        // send(robot_num , 16 , 2);
    }
}*/