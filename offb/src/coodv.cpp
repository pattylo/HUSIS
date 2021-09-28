#include "include/movement.h"
#include <offb/obj.h>
#include <offb/objs.h>
#include <offb/frame.h>
#include <sensor_msgs/NavSatFix.h>

static vector<offb::obj> devices;
static bool all_obj_found = false;

void obj_cb(const offb::objsConstPtr &msg)
{
    devices = msg->points;
    if (devices.size() == 8)
    {
        all_obj_found = true;
    }
}

static mavros_msgs::State current_state_0, current_state_1, current_state_2;

void state_callback_0(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_0 = *msg;
}

void state_callback_1(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_1 = *msg;
}

void state_callback_2(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_2 = *msg;
}

static waypts uavinfo0, uavinfo1, uavinfo2;

void pose_0_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uavinfo0.x = msg->pose.position.x;
    uavinfo0.y = msg->pose.position.y;
    uavinfo0.z = msg->pose.position.z;
}

void pose_1_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uavinfo1.x = msg->pose.position.x;
    uavinfo1.y = msg->pose.position.y;
    uavinfo1.z = msg->pose.position.z;
}

void pose_2_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    uavinfo2.x = msg->pose.position.x;
    uavinfo2.y = msg->pose.position.y;
    uavinfo2.z = msg->pose.position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coordinate");
    ros::NodeHandle nh;

    ros::Subscriber obj_sub = nh.subscribe<offb::objs>
                                  ("/cities", 1, obj_cb);

    ros::Subscriber sub_pos_0 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav0/mavros/local_position/pose", 1, pose_0_cb);
    ros::Subscriber sub_pos_1 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav1/mavros/local_position/pose", 1, pose_1_cb);
    ros::Subscriber sub_pos_2 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav2/mavros/local_position/pose", 1, pose_2_cb);



    ros::Subscriber sub_state_0 = nh.subscribe<mavros_msgs::State>
                                ("/uav0/mavros/state", 1, state_callback_0);
    ros::Subscriber sub_state_1 = nh.subscribe<mavros_msgs::State>
                                ("/uav1/mavros/state", 1, state_callback_1);
    ros::Subscriber sub_state_2 = nh.subscribe<mavros_msgs::State>
                                ("/uav2/mavros/state", 1, state_callback_2);

    ros::Publisher pub_ag0_route = nh.advertise<offb::objs>
                                  ("/uav0/route", 1);
    ros::Publisher pub_ag1_route = nh.advertise<offb::objs>
                                  ("/uav1/route", 1);
    ros::Publisher pub_ag2_route = nh.advertise<offb::objs>
                                  ("/uav2/route", 1);

//    ros::Publisher pub_ag0_dFrame = nh.advertise<offb::frame>
//                                  ("/uav0/frame", 1);
//    ros::Publisher pub_ag1_dFrame = nh.advertise<offb::frame>
//                                  ("/uav1/frame", 1);
//    ros::Publisher pub_ag2_dFrame = nh.advertise<offb::frame>
//                                  ("/uav2/frame", 1);

    ros::Rate rate(20);
    while(ros::ok() && !current_state_0.connected && !current_state_1.connected && !current_state_2.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 0; i < 80 ; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic> route_ag0, route_ag1, route_ag2;
    bool calculated = false;
    offb::objs uav0, uav1, uav2;

    int assign_no_0, assign_no_1, assign_no_2;



    int temp_i = 0;

    ros::spinOnce();
    rate.sleep();

//    cv::Mat show1(1200, 1200, CV_8UC3, CV_RGB(200,200,0));
//    cv::Mat show2(1200, 1200, CV_8UC3, CV_RGB(200,200,0));
//    cv::Mat show3(1200, 1200, CV_8UC3, CV_RGB(200,200,0));

    while(ros::ok())
    {
        //add coordinate transformation into uavinfo
        if(all_obj_found)
        {
            int temp = int(devices.size());

            // the switch determines the number of each agent's inspection points
            switch(temp % 3)
            {           
            case 1:
            {
                assign_no_0 = temp/3 + 1;
                assign_no_1 = temp/3;
                assign_no_2 = temp/3;
                break;
            }
            case 2:
            {
                assign_no_0 = temp/3 + 1;
                assign_no_1 = temp/3 + 1;
                assign_no_2 = temp/3;
                break;
            }
            default:
            {
                assign_no_0 = temp/3;
                assign_no_1 = temp/3;
                assign_no_2 = temp/3;
            }
            }

            vector<MOT> insp_ags;


            //add frame differences
            for(int i = 0; i < assign_no_0; i++)
            {
                MOT temp;
                temp.pt_w = Eigen::Vector4d(uavinfo0.x, uavinfo0.y, uavinfo0.z, 1);
                insp_ags.push_back(temp);
            }

            for(int i = 0; i < assign_no_1; i++)
            {
                MOT temp;
                temp.pt_w = Eigen::Vector4d(uavinfo1.x, uavinfo1.y, uavinfo0.z, 1);
                insp_ags.push_back(temp);
            }

            for(int i = 0; i < assign_no_2; i++)
            {
                MOT temp;                
                temp.pt_w = Eigen::Vector4d(uavinfo2.x, uavinfo2.y, uavinfo0.z, 1);
                insp_ags.push_back(temp);
            }

            vector<MOT> insp_wps;
            for(auto insp : devices)
            {
                MOT temp;
                temp.pt_w = Eigen::Vector4d(insp.X_c, insp.Y_c, insp.Z_c, 1);
                insp_wps.push_back(temp);
            }

            //now we have the devices points and the UAV current position
            //-> we throw everything into the Hungarian algo

            vector<Match> result;
            if(insp_ags.size() == devices.size())
            {
                assign::hungarian distrib;
                result = distrib.solution(insp_ags, insp_wps);
            }

            vector<waypts> agent_0_insp, agent_1_insp, agent_2_insp;
//            for (auto what : result)//check ha here
            for(int i = 0; i < int(result.size()); i++)
            {
                if(i < assign_no_0)
                {
                    waypts temp;
                    temp = {insp_wps[size_t(result[size_t(i)].id)].pt_w(0),
                            insp_wps[size_t(result[size_t(i)].id)].pt_w(1),
                            insp_wps[size_t(result[size_t(i)].id)].pt_w(2)};
                    agent_0_insp.push_back(temp);
                }
                else if(i >= assign_no_0 && i < assign_no_0 + assign_no_1)
                {
                    waypts temp;
                    temp = {insp_wps[size_t(result[size_t(i)].id)].pt_w(0),
                           insp_wps[size_t(result[size_t(i)].id)].pt_w(1),
                           insp_wps[size_t(result[size_t(i)].id)].pt_w(2)};
                    agent_1_insp.push_back(temp);
                }
                else if (i >= assign_no_0 + assign_no_1)
                {
                    waypts temp;
                    temp = {insp_wps[size_t(result[size_t(i)].id)].pt_w(0),
                           insp_wps[size_t(result[size_t(i)].id)].pt_w(1),
                           insp_wps[size_t(result[size_t(i)].id)].pt_w(2)};
                    agent_2_insp.push_back(temp);
                }
            }
            //now we know agent 0,1,2 and their corresponding mission waypoints
            //now we conduct ACOTSP


            //yet we need waypts datatype to do ACOTSP

            //we also add the agents' current position to the cities list
            waypts wp0_now, wp1_now, wp2_now;

            wp0_now = {uavinfo0.x, uavinfo0.y, 2};
            agent_0_insp.push_back(wp0_now);

            wp1_now = {uavinfo1.x, uavinfo1.y, 2};
            agent_1_insp.push_back(wp1_now);

            wp2_now = {uavinfo2.x, uavinfo2.y, 2};
            agent_2_insp.push_back(wp2_now);

            int current_city_0 = int(agent_0_insp.size() - 1) ,
                    current_city_1 = int(agent_1_insp.size() - 1),
                    current_city_2 = int(agent_2_insp.size() - 1);

            //then we conduct ACOTSP
            assign::mTSP agent0(agent_0_insp, 20);
            assign::mTSP agent1(agent_1_insp, 20);
            assign::mTSP agent2(agent_2_insp, 20);
            agent0.execute();
            agent1.execute();
            agent2.execute();
            route_ag0 = agent0.get();
            route_ag1 = agent1.get();
            route_ag2 = agent2.get();


            //shift vector order of the route
            vector<int> route_ag0_tempo, route_ag1_tempo, route_ag2_tempo;

            for (int i = 0; i < route_ag0.cols(); i++)
            {
                route_ag0_tempo.push_back(route_ag0(i));
//                cout<<route_ag0(i)<<endl;
            }
            for (int i = 0; i < route_ag1.cols(); i++)
            {
                route_ag1_tempo.push_back(route_ag1(i));
            }
            for (int i = 0; i < route_ag2.cols(); i++)
            {
                route_ag2_tempo.push_back(route_ag2(i));
            }

            vector<int>::iterator it0 = find(route_ag0_tempo.begin(), route_ag0_tempo.end(), current_city_0);
            vector<int>::iterator it1 = find(route_ag1_tempo.begin(), route_ag1_tempo.end(), current_city_1);
            vector<int>::iterator it2 = find(route_ag2_tempo.begin(), route_ag2_tempo.end(), current_city_2);
            int index0 = int(distance(route_ag0_tempo.begin(), it0));
            int index1 = int(distance(route_ag1_tempo.begin(), it1));
            int index2 = int(distance(route_ag2_tempo.begin(), it2));
            //the indices of the current city

//            cout<<"starto: "<<index0<<endl;

            vector<int> route_ag0_final, route_ag1_final, route_ag2_final;

            for (size_t i = size_t(index0) + 1; i < route_ag0_tempo.size(); i++)
            {
                route_ag0_final.push_back(route_ag0_tempo[i]);
            }

            for (size_t i = 0;i < size_t(index0);i++)
            {
                route_ag0_final.push_back(route_ag0_tempo[i]);
            }
            route_ag0_final.push_back(route_ag0_tempo[size_t(index0)]);

            //~~~~~~

            for (size_t i = size_t(index1) + 1; i < route_ag1_tempo.size(); i++)
            {
                route_ag1_final.push_back(route_ag1_tempo[i]);
            }

            for (size_t i = 0;i < size_t(index1);i++)
            {
                route_ag1_final.push_back(route_ag1_tempo[i]);
            }
            route_ag1_final.push_back(route_ag1_tempo[size_t(index1)]);

            //~~~~~~

            for (size_t i = size_t(index2) + 1; i < route_ag2_tempo.size(); i++)
            {
                route_ag2_final.push_back(route_ag2_tempo[i]);
            }

            for (size_t i = 0;i < size_t(index2);i++)
            {
                route_ag2_final.push_back(route_ag2_tempo[i]);
            }
            route_ag2_final.push_back(route_ag2_tempo[size_t(index2)]);
            //shifting done

            //now we push them back to offb (i know it's dumb but I write msg class after the assign class)
            //the assign class is elegant, go check it out, wink, wink;

            for(size_t i = 0; i < route_ag0_final.size(); i++)
            {
                offb::obj temp;
                temp.X_c = agent_0_insp[size_t(route_ag0_final[i])].x;
                temp.Y_c = agent_0_insp[size_t(route_ag0_final[i])].y;
                temp.Z_c = agent_0_insp[size_t(route_ag0_final[i])].z;
                uav0.points.push_back(temp);
            }


            for(size_t i = 0; i < route_ag1_final.size(); i++)
            {
                offb::obj temp;
                temp.X_c = agent_1_insp[size_t(route_ag1_final[i])].x;
                temp.Y_c = agent_1_insp[size_t(route_ag1_final[i])].y;
                temp.Z_c = agent_1_insp[size_t(route_ag1_final[i])].z;
                uav1.points.push_back(temp);
            }

            for(size_t i = 0; i < route_ag2_final.size(); i++)
            {
                offb::obj temp;
                temp.X_c = agent_2_insp[size_t(route_ag2_final[i])].x;
                temp.Y_c = agent_2_insp[size_t(route_ag2_final[i])].y;
                temp.Z_c = agent_2_insp[size_t(route_ag2_final[i])].z;
                uav2.points.push_back(temp);
            }

            calculated = true;

            if(temp_i == 0)
            {
                temp_i = 2;
            }
        }


        if(all_obj_found)
        {
            uav0.got_stuff = true;
            uav1.got_stuff = true;
            uav2.got_stuff = true;
        }

        pub_ag0_route.publish(uav0);
        pub_ag1_route.publish(uav1);
        pub_ag2_route.publish(uav2);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
