#include "include/movement.h"
#include <offb/obj.h>
#include <offb/objs.h>
#include <offb/frame.h>
#include <sensor_msgs/NavSatFix.h>

static vector<offb::obj> devices;
static bool all_obj_found = false;
static offb::frame ag0_frame_pub, ag1_frame_pub, ag2_frame_pub;

void obj_cb(const offb::objsConstPtr &msg)
{
    devices = msg->points;
    if (devices.size() > 10)
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

void pose_0_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    uavinfo0.x = msg->pose.position.x + ag0_frame_pub.frame_x;
    uavinfo0.y = msg->pose.position.y + ag0_frame_pub.frame_y;
    uavinfo0.z = msg->pose.position.z;
}

void pose_1_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    uavinfo1.x = msg->pose.position.x + ag1_frame_pub.frame_x;
    uavinfo1.y = msg->pose.position.y + ag1_frame_pub.frame_y;
    uavinfo1.z = msg->pose.position.z;
}

void pose_2_cb(const geometry_msgs::PoseStampedConstPtr &msg)
{
    uavinfo2.x = msg->pose.position.x + ag2_frame_pub.frame_x;
    uavinfo2.y = msg->pose.position.y + ag2_frame_pub.frame_y;
    uavinfo2.z = msg->pose.position.z;
}

static waypts hp0, hp1, hp2;
static int gps0_s = -2, gps1_s = -2, gps2_s = -2;
void gps_0_cb(const sensor_msgs::NavSatFixConstPtr &msg)
{
    gps0_s = msg->status.status;
    hp0.x = msg->longitude;
    hp0.y = msg->latitude;
    hp0.z = msg->altitude;
}

void gps_1_cb(const sensor_msgs::NavSatFixConstPtr &msg)
{
    gps1_s = msg->status.status;
    hp1.x = msg->longitude;
    hp1.y = msg->latitude;
    hp1.z = msg->altitude;
}

void gps_2_cb(const sensor_msgs::NavSatFixConstPtr &msg)
{
    gps2_s = msg->status.status;
    hp2.x = msg->longitude;
    hp2.y = msg->latitude;
    hp2.z = msg->altitude;
}

void gps2m(frame_d gps_coord_ag0, frame_d &gps_coord_other_ag)//gps to meter
{
    double Lat1 = gps_coord_ag0.y;
    double Lon1 = gps_coord_ag0.x;
    double Lat2 = gps_coord_other_ag.y;
    double Lon2 = gps_coord_other_ag.x;

    double latMid, m_per_deg_lat, m_per_deg_lon, deltaLat, deltaLon, dist_m_x, dist_m_y;

    latMid = (Lat1+Lat2 )/2.0;  // or just use Lat1 for slightly less accurate estimate


    m_per_deg_lat = 111132.954 - 559.822 * cos( 2.0 * latMid * M_PI / 180) + 1.175 * cos( 4.0 * latMid * M_PI / 180);
    m_per_deg_lon = (3.14159265359/180 ) * 6367449 * cos ( latMid * M_PI / 180);

//    m_per_deg_lat = 111.32 * 1000;
//    m_per_deg_lon = 40075 * cos(latMid * M_PI / 180) / 360;
//    cout<<m_per_deg_lat<<endl;
//    cout<<m_per_deg_lon<<endl;

//    cout<<"here's the difference: "<<Lat1 - Lat2<<endl;
//    cout<<"here's the difference: "<<Lon1 - Lon2<<endl;


    dist_m_x = (Lon2 - Lon1) * m_per_deg_lon;
    dist_m_y = (Lat2 - Lat1) * m_per_deg_lat;

    gps_coord_other_ag.x = dist_m_x;
    gps_coord_other_ag.y = dist_m_y;
}

void frame_cal();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coordinate");
    ros::NodeHandle nh;
    ag0_frame_pub.frame_x = 0;
    ag0_frame_pub.frame_y = 0;
    ag0_frame_pub.dFrame_calculated = false;
    ag1_frame_pub.frame_x = 0;
    ag1_frame_pub.frame_y = 0;
    ag1_frame_pub.dFrame_calculated = false;
    ag2_frame_pub.frame_x = 0;
    ag2_frame_pub.frame_y = 0;
    ag2_frame_pub.dFrame_calculated = false;

    ros::Subscriber obj_sub = nh.subscribe<offb::objs>
                                  ("/searched_by_yolo", 1, obj_cb);
    ros::Subscriber sub_pos_0 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav0/mavros/local_postion/pose", 1, pose_0_cb);
    ros::Subscriber sub_pos_1 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav1/mavros/local_postion/pose", 1, pose_1_cb);
    ros::Subscriber sub_pos_2 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav2/mavros/local_postion/pose", 1, pose_2_cb);

    ros::Subscriber sub_ag0_gps = nh.subscribe<sensor_msgs::NavSatFix>
                                  ("/uav0/mavros/global_position/global", 1, gps_0_cb);
    ros::Subscriber sub_ag1_gps = nh.subscribe<sensor_msgs::NavSatFix>
                                  ("/uav1/mavros/global_position/global", 1, gps_1_cb);
    ros::Subscriber sub_ag2_gps = nh.subscribe<sensor_msgs::NavSatFix>
                                  ("/uav2/mavros/global_position/global", 1, gps_2_cb);

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

    ros::Publisher pub_ag0_dFrame = nh.advertise<offb::frame>
                                  ("/uav0/frame", 1);
    ros::Publisher pub_ag1_dFrame = nh.advertise<offb::frame>
                                  ("/uav1/frame", 1);
    ros::Publisher pub_ag2_dFrame = nh.advertise<offb::frame>
                                  ("/uav2/frame", 1);

    ros::Rate rate(20);
    while(ros::ok() && !current_state_0.connected && !current_state_1.connected && !current_state_2.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (gps0_s <= -1 || gps1_s <= -1 || gps2_s <= -1)
    {
        ros::spinOnce();
        rate.sleep();
    }

    for (int i = 0; i < 80 ; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    frame_cal();

    Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic> route_ag0, route_ag1, route_ag2;
    bool calculated = false;
    offb::objs uav0, uav1, uav2;

    int assign_no_0, assign_no_1, assign_no_2;

    while(ros::ok())
    {
        //add coordinate transformation into uavinfo
        if(all_obj_found && !calculated)
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
                result = distrib.solution(insp_wps, insp_ags);
            }

            vector<waypts> agent_0_insp, agent_1_insp, agent_2_insp;
            for (auto what : result)//check ha here
            {
                if(what.id < assign_no_0)
                {
                    waypts temp;
                    temp = {insp_wps[size_t(what.id)].pt_w(0),
                            insp_wps[size_t(what.id)].pt_w(1),
                            insp_wps[size_t(what.id)].pt_w(2)};
                    agent_0_insp.push_back(temp);
                }
                else if(what.id > assign_no_0 && what.id < assign_no_0 + assign_no_1)
                {
                    waypts temp;
                    temp = {insp_wps[size_t(what.id)].pt_w(0),
                           insp_wps[size_t(what.id)].pt_w(1),
                           insp_wps[size_t(what.id)].pt_w(2)};
                    agent_1_insp.push_back(temp);
                }
                else if (what.id > assign_no_0 + assign_no_1)
                {
                    waypts temp;
                    temp = {insp_wps[size_t(what.id)].pt_w(0),
                           insp_wps[size_t(what.id)].pt_w(1),
                           insp_wps[size_t(what.id)].pt_w(2)};
                    agent_2_insp.push_back(temp);
                }
            }
            //now we know agent 0,1,2 and their corresponding mission waypoints
            //now we conduct ACOTSP

            //yet we need waypts datatype to do ACOTSP

            //we also add the agents' current position to the cities list
            waypts wp0_now, wp1_now, wp2_now;

            wp0_now = {uavinfo0.x, uavinfo0.y, uavinfo0.z};
            agent_0_insp.push_back(wp0_now);

            wp1_now = {uavinfo1.x, uavinfo1.y, uavinfo1.z};
            agent_1_insp.push_back(wp1_now);

            wp2_now = {uavinfo2.x, uavinfo2.y, uavinfo2.z};
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

            vector<int> route_ag0_final, route_ag1_final, route_ag2_final;

            for (size_t i = size_t(index0); i < route_ag0_tempo.size(); i++)
            {
                route_ag0_final.push_back(route_ag0_tempo[i]);
            }

            for (size_t i = 0;i < size_t(index0);i++)
            {
                route_ag0_final.push_back(route_ag0_tempo[i]);
            }
            route_ag0_final.push_back(route_ag0_tempo[size_t(index0)]);

            //~~~~~~

            for (size_t i = size_t(index1); i < route_ag1_tempo.size(); i++)
            {
                route_ag1_final.push_back(route_ag1_tempo[i]);
            }

            for (size_t i = 0;i < size_t(index0);i++)
            {
                route_ag0_final.push_back(route_ag0_tempo[i]);
            }
            route_ag0_final.push_back(route_ag0_tempo[size_t(index0)]);

            //~~~~~~

            for (size_t i = size_t(index2); i < route_ag2_tempo.size(); i++)
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
            uav0.got_stuff = true;
            uav1.got_stuff = true;
            uav2.got_stuff = true;
        }

        pub_ag0_route.publish(uav0);
        pub_ag1_route.publish(uav1);
        pub_ag2_route.publish(uav2);

        pub_ag0_dFrame.publish(ag0_frame_pub);
        pub_ag1_dFrame.publish(ag1_frame_pub);
        pub_ag2_dFrame.publish(ag2_frame_pub);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void frame_cal()
{
    ros::Rate rate(20);
    Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic> route_ag0, route_ag1, route_ag2;

    vector<double> long_avg_0, lati_avg_0, long_avg_1, lati_avg_1, long_avg_2, lati_avg_2;

    for (int i = 0; i < 40; i++) // the hp here is the global absolute position
    {
        long_avg_0.push_back(hp0.x);
        lati_avg_0.push_back(hp0.y);
        long_avg_1.push_back(hp1.x);
        lati_avg_1.push_back(hp1.y);
        long_avg_2.push_back(hp2.x);
        lati_avg_2.push_back(hp2.y);
        ros::spinOnce();
        rate.sleep();
    }

    frame_d F_0 = {accumulate(long_avg_0.begin(), long_avg_0.end(),0.0)/long_avg_0.size(),
                   accumulate(lati_avg_0.begin(), lati_avg_0.end(),0.0)/lati_avg_0.size()};

    frame_d F_1 = {accumulate(long_avg_1.begin(), long_avg_1.end(),0.0)/long_avg_1.size(),
                   accumulate(lati_avg_1.begin(), lati_avg_1.end(),0.0)/lati_avg_1.size()};

    frame_d F_2 = {accumulate(long_avg_2.begin(), long_avg_2.end(),0.0)/long_avg_2.size(),
                   accumulate(lati_avg_2.begin(), lati_avg_2.end(),0.0)/lati_avg_2.size()};

    gps2m(F_0, F_1);
    gps2m(F_0, F_2);
    gps2m(F_0, F_0);

    cout<<"the result of the d_frame:"<<endl;
    cout<<F_0.x<<endl;
    cout<<F_0.y<<endl;

    cout<<F_1.x<<endl;
    cout<<F_1.y<<endl;

    cout<<F_2.x<<endl;
    cout<<F_2.y<<endl;

    ag0_frame_pub.frame_x = F_0.x;
    ag0_frame_pub.frame_y = F_0.y;
    ag0_frame_pub.dFrame_calculated = true;

    ag1_frame_pub.frame_x = F_1.x;
    ag1_frame_pub.frame_y = F_1.y;
    ag1_frame_pub.dFrame_calculated = true;

    ag2_frame_pub.frame_x = F_2.x;
    ag2_frame_pub.frame_y = F_2.y;
    ag2_frame_pub.dFrame_calculated = true;

}
