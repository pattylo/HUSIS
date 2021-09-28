#include "include/movement.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include "include/run_yolo.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"
#include "include/am_traj.hpp"
#include <random>
#include <sensor_msgs/Imu.h>
#include "offb/frame.h"

using namespace std;

//predefined
static double gx = 6.922 + 6 * cos(M_PI * 5/6), gy = 0 + 6 * sin(M_PI * 5/6), gz = 1.5;
static double sx = 4 * cos(M_PI * 11/6), sy = 4 * sin(M_PI * 11/6);//!!!!!!!NEED SET TO ZEROs WHEN EXPERIMENT AS VICON DOESN'T EXIST FRAME DIFFERENCE...I THINK?
static double r_safe = 1.5 * 0.55;
static double r_clear = 0.8;

enum flying_step
{
    IDLE,
    TO2Hover,
    Hover,
    Prep,
    Starto,
    Move2wp,
    Change,
    Avoid,
    HoverandSway,
    LAND,
    END,
    Land,
    gothrough,
    found,
    HoverSwaySearch1,
    HoverSwaySearch2,
    HoverSwaySearch3,
    locate,
    gotowp1,
    gotowp2,
    gotowp3,
    sethead,
    Prep2,
    Cal_T,
    maneuver,
    };

static flying_step  fly = IDLE;
static UAVpose uavinfo, uavinfo_0, uavinfo_2, uavinfo_0_f, uavinfo_2_f;
static Eigen::Vector3d predicted;
static mavros_msgs::State current_state;
static visualization_msgs::Marker UAVtrajectory;
static vector<waypts> waypoints;
static vector<waypts> referpts;
static vector<waypts> futurepts;
static cv::Mat frame;
static cv::Mat image_dep;
static visualization_msgs::Marker edge_points;
static double v, vx, vy, vz;
static vector<waypts> traj_p, traj_v, traj_a;
static size_t i = 0;
static double yaw = M_PI * 5/6;

static frame_d F;

void frame_cb(const offb::frameConstPtr &msg)
{
    F.x = msg->frame_x;
    F.y = msg->frame_y;
    F.got = msg->dFrame_calculated;
}

void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo.x = pose->pose.position.x + F.x;
    uavinfo.y = pose->pose.position.y + F.y;
    uavinfo.z = pose->pose.position.z;
    uavinfo.ow = pose->pose.orientation.w;
    uavinfo.ox = pose->pose.orientation.x;
    uavinfo.oy = pose->pose.orientation.y;
    uavinfo.oz = pose->pose.orientation.z;
}

void position_callback_0(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo_0.x = pose->pose.position.x;
    uavinfo_0.y = pose->pose.position.y;
    uavinfo_0.z = pose->pose.position.z;
}

void position_callback_0_f(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo_0_f.x = pose->pose.position.x;
    uavinfo_0_f.y = pose->pose.position.y;
    uavinfo_0_f.z = pose->pose.position.z;
}

void position_callback_2(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo_2.x = pose->pose.position.x;
    uavinfo_2.y = pose->pose.position.y;
    uavinfo_2.z = pose->pose.position.z;
}

void position_callback_2_f(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo_2_f.x = pose->pose.position.x;
    uavinfo_2_f.y = pose->pose.position.y;
    uavinfo_2_f.z = pose->pose.position.z;
}

void velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& velocity)
{
    vx = velocity->twist.linear.x;
    vy = velocity->twist.linear.y;
    vz = velocity->twist.linear.z;
    v = sqrt(vx*vx+vy*vy+vz*vz);
}

void rpy_to_Q(double yaw, double &w, double &x, double &y, double &z)
{
    w = cos(0) * cos (0) * cos (yaw/2) + sin (0) * sin (0) * sin (yaw/2) ;
    x = sin(0) * cos (0) * cos (yaw/2) - cos (0) * sin (0) * sin (yaw/2) ;
    y = cos(0) * sin (0) * cos (yaw/2) + sin (0) * cos (0) * sin (yaw/2) ;
    z = cos(0) * cos (0) * sin (yaw/2) - sin (0) * sin (0) * cos (yaw/2) ;
}

bool detect();
void am_traj_exc(vector<Eigen::Vector3d> detour);//detour will contain current state, the detour waypoint, and the goal state
Eigen::Vector3d getmidpt(geometry_msgs::PoseStamped future_state);

waypts b2w(Eigen::Matrix<double, 3, 1> body_pt)
{
    Eigen::Matrix<double, 4, 1> body, world;
    body(0) = body_pt(0);
    body(1) = body_pt(1);
    body(2) = body_pt(2);
    body(3) = 1;

    Eigen::Matrix<double, 3, 3> matrix_for_q;



    double current_yaw = atan2(vy, vx);
    vector<waypts> initializer;
    movement temp(initializer);
    geometry_msgs::PoseStamped uav_q;
    temp.rpy2Q(uav_q, current_yaw);

//    Eigen::Quaterniond q2r_matrix(uav_q.pose.orientation.w,
//                                  uav_q.pose.orientation.x,
//                                  uav_q.pose.orientation.y,
//                                  uav_q.pose.orientation.z);
    Eigen::Quaterniond q2r_matrix(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);

    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world <<
        matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), uavinfo.x,
        matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), uavinfo.y,
        matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), uavinfo.z,
        0,0,0,1;

    world = body_to_world * body;
    waypts result = {world(0),world(1), world(2)};
    return result;
}

int main(int argc, char **argv)
{
    cout<<"agent 1"<<endl;
    ros::init(argc, argv, "rca1"); //initialize ROS via passing argc argv. the "offb_node" will be the name of your node's name
    ros::NodeHandle nh;//the node handle to handle the process of the node. it'll as well intialize the node.

    //subscribers
    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>
                                ("/uav1/mavros/state", 1, state_callback);


    ros::Subscriber sub_uavposition = nh.subscribe<geometry_msgs::PoseStamped>
                                      ("/uav1/mavros/local_position/pose", 1, position_callback);

    ros::Subscriber sub_uav_0 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav0/current_state", 1, position_callback_0);

    ros::Subscriber sub_uav_2 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav2/current_state", 1, position_callback_2);

    ros::Subscriber sub_uav_0_f = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav0/future_state", 1, position_callback_0_f);

    ros::Subscriber sub_uav_2_f =nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav2/future_state", 1, position_callback_2_f);


    ros::Subscriber sub_velocity = nh.subscribe<geometry_msgs::TwistStamped>
                                   ("/uav1/mavros/local_position/velocity_local", 1, velocity_callback);

    ros::Subscriber sub_frame = nh.subscribe<offb::frame>
                                  ("/uav1/frame", 1, frame_cb);

    //publishers
    ros::Publisher pub_traj_pts = nh.advertise<geometry_msgs::PoseStamped>
                                  ("/uav1/mavros/setpoint_position/local", 1);

    ros::Publisher pub_now = nh.advertise<geometry_msgs::PoseStamped>
                                ("/uav1/current_state", 1);

    ros::Publisher pub_future = nh.advertise<geometry_msgs::PoseStamped>
                                ("/uav1/future_state", 1);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/uav1/mavros/set_mode");

    geometry_msgs::PoseStamped pose, future_state;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //mode setting
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    while(!F.got)
    {
        ros::spinOnce();
        rate.sleep();
    }
    cout<<"reference frame synced! Let's gooo~"<<endl;
    cout<<F.x<<endl;
    cout<<F.y<<endl;
    cout<<F.got<<endl;

    double last_request = ros::Time::now().toSec();
    waypts takeoff ={F.x,F.y,2};
    waypts lsp = {F.x,F.y,0}, lep;
    vector<waypts> initializer;
    movement move(initializer);

    //State Machines~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    while(ros::ok())
    {
        //preparation~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == IDLE)
        {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
            {
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now().toSec();
            }
            else
            {
                if( !current_state.armed && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
                {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                        fly = TO2Hover;
                        lep = takeoff;
                    }
                    last_request = ros::Time::now().toSec();
                }
            }
        }

        //takeoff~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == TO2Hover && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
        {
            move.justmove(uavinfo, pose, last_request, ros::Time::now(),lsp, lep);
            if(move.switchflymode_ == true)
            {
                last_request = ros::Time::now().toSec();
                fly = Move2wp;
                lsp = takeoff;
                lep = {gx,gy,gz};
            }
        }

        //move~~~~~~~~~~~~~~~~~~~~~~~~~~
        if(fly == Move2wp && (ros::Time::now().toSec() - last_request > ros::Duration(4.0).toSec()))
        {
            cout<<"hi I now calculate for stright line"<<endl;
            vector<Eigen::Vector3d> detour;
            Eigen::Vector3d now(uavinfo.x, uavinfo.y, uavinfo.z);
            Eigen::Vector3d goal(gx,gy,gz);
            detour.push_back(now);
            detour.push_back(goal);
            am_traj_exc(detour);
            fly = maneuver;
        }

        if(fly == Cal_T)
        {
            vector<Eigen::Vector3d> detour;
            Eigen::Vector3d now(uavinfo.x, uavinfo.y, uavinfo.z);
            Eigen::Vector3d goal(gx,gy,gz), mid = getmidpt(future_state);
            detour.push_back(now);
            detour.push_back(mid);
            detour.push_back(goal);
            cout<<"this is uav1: "<<endl;
            cout<<now<<endl;
            cout<<mid<<endl;
            cout<<goal<<endl;
            am_traj_exc(detour);
            fly = maneuver;
            i = 0;
        }

        if(fly == maneuver)
        {
            predicted = move.futurestate(traj_p, i);
            future_state.pose.position.x = predicted(0);
            future_state.pose.position.y = predicted(1);
            future_state.pose.position.z = predicted(2);

            pose.pose.position.x=traj_p[i].x;
            pose.pose.position.y=traj_p[i].y;
            pose.pose.position.z=traj_p[i].z;

            yaw = atan2(traj_v[i].y, traj_v[i].x);

            if(i<traj_p.size())
                i++;
            else
            {
                fly = Hover;
                last_request = ros::Time::now().toSec();
            }

            if(detect())
            {
                cout<<"uav1 says gonna hit"<<endl;
                fly = Cal_T;
            }

        }

        if(fly == Hover)
        {
            move.hover(pose, lep);
            if(ros::Time::now().toSec() - last_request > ros::Duration(5.0).toSec())
                fly = LAND;
        }

        if(fly == LAND)
        {
            waypts hi2, hi1;
            hi2 = {uavinfo.x, uavinfo.y, 2};
            hi1 = {uavinfo.x, uavinfo.y, 0};
            move.justmove(uavinfo, pose, last_request, ros::Time::now(), hi2, hi1);
            if(move.switchflymode_)
                fly = END;
        }


        if(fly == END)
        {
            pose.pose.position.z = pose.pose.position.z - 0.02;

            if(pose.pose.position.z < 0.05)
            {
                arm_cmd.request.value = false;
                if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success )
                {
                    cout << "UAV about to touch ground" << endl;
                    cout << "Touched and end...."<< endl;
                    return 0;//break the control UAV will land automatically
                }
             }
        }

        if(fly == TO2Hover || fly == maneuver || fly == Hover)
        {
            pose.pose.position.x = pose.pose.position.x - F.x;
            pose.pose.position.y = pose.pose.position.y - F.y;
        }

        rpy_to_Q(yaw,
                 pose.pose.orientation.w,
                 pose.pose.orientation.x,
                 pose.pose.orientation.y,
                 pose.pose.orientation.z);

        geometry_msgs::PoseStamped currentinfo;
        currentinfo.pose.position.x = uavinfo.x;
        currentinfo.pose.position.y = uavinfo.y;
        currentinfo.pose.position.z = uavinfo.z;

        pub_now.publish(currentinfo);
        pub_future.publish(future_state);
        pub_traj_pts.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

bool detect()
{
    double d10 = sqrt( pow( (predicted(0) - uavinfo_0.x), 2 )
                       + pow( (predicted(1) - uavinfo_0.y), 2 )
                       + pow( (predicted(2) - uavinfo_0.z), 2 ) );
    double d12 = sqrt( pow( (predicted(0) - uavinfo_2.x), 2 )
                       + pow( (predicted(1) - uavinfo_2.y), 2 )
                       + pow( (predicted(2) - uavinfo_2.z), 2 ) );
    double d10f = sqrt( pow( (predicted(0) - uavinfo_0_f.x), 2 )
                       + pow( (predicted(1) - uavinfo_0.y), 2 )
                       + pow( (predicted(2) - uavinfo_0.z), 2 ) );
    double d12f = sqrt( pow( (predicted(0) - uavinfo_2.x), 2 )
                       + pow( (predicted(1) - uavinfo_2.y), 2 )
                       + pow( (predicted(2) - uavinfo_2.z), 2 ) );

//    ofstream ds("patrick/home//d_1.txt", ios::app);
//    ds<<d10<<endl;
//    ds<<d12<<endl;
//    ds<<d10f<<endl;
//    ds<<d12f<<endl;
//    ds<<endl;
//    ds.close();


    if(d10 < r_clear || d12 < r_clear || d10f < r_clear || d12f < r_clear)
    {
        if(r_safe > uavinfo.z)
            r_safe = uavinfo.z * 0.5;
        else
            r_safe = 1.2 * 0.55;
        return true;
    }
    else
        return false;
}

Eigen::Vector3d getmidpt(geometry_msgs::PoseStamped future_state)
{
    Eigen::Matrix<double, 3, 1> temp;
    temp(0) = abs(sqrt(pow(future_state.pose.position.x - uavinfo.x,2)
                       + pow(future_state.pose.position.y - uavinfo.y,2)
                       + pow(future_state.pose.position.z - uavinfo.z,2))
                  );
//    temp(0) =  r_safe;
    temp(1) = -r_safe * cos(M_PI * 30 / 180);
    temp(2) =  r_safe * sin(M_PI * 30 / 180);

    waypts mdpt = b2w(temp);
    Eigen::Vector3d rv;
    rv(0) = mdpt.x;
    rv(1) = mdpt.y;
    rv(2) = mdpt.z;
    //rv = {0,-0.5,1};
    //    cout<<"hihihihihihih"<<endl;
    return rv;
//    Eigen::Vector3d rv;
//    rv = {0,0.5,1};
//    return rv;
}

void am_traj_exc(vector<Eigen::Vector3d> detour)//detour will contain current state, the detour waypoint, and the goal state
{

    //am_traj instantiation
    Eigen::Vector3d v0(vx,vy,vz), vf(0,0,0);
    Eigen::Vector3d a0(0,0,0), af(0,0,0);

//    if(traj_v.size() == 0)
//        v0 = Eigen::Vector3d(0,0,0);
//    else
//        v0 = Eigen::Vector3d(traj_v[i].x, traj_v[i].y, traj_v[i].z);

//    if(traj_a.size() == 0)
//        a0 = Eigen::Vector3d(0,0,0);
//    else
//        a0 = Eigen::Vector3d(traj_a[i].x, traj_a[i].y, traj_a[i].z);

    traj_p.clear();
    traj_v.clear();
    traj_a.clear();

    vector<Eigen::Vector3d> wps;
    for(auto wp : detour)
    {
        wps.emplace_back(wp);
    }
    //    wps.emplace_back(uavinfo.x, uavinfo.y, uavinfo.z);
    //    //wps.emplace_back(detour.x, detour.y, detour.z);
    //    wps.emplace_back(gx,gy,gz);

    AmTraj amTrajOpt(1024.0, 32.0, 1.0, 0.5, 0.5, 32, 0.02);//wT, wA, wJ, maxV, maxA, maxIterations, eps(relative tolerance)
    Trajectory traj = amTrajOpt.genOptimalTrajDTC(wps,v0,a0,vf,af);
    double T = 0.02;
    waypts temp;
    Eigen::Vector3d lastX = traj.getPos(0.0);
    Eigen::Vector3d lastV = traj.getVel(0.0);
    Eigen::Vector3d lastA = traj.getAcc(0.0);

    //    cout<<"hi, duration"<<endl;
    //    cout<<traj.getTotalDuration()<<endl;
    for(double t = 0.02; t < traj.getTotalDuration(); t+=T)
    {
        geometry_msgs::Point point_p, point_v, point_a;
        Eigen::Vector3d X = traj.getPos(t);
        Eigen::Vector3d V = traj.getVel(t);
        Eigen::Vector3d A = traj.getAcc(t);

        point_p.x = lastX(0);
        point_p.y = lastX(1);
        point_p.z = lastX(2);
        temp.x = point_p.x;
        temp.y = point_p.y;
        temp.z = point_p.z;
        traj_p.push_back(temp);
        lastX = X;

        point_v.x = lastV(0);
        point_v.y = lastV(1);
        point_v.z = lastV(2);
        temp.x = point_v.x;
        temp.y = point_v.y;
        temp.z = point_v.z;
        traj_v.push_back(temp);
        lastV = V;

        point_a.x = lastA(0);
        point_a.x = lastA(1);
        point_a.x = lastA(2);
        temp.x = point_a.x;
        temp.y = point_a.y;
        temp.z = point_a.z;
        traj_a.push_back(temp);
        lastA = A;

        //        edge_points.points.push_back(point);
        //        point.x = X(0);
        //        point.y = X(1);
        //        point.z = X(2);
        //        edge_points.points.push_back(point);
    }
    //    cout<<"optimized result:"<<trajectory.size()<<endl;
}


