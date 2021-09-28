    #include "include/movement.h"
#include "visualization_msgs/Marker.h"
#include <algorithm>
#include "include/run_yolo.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/PointStamped.h"
#include "include/am_traj.hpp"
#include <random>
#include <sensor_msgs/Imu.h>


using namespace std;

//predefined
static double gx = 2 * cos(M_PI * 1/6), gy = 2 * sin(M_PI * 1/6), gz = 1;
static double sx = 4 * cos(M_PI * 7/6), sy = 4 * sin(M_PI * 7/6);
//->!!!!!!!NEED SET TO ZEROs WHEN EXPERIMENT, AS VICON DOESN'T EXIST FRAME DIFFERENCE
//->!!!!!!!REMAIN THE SAME WHEN CONDUCTING EXPERIMENT AT OUTSIDE
static double r_safe = 0.5;

enum flying_step
{
    IDLE,
    TO2Hover,
    Hover,
    Move2wp,
    HoverandSway,
    Assign,
    Land,
    found,
    Inspect,
    locate,
    gotowp1,
    gotowp2,
    gotowp3,
    Heading,
    Cal_T,
    maneuver,
};

static flying_step  fly = IDLE;
static UAVpose uavinfo, uavinfo_1, uavinfo_2, uavinfo_1_f, uavinfo_2_f;
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
static double ax, ay, az;
static vector<waypts> traj_p, traj_v, traj_a;
static size_t i = 0;
static double yaw = M_PI * 1/6;


void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void position_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo.x = pose->pose.position.x + sx;
    uavinfo.y = pose->pose.position.y + sy;
    uavinfo.z = pose->pose.position.z;
    uavinfo.ow = pose->pose.orientation.w;
    uavinfo.ox = pose->pose.orientation.x;
    uavinfo.oy = pose->pose.orientation.y;
    uavinfo.oz = pose->pose.orientation.z;
    //cout<<uavinfo.x<<endl;
}

void position_callback_1_f(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
//    cout<<"hi"<<endl;
    uavinfo_1_f.x = pose->pose.position.x;
    uavinfo_1_f.y = pose->pose.position.y;
    uavinfo_1_f.z = pose->pose.position.z;
}

void position_callback_2_f(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo_2_f.x = pose->pose.position.x;
    uavinfo_2_f.y = pose->pose.position.y;
    uavinfo_2_f.z = pose->pose.position.z;
}

void position_callback_1(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    //    cout<<"hi"<<endl;
    uavinfo_1.x = pose->pose.position.x;
    uavinfo_1.y = pose->pose.position.y;
    uavinfo_1.z = pose->pose.position.z;
}

void position_callback_2(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uavinfo_2.x = pose->pose.position.x;
    uavinfo_2.y = pose->pose.position.y;
    uavinfo_2.z = pose->pose.position.z;
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

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    ax = msg -> linear_acceleration.x;
    ay = msg -> linear_acceleration.y;
    az = msg -> linear_acceleration.z;
}

void callback(const sensor_msgs::CompressedImageConstPtr & rgbimage, const sensor_msgs::ImageConstPtr & depth)
{
    cv_bridge::CvImageConstPtr depth_ptr;
    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image_dep = depth_ptr->image;

    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

}

static double fx, fy, cx, cy; //focal length and principal point

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
}

bool detect();
void am_traj_exc(vector<Eigen::Vector3d> detour);//detour will contain current state, the detour waypoint, and the goal state
Eigen::Vector3d getmidpt(geometry_msgs::PoseStamped futurestate);
waypts b2w(Eigen::Matrix<double, 3, 1> body_pt)
{
    Eigen::Matrix<double, 4, 1> body, world;
    body(0) = body_pt(0);
    body(1) = body_pt(1);
    body(2) = body_pt(2);
    body(3) = 1;
    Eigen::Matrix<double, 3, 3> matrix_for_q;
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
    cout<<"agent 0"<<endl;
    ros::init(argc, argv, "rca"); //initialize ROS via passing argc argv. the "offb_node" will be the name of your node's name
    ros::NodeHandle nh;//the node handle to handle the process of the node. it'll as well intialize the node.

    //the followings are the instantiation of either subscriber, publisher, service & client
    //it tells the ROS master that what we're gonna execute in this node

    //subscribers
    ros::Subscriber sub_state = nh.subscribe<mavros_msgs::State>
                                ("/uav0/mavros/state", 1, state_callback);
    ros::Subscriber sub_uavposition = nh.subscribe<geometry_msgs::PoseStamped>
                                   ("/uav0/mavros/local_position/pose", 1, position_callback);

    ros::Subscriber sub_uav_1_f = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav1/future_state", 1, position_callback_1_f);

    ros::Subscriber sub_uav_2_f = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav2/future_state", 1, position_callback_2_f);

    ros::Subscriber sub_uav_1 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav1/future_state", 1, position_callback_1);

    ros::Subscriber sub_uav_2 = nh.subscribe<geometry_msgs::PoseStamped>
                                  ("/uav2/future_state", 1, position_callback_2);

    ros::Subscriber sub_velocity = nh.subscribe<geometry_msgs::TwistStamped>
                                      ("/uav0/mavros/local_position/velocity_local", 1, velocity_callback);

    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>
                                  ("/uav0/mavros/imu/data", 1, imu_cb);

//    ros::Subscriber sub_a = nh.subscribe<geometry_msgs::AccelStamped>
//                            ("/uav0/mavros/local_position/")

    //publishers
    ros::Publisher pub_traj_pts = nh.advertise<geometry_msgs::PoseStamped>
                                   ("/uav0/mavros/setpoint_position/local", 1);

    ros::Publisher pub_future = nh.advertise<geometry_msgs::PoseStamped>
                                  ("/uav0/future_state", 1);

    //ros::Publisher

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("/uav0/mavros/set_mode");

//    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
//    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
//    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
//    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
//    sync.registerCallback(boost::bind(&callback, _1, _2));
//    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);

    geometry_msgs::PoseStamped pose, future_state;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    //send a few setpoints before starting
//    for(int i = 100; ros::ok() && i > 0; --i){
//        pub_traj_pts.publish(pose);
//        ros::spinOnce();
//        rate.sleep();
//    }

    //mode setting
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    double last_request = ros::Time::now().toSec();
    waypts takeoff ={sx,sy,1};
    waypts lsp = {sx,sy,0}, lep;
    vector<waypts> initializer;
    movement move(initializer);

    frame_d F ={sx,sy};

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
            Eigen::Vector3d now(uavinfo.x, uavinfo.y, uavinfo.z);//traj_p[i].x, traj_p[i].y, traj_p[i].z;
            Eigen::Vector3d goal(gx,gy,gz), mid = getmidpt(future_state);
            detour.push_back(now);
            detour.push_back(mid);
            detour.push_back(goal);
            cout<<"agent 0 non state"<<endl;
            cout<<uavinfo.x<<endl;
            cout<<uavinfo.y<<endl;
            cout<<uavinfo.z<<endl;
            cout<<"agent 0 waypts"<<endl;
            cout<<now<<endl;
            cout<<mid<<endl;
            cout<<goal<<endl<<endl;
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
//            cout<<future_state.pose.position.x<<endl;
//            double x,y,z;
//            x=trajectory[i].x;
//            y=trajectory[i].y;
//            z=trajectory[i].z;

            pose.pose.position.x=traj_p[i].x;
            pose.pose.position.y=traj_p[i].y;
            pose.pose.position.z=traj_p[i].z;

            yaw = atan2(traj_v[i].y, traj_v[i].x);

            if(i<traj_p.size())
                i++;
            else
                fly = Hover;


            if(detect())
            {
                cout<<"uav0 says gonna hit"<<endl;
                fly = Cal_T;
            }

        }

//        cout<<fly<<endl;
        if(fly == Hover)
        {
            move.hover(pose, lep);
        }

//        cout<<move.switchflymode_;

        rpy_to_Q(yaw, pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);

        if(fly == TO2Hover || fly == maneuver || fly == Hover)
        {
            pose.pose.position.x = pose.pose.position.x - F.x;
            pose.pose.position.y = pose.pose.position.y - F.y;
        }


//        future_state.pose.position.x = future_state.pose.position.x - F.x;
//        future_state.pose.position.y = future_state.pose.position.y - F.y;
//        predicted(0) = future_state.pose.position.x;
//        predicted(1) = future_state.pose.position.y;

        pub_future.publish(future_state);
        pub_traj_pts.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

bool detect()
{
    double d01 = sqrt( pow( (predicted(0) - uavinfo_1_f.x), 2 ) + pow( (predicted(1) - uavinfo_1_f.y), 2 ) + pow( (predicted(2) - uavinfo_1_f.z), 2 ) );
    double d02 = sqrt( pow( (predicted(0) - uavinfo_2_f.x), 2 ) + pow( (predicted(1) - uavinfo_2_f.y), 2 ) + pow( (predicted(2) - uavinfo_2_f.z), 2 ) );
//    cout<<"agent 0 predict itself"<<predicted(0)<<endl;
//    cout<<"agent 0 predict others"<<uavinfo_1_f.x<<endl;
//    cout<<"d01: "<<d01<<endl;
    if(d01 < 0.4 && d02 < 0.4)
    {
        r_safe = 0.8;
        return true;
    }
    else if(d01 < 0.4 || d02 < 0.4)
    {
        r_safe = 0.5;
        return true;
    }
    else
        return false;
}

Eigen::Vector3d getmidpt(geometry_msgs::PoseStamped future_state)
{
//    Eigen::Vector3d ego_direction, ego_direction_1, ego_direction_2;
//    ego_direction = Eigen::Vector3d(future_state.pose.position.x-uavinfo.x,
//                                   future_state.pose.position.x-uavinfo.y,
//                                   future_state.pose.position.x-uavinfo.z);
//    ego_direction_1 = Eigen::Vector3d(uavinfo_1_f.x - uavinfo_1.x,
//                                      uavinfo_1_f.y - uavinfo_1.y,
//                                      uavinfo_1_f.z - uavinfo_1.z);
//    ego_direction_2 = Eigen::Vector3d(uavinfo_2_f.x - uavinfo_2.x,
//                                      uavinfo_2_f.y - uavinfo_2.y,
//                                      uavinfo_2_f.z - uavinfo_2.z);
//    int whichcase;

//    //whichcase = 0 -> roundabout
//    //whichcase = 1 -> go straight
//    //whichcase = 2 -> go up
//    //whichcase = 3 -> go down

//    double ip01 = ego_direction.dot(ego_direction_1);
//    double ip02 = ego_direction.dot(ego_direction_2);

//    if(ip01 < 0)
//    {
//        whichcase = 0;
//    }

//    if(whichcase == 0)
//        ;

//    cout<<"agent 0 says my future and current"<<uavinfo.x<<" "<<future_state.pose.position.x<<endl;

    Eigen::Matrix<double, 3, 1> temp;
    temp(0) = abs(sqrt(pow(future_state.pose.position.x - uavinfo.x,2)
                       + pow(future_state.pose.position.y - uavinfo.y,2)
                       + pow(future_state.pose.position.z - uavinfo.z,2))
                  );
    temp(1) = -r_safe;
    temp(2) = 0;

    waypts mdpt = b2w(temp);
    Eigen::Vector3d rv;
    rv(0) = mdpt.x;
    rv(1) = mdpt.y;
    rv(2) = mdpt.z;
    //rv = {0,-0.5,1};
//    cout<<"hihihihihihih"<<endl;
    return rv;
}

void am_traj_exc(vector<Eigen::Vector3d> detour)//detour will contain current state, the detour waypoint, and the goal state
{

    //am_traj instantiation
    Eigen::Vector3d v0(vx,vy,vz), vf(0,0,0);
    Eigen::Vector3d a0(ax,ay,az), af(0,0,0);
//    cout<<"velocity"<<endl<<vx<<endl<<vy<<endl<<vz<<endl<<endl;;

//    cout<<"hey, I'm agent 0, and my velocity (when optimization) is:"<<endl<<v0<<endl;
//    cout<<"hey, I'm agent 0, and my accelera (when optimization) is:"<<endl<<a0<<endl;

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

    AmTraj amTrajOpt(1024.0, 32.0, 1.0, 1.0, 0.5, 32, 0.02);//wT, wA, wJ, maxV, maxA, maxIterations, eps(relative tolerance)
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


