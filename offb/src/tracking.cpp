#include "include/run_yolo.h"
#include <opencv2/tracking/kalman_filters.hpp>
#include <visualization_msgs/Marker.h>
#include "include/movement.h"
#include "offb/obj.h"
#include "offb/objs.h"
#include <std_msgs/Bool.h>

using namespace std;
static cv::Mat frame;

//static cv::String weightpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/insp_yolo/yolov4-insp.weights";
//static cv::String cfgpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/insp_yolo/yolov4-insp.cfg";
//static cv::String classnamepath = "/home/patrick/catkin_ws/src/offb/src/include/yolo/insp_yolo/insp.names";

//static cv::String weightpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/official_yolo/yolov4-tiny.weights";
//static cv::String cfgpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/official_yolo/yolov4-tiny.cfg";
//static cv::String classnamepath = "/home/patrick/catkin_ws/src/offb/src/include/yolo/official_yolo/coco.names";



static cv::String weightpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/e_yolo/yb.weights";
static cv::String cfgpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/e_yolo/yb.cfg";
static cv::String classnamepath = "/home/patrick/catkin_ws/src/offb/src/include/yolo/e_yolo/yb.names";



//static cv::String weightpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/fyp_yolo/yt608.weights";
//static cv::String cfgpath ="/home/patrick/catkin_ws/src/offb/src/include/yolo/fyp_yolo/yt608.cfg";
//static cv::String classnamepath = "/home/patrick/catkin_ws/src/offb/src/include/yolo/fyp_yolo/obj.names";

static run_yolo Yolonet(cfgpath, weightpath, classnamepath, float(0.75));

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
    cv::Mat image_dep = depth_ptr->image;
    Yolonet.getdepthdata(image_dep);
    try
    {
        frame = cv::imdecode(cv::Mat(rgbimage->data),1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

//as the objects are not "dynamic" and the drone is not moving super nonlinearly
//-> IIR filter should be fine
static UAVpose uavinfo;
void position_cb( const geometry_msgs::PoseStampedConstPtr &msg)
{
    uavinfo.x = msg -> pose.position.x;
    uavinfo.y = msg -> pose.position.y;
    uavinfo.z = msg -> pose.position.z;
    uavinfo.ow = msg -> pose.orientation.w;
    uavinfo.ox = msg -> pose.orientation.x;
    uavinfo.oy = msg -> pose.orientation.y;
    uavinfo.oz = msg -> pose.orientation.z;
}

static double fx, fy, cx, cy;
void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg)
{
    fx = msg->K[0];//629.742
    fy = msg->K[4];//629.074
    cx = msg->K[2];//642.324
    cy = msg->K[5];//362.136
}


static bool swayornot;

void sway_cb(const std_msgs::Bool::ConstPtr& msg)
{
    swayornot = msg->data;
}

//static double z_object,x_object,y_object;
//static offb::objs obj_real_states;

Eigen::Vector4d c2w_cities(Eigen::Vector3d measured)
{
    Eigen::Translation3d tranOdomBody(uavinfo.x, uavinfo.y, uavinfo.z);
    Eigen::Quaterniond quatOdomBody(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);
    Eigen::Isometry3d curOdomPose = tranOdomBody * quatOdomBody;

    double z = measured(2) - 1.2,
            x = z * (measured(0) - cx) / fx,
            y = z * (measured(1) - cy) / fy;

    Eigen::Vector4d pt_c (x, y, z, 1);
    Eigen::Vector4d offset(0.14,0.02,0,0);
    Eigen::Matrix4d m_c2b;
    m_c2b << 0, 0, 1, 0,
            -1, 0, 0, 0,
             0, -1, 0, 0,
             0, 0, 0, 1;
    Eigen::Matrix4d m_b2w = curOdomPose.matrix();

    Eigen::Vector4d pt_w = m_b2w * (m_c2b * pt_c + offset);

    return pt_w;//return city position (not the state of the object
}

Eigen::Vector4d c2w(Eigen::Vector3d measured)
{
    Eigen::Translation3d tranOdomBody(uavinfo.x, uavinfo.y, uavinfo.z);
    Eigen::Quaterniond quatOdomBody(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);
    Eigen::Isometry3d curOdomPose = tranOdomBody * quatOdomBody;

    double z = measured(2),
            x = z * (measured(0) - cx) / fx,
            y = z * (measured(1) - cy) / fy;

    Eigen::Matrix<double, 4, 1> cam (x,y,z,1), body, world;
    Eigen::Matrix<double, 4, 4> cam_to_body;
    cam_to_body << 0,0,1,0,
        -1,0,0,0,
        0,-1,0,0,
        0,0,0,1;

    Eigen::Matrix<double, 3, 3> matrix_for_q;
    Eigen::Quaterniond q2r_matrix(uavinfo.ow, uavinfo.ox, uavinfo.oy, uavinfo.oz);

    matrix_for_q = q2r_matrix.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> body_to_world;
    body_to_world <<
        matrix_for_q(0,0), matrix_for_q(0,1), matrix_for_q(0,2), uavinfo.x,
        matrix_for_q(1,0), matrix_for_q(1,1), matrix_for_q(1,2), uavinfo.y,
        matrix_for_q(2,0), matrix_for_q(2,1), matrix_for_q(2,2), uavinfo.z,
        0,0,0,1;

    Eigen::Vector4d offset(0.14,0.02,0,0);
    world = body_to_world * cam_to_body * (cam + offset);


    Eigen::Vector4d pt_w;
    pt_w(0) = world(0);
    pt_w(1) = world(1);
    pt_w(2) = world(2);
    pt_w(3) = 1;
ofstream save("/home/patrick/catkin_ws/src/offb/src/include/state/obj.txt", ios::app);
            save<<pt_w<<endl<<endl;
        save<<uavinfo.x<<endl;
        save<<uavinfo.y<<endl;
        save<<uavinfo.z<<endl;
        save<<endl;
            save.close();

    return pt_w;//return city position (not the state of the object
}

int main(int argc, char** argv)
{
    cout<<"Object detection..."<<endl;

    ros::init(argc, argv, "tracking");
    ros::NodeHandle nh;
    ros::Publisher obj_wp_pub = nh.advertise<offb::objs>("/cities",1);
    ros::Publisher obj_ob_pub = nh.advertise<offb::objs>("/wherearetheobjects", 1);

    message_filters::Subscriber<sensor_msgs::CompressedImage> subimage(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<sensor_msgs::Image> subdepth(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subimage, subdepth);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);

    ros::Subscriber uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 1, position_cb);
    ros::Subscriber sway_sub = nh.subscribe<std_msgs::Bool>
        ("/now_sway", 1, sway_cb);




    ros::Rate rate(20.0);

    offb::objs enms_cities, enms_obj;
    enms_cities.got_stuff = false;
    enms_obj.got_stuff = false;

    vector<objectinfo> detections;
    vector<Match> result_ids, result_ids_obj;

    vector<MOT> measurements, measurements_obj;
    vector<MOT> city_states, objstates;
    vector<int> new_candidate, new_candidate_obj;

    Eigen::Matrix4d IIR_n, IIR_o;
    IIR_n << 0.9, 0, 0, 0,
            0, 0.9, 0, 0,
            0, 0, 0.9, 0,
            0, 0, 0, 1;

    IIR_o << 0.1, 0, 0, 0,
            0, 0.1, 0, 0,
            0, 0, 0.1, 0,
            0, 0, 0, 1;

    bool first_time = true;
int what_i = 0;

    while(ros::ok())
    {
        if(!frame.empty() )//&& swayornot)
        {
            Yolonet.rundarknet(frame);
            detections = Yolonet.obj_vector;

            if(!first_time)
            {

                cout<<endl<<endl;
                for (size_t i = 0; i < detections.size(); i++)
                {
                    Eigen::Vector3d temp (detections[i].boundingbox.x + detections[i].boundingbox.width / 2,
                                          detections[i].boundingbox.y + detections[i].boundingbox.height/ 2,
                                          detections[i].depth);

                    Eigen::Vector4d convert = c2w_cities (temp);
                    MOT anobj;
                    anobj.pt_w = convert;
                    measurements.push_back(anobj);

                    Eigen::Vector4d convert_obj = c2w(temp);
                    MOT anobj_real;
                    anobj_real.pt_w = convert_obj;
                    measurements_obj.push_back(anobj_real);

                }
//                cout<<"mea: "<<measurements.size()<<endl;

                assign::hungarian match;
                result_ids = match.solution(measurements, city_states);

                assign::hungarian match_for_object;
                result_ids_obj = match_for_object.solution(measurements_obj, objstates);
                //results here

//                cout<<"this is result_id: "<<result_ids.size()<<endl;
//                cout<<endl<<endl;
//                for(auto what : result_ids)
//                {
//                    cout<<what.id<<endl;
//                }
//                cout<<endl<<endl;

                for (size_t i = 0; i < result_ids.size(); i++)
                {
                    if(result_ids[i].toofar)
                    {
                        cout<<"newbie here"<<endl;
                        new_candidate.push_back(int(i));
                    }
                    else
                    {
                        if(result_ids[i].id > int(city_states.size() - 1))//should be objectstate size
                        {
                            cout<<"newbie here"<<endl;
                            new_candidate.push_back(int(i));
                        }
                        else
                        {
                            if(i < measurements.size())
                                city_states[size_t(result_ids[i].id)].pt_w
                                    = IIR_n * measurements[i].pt_w
                                    + IIR_o * city_states[size_t(result_ids[i].id)].pt_w;
                            else
                            {
                                city_states[size_t(result_ids[i].id)].pt_w
                                    = city_states[size_t(result_ids[i].id)].pt_w;
                                cout<<"not enough meas"<<endl;
                            }

                        }

                    }
                }

                for (size_t i = 0; i < new_candidate.size(); i++)
                {
                    MOT temp;
                    temp.pt_w = measurements[size_t(new_candidate[i])].pt_w;
                    city_states.push_back(temp);
                }



                for (size_t i = 0; i < result_ids_obj.size(); i++)
                {
                    if(result_ids_obj[i].toofar)
                    {
                        cout<<"newbie here"<<endl;
                        new_candidate_obj.push_back(int(i));
                    }
                    else
                    {
                        if(result_ids_obj[i].id > int(objstates.size() - 1))//should be objectstate size
                        {
                            cout<<"newbie here"<<endl;
                            new_candidate_obj.push_back(int(i));
                        }
                        else
                        {
                            if(i < measurements_obj.size())
                                objstates[size_t(result_ids_obj[i].id)].pt_w
                                    = IIR_n * measurements_obj[i].pt_w
                                    + IIR_o * objstates[size_t(result_ids_obj[i].id)].pt_w;
                            else
                            {
                                objstates[size_t(result_ids_obj[i].id)].pt_w
                                    = objstates[size_t(result_ids_obj[i].id)].pt_w;
                                cout<<"not enough meas"<<endl;
                            }

                        }

                    }
                }

                for (size_t i = 0; i < new_candidate_obj.size(); i++)
                {
                    MOT temp;
                    temp.pt_w = measurements_obj[size_t(new_candidate_obj[i])].pt_w;
                    objstates.push_back(temp);
                }



            }
            else  //only flow with this for the very first time
            {
                if(detections.size() > 0)
                {
                    first_time = false;
                    for(size_t i = 0; i < detections.size(); i++)
                    {
                        MOT temp, temp_;
                        temp.id = int(i);
                        Eigen::Vector4d convert = c2w_cities (Eigen::Vector3d (
                                                           detections[i].boundingbox.x + detections[i].boundingbox.width / 2,
                                                           detections[i].boundingbox.y + detections[i].boundingbox.height/ 2,
                                                           detections[i].depth)
                                                       );
                        temp.pt_w = convert;
                        city_states.push_back(temp);

                        Eigen::Vector4d convert_obj = c2w(Eigen::Vector3d (
                                                              detections[i].boundingbox.x + detections[i].boundingbox.width / 2,
                                                              detections[i].boundingbox.y + detections[i].boundingbox.height/ 2,
                                                              detections[i].depth)
                                                          );
                        temp_.pt_w = convert_obj;
                        objstates.push_back(temp_);

                    }
                    enms_cities.got_stuff = true;
                    enms_obj.got_stuff = true;
                }
                else
                {
                    //nothing happen
                }
            }



    for(auto thing : city_states)
            {
                offb::obj enm;
                enm.X_c = thing.pt_w(0); enm.Y_c = thing.pt_w(1); enm.Z_c = thing.pt_w(2);
//                cout<<enm<<endl;
                enms_cities.points.push_back(enm);
            }
            for(auto thing : objstates)
            {

                offb::obj enm_obj;
                enm_obj.X_c = thing.pt_w(0); enm_obj.Y_c = thing.pt_w(1); enm_obj.Z_c = thing.pt_w(2);
        enms_obj.points.push_back(enm_obj);


            }
        if(what_i%20== 0)
        {


        }




        what_i++;
//cout<<"do we sway? "<<swayornot<<endl;
          cout<<enms_obj<<endl;
            cout<<city_states.size()<<endl;
        cout<<objstates.size()<<endl;
            Yolonet.display(frame);
////            cout<<first_time<<endl;
////            cout<<uavinfo.x<<endl;
////            cout<<uavinfo.y<<endl;
////            cout<<uavinfo.z<<endl;
////            cout<<uavinfo.ow<<endl;
////            cout<<uavinfo.ox<<endl;
////            cout<<uavinfo.oy<<endl;
////            cout<<uavinfo.oz<<endl;
//            cout<<endl;

        }

//        cout<<enms_cities<<endl;

        cv::waitKey(20);
        obj_wp_pub.publish(enms_cities);
        obj_ob_pub.publish(enms_obj);

        measurements.clear();
        measurements_obj.clear();

        enms_cities.points.clear();
        enms_obj.points.clear();

        new_candidate.clear();
        new_candidate_obj.clear();

        detections.clear();

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}
