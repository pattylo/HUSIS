#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include "define.hpp"

#include <fstream>
#include <istream>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <numeric>
#include <chrono>
#include <iomanip>

using namespace std;
static vector<double> DDD;

typedef struct MOT
{
    cv::Mat state_of_1obj;//
    int id;
    int tracked_i;
    int missing;    
    Eigen::Vector4d pt_w;
}MOT;

typedef struct Match
{
    int id;
    bool toofar;
}Match;

class assign
{
    class maptool //for building the map + tour +
    {
      int q;

    public:
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cost_matrix, vis_matrix;
      maptool(vector<waypts> wps_of_agent)
      {
        q = int(wps_of_agent.size());
        cost_matrix.resize(q,q);

        vis_matrix.resize(q,q);
        vis_matrix.setZero();

        for(int i=0;i<q;i++)
        {
          for(int j=0;j<q;j++)
          {
            cost_matrix(i,j) = sqrt(pow(wps_of_agent[size_t(i)].x - wps_of_agent[size_t(j)].x,2)
                                    +pow(wps_of_agent[size_t(i)].y - wps_of_agent[size_t(j)].y,2)
                                    +pow(wps_of_agent[size_t(i)].z - wps_of_agent[size_t(j)].z,2));
          }
        }
        for(int i=0;i<cost_matrix.rows();i++)
        {
          for(int j=0;j<cost_matrix.cols();j++)
          {
            if(cost_matrix(i,j)!=0)
              vis_matrix(i,j) = 1/cost_matrix(i,j);
          }
        }

      }
      ~maptool(){}

    };

    class salesman //for each agent + tour + evaporation
    {
      vector<waypts> wps;
      int iteration;
      int q_ants;
      int q_cities;
      vector< Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> > best_route;
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cost, vis, vis_, trace;
      Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic> route, route_combined;

      vector<double> route_cost;
      int alpha = 1, beta = 4;

      double getRand()
      {
            double p = (rand() / (RAND_MAX + 1.0));
            return p;
      }

      void random_start()
      {
        //if ten ants, then ten random starting point
        srand(time(NULL));
        for (int i = 0;i<q_ants;i++)
        {
          int iRand = rand() % (q_cities-1);
          route(i,0) = iRand;
        }
      }

      void ant_tour_step() //go next city
      {
        for (int i=0;i<q_ants;i++)
        {
          vis_ = vis;

          for (int j=0;j<q_cities-1;j++)
          {
            int city_current = route(i,j);
            vis_.col(city_current).setZero();

            Eigen::Matrix <double, Eigen::Dynamic, Eigen::Dynamic> temp1, temp2, temp, trace_, p;
            temp1 = pow(trace.row(city_current).array(), alpha);
            temp2 = pow(vis_.row(city_current).array(), beta);
            temp = temp1.cwiseProduct(temp2);

            double s = temp.sum();
            p = 1/s * temp;

            double r = getRand();
            s = 0;
            for (int k=0;k<q_cities;k++)
            {
              s += p(k);
              if(s>r)
              {
                route(i,j+1) = k;
                break;
              }
            }
          }
        }
      }

      void trace_update()
      {
        vector<double> f;
        double temp = *min_element(route_cost.begin(), route_cost.end());
        for(size_t i=0;i<route_cost.size();i++)
        {
          f.push_back(route_cost.at(i) - 0.97 * temp);
        }

        for (int i=0;i<q_ants;i++)
        {
          for (int j=0;j<q_cities;j++)
          {
            double dt = 1/f.at(size_t(i));
            trace(route_combined(i,j), route_combined(i,j+1)) = (1- 0.15) * trace(route_combined(i,j), route_combined(i,j+1)) + dt;
          }
        }
      }

      void cal_cost()
      {
        double s;

        route_combined.resize(route.rows(), route.cols()+1);
        route_combined << route, route.col(1);

        for (int i=0;i< q_ants;i++)
        {
          s = 0;
          for (int j=0;j<q_cities;j++)
          {
            s += cost(route_combined(i,j),route_combined(i,j+1));
          }
          route_cost.push_back(s);
        }
      }

    public:
      salesman(vector<waypts> wps_of_agent, int loop)
      {
        this->wps = wps_of_agent;
        this->iteration = loop;
        this->q_ants = int(wps_of_agent.size());
        this->q_cities = int(wps_of_agent.size());

        cost.resize(this->q_cities, this->q_cities);
        vis.resize(this->q_cities, this->q_cities);
        vis_.resize(this->q_cities, this->q_cities);
        trace.resize(this->q_cities, this->q_cities);
        route.resize(this->q_cities, this->q_cities);

        route.setZero();
        trace.setConstant(0.0001);

        maptool mapp(wps);
        cost = mapp.cost_matrix;
        vis = mapp.vis_matrix;
      }
      ~salesman()
      {

      }

      void execute()
      {
        for(int i=0;i<iteration;i++)
        {
          random_start();

          ant_tour_step();

          cal_cost();

          trace_update();

          int which_route = std::min_element(route_cost.begin(),route_cost.end()) - route_cost.begin();
          double d = *min_element(route_cost.begin(), route_cost.end());
          route_cost.clear();

          best_route.push_back(route.row(which_route));

//          cout<<"This is the trace matrix: "<<endl<<trace<<endl<<endl;

        }
      }

      inline Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic> get()
      {
        return best_route.at(best_route.size()-1);
      }


    };

public:
    class mTSP //still got grouping
    {
      vector<waypts> wps;
      int iteration;
      Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic> result;

    public:
      mTSP(vector<waypts> inspect_pts, int loop)
      {
        this->wps = inspect_pts;
        this->iteration = loop;
      }
      ~mTSP()
      {

      }

      void execute()
      {
        salesman agent(wps, iteration);
        agent.execute();
        result = agent.get();
      }

      inline Eigen::Matrix <int, Eigen::Dynamic, Eigen::Dynamic> get() //m=3
      {
        return result;
      }

    };

    class hungarian
    {
        void stp1(int &step)
        {
            double minval;
            Eigen::MatrixXd minvals;

            minvals = cost.rowwise().minCoeff();
            for (int i = 0; i < cost.rows(); i++)
                {
                    minval = minvals(i, 0);
                    for (int j = 0; j < cost.cols(); j++)
                    {
                        cost(i, j) = cost(i, j) - minval;
                    }
                }
            step = 2;

        }//reduce with the minima of row and column

        void stp2(int &step)
        {
            for (int r = 0; r < cost.rows(); r++)
            {
                for (int c = 0; c < cost.cols(); c++)
                {
                    if (cost(r, c) == 0 && cover_row[r] == 0 && cover_col[c] == 0)
                    {
                        mask(r, c) = 1;
                        cover_row[r] = 1;
                        cover_col[c] = 1;
                    }
                }
            }
            for (int r = 0; r < cost.rows(); r++)
                cover_row[r] = 0;
            for (int c = 0; c < cost.cols(); c++)
                cover_col[c] = 0;
            step = 3;
        }

        void stp3(int &step)
        {
            int count = 0;
            for (int r = 0; r < cost.rows(); r++)
                for (int c = 0; c < cost.cols(); c++)
                    if (mask(r, c) == 1)
                        cover_col[c] = 1;
            for (int c = 0; c < cost.cols(); c++)
                if (cover_col[c] == 1)
                    count += 1;
            if (count == cost.cols() )
                step = 7;
            else
                step = 4;
        }

        void stp4(int &step)
        {
            int row = -1;
            int col = -1;
            bool done;
            done = false;

            while (!done)
            {
                find_a_zero(row, col);
                if (row == -1)
                {
                    done = true;
                    step = 6;
                }
                else
                {
                    mask(row, col) = 2;
                    if (star_in_row(row))
                    {
                        find_star_in_row(row, col);
                        cover_row[row] = 1;
                        cover_col[col] = 0;
                    }
                    else
                    {
                        done = true;
                        step = 5;
                        path_row_0 = row;
                        path_col_0 = col;
                    }
                }
            }
        }

        void stp5(int &step)
        {
            bool done;
            int row = -1;
            int col = -1;

            path_count = 1;
            path(path_count - 1, 0) = path_row_0;
            path(path_count - 1, 1) = path_col_0;
            done = false;
            while (!done)
            {
                find_star_in_col(path(path_count - 1, 1), row);
                if (row > -1)
                {
                    path_count += 1;
                    path(path_count - 1, 0) = row;
                    path(path_count - 1, 1) = path(path_count - 2, 1);
                }
                else
                    done = true;
                if (!done)
                {
                    find_prime_in_row(path(path_count - 1, 0), col);
                    path_count += 1;
                    path(path_count - 1, 0) = path(path_count - 2, 0);
                    path(path_count - 1, 1) = col;
                }
            }
            augment_path();
            clear_covers();
            erase_primes();
            step = 3;
        }

        void stp6(int &step)
        {
            double minval = DBL_MAX;
            find_min(minval);
            for (int r = 0; r < cost.rows(); r++)
                for (int c = 0; c < cost.cols(); c++)
                {
                    if (cover_row[r] == 1)
                        cost(r, c) += minval;
                    if (cover_col[c] == 0)
                        cost(r, c) -= minval;
                }
            //cout<<minval<<endl;
            step = 4;
        }

        void stp7()
        {
            for(int r = 0; r<cost.rows(); r++)
            {
                for (int c = 0; c<cost.cols();c++)
                {
                    if(mask(r,c) == 1 && copy(r,c) <= 1.4 /*&& copy(r,c) != 0*/   )
                        //need to modify the true or false threshold
                    {
                        Match temp = {c,false};
                        id_match.push_back(temp);
                    }
                    else if(mask(r,c) == 1 && copy(r,c) > 1.4 /*|| copy(r,c) == 0   )*/)
                    {
                        Match temp = {c,true};
                        id_match.push_back(temp);
                    }

                }
            }
        }

        void find_a_zero(int &row, int &col)
        {
            int r = 0;
            int c;
            bool done;
            row = -1;
            col = -1;
            done = false;

            while (!done)
            {
                c = 0;
                while (true)
                {
                    if (cost(r, c) == 0 && cover_row[r] == 0 && cover_col[c] == 0)
                    {
                        row = r;
                        col = c;
                        done = true;
                    }
                    c += 1;
                    if (c >= cost.cols() || done)
                        break;
                }
                r += 1;
                if (r >= cost.rows())
                    done = true;
            }
        }

        bool star_in_row(int row)
        {
            bool temp = false;
            for (int c = 0; c < cost.cols(); c++)
                if (mask(row, c) == 1)
                {
                    temp = true;
                    break;
                }
            return temp;
        }

        void find_star_in_row(int row, int &col)
        {
            col = -1;
            for (int c = 0; c < cost.cols(); c++)
            {
                if (mask(row, c) == 1)
                    col = c;
            }
        }

        void find_min(double &minval)
        {
            for (int r = 0; r < cost.rows(); r++)
                for (int c = 0; c < cost.cols(); c++)
                    if (cover_row[r] == 0 && cover_col[c] == 0)
                        if (minval > cost(r, c))
                            minval = cost(r, c);
        }

        void find_star_in_col(int col, int &row)
        {
            row = -1;
            for (int i = 0; i < cost.rows(); i++)
                if (mask(i, col) == 1)
                    row = i;
        }

        void find_prime_in_row(int row, int &col)
        {
            for (int j = 0; j < cost.cols(); j++)
                if (mask(row, j) == 2)
                    col = j;
        }

        void augment_path()
        {
            for (int p = 0; p < path_count; p++)
            {
                for (int p = 0; p < path_count; p++)
                {
                    int i = path(p, 0);
                    int j = path(p, 1);
                    if (mask(i, j) == 1)
                        mask(i, j) = 0;
                    else
                        mask(i, j) = 1;
                }

            }
        }

        void clear_covers()
        {
            for (int r = 0; r < cost.rows(); r++)
                cover_row[r] = 0;
            for (int c = 0; c < cost.cols(); c++)
                cover_col[c] = 0;
        }

        void erase_primes()
        {
            for (int r = 0; r < cost.rows(); r++)
                for (int c = 0; c < cost.cols(); c++)
                    if (mask(r, c) == 2)
                        mask(r, c) = 0;
        }
        int step = 1;
        Eigen::MatrixXd cost, mask, path, copy;
        vector<int> cover_row;
        vector<int> cover_col;
        int path_row_0, path_col_0, path_count;

        void cost_generate(vector<cv::Point3d> detected, vector<cv::Point3d> previous)
        {
            if(detected.size()==previous.size())
               {
                   cost.setZero(detected.size(), previous.size());
                   mask.setZero(detected.size(), previous.size());
                   cover_row = vector<int>(detected.size(),0);
                   cover_col = vector<int>(detected.size(),0);
                   path.setZero(detected.size()*2,2);
               }
               else if (detected.size()<previous.size())
               {
                   cost.setZero(previous.size(), previous.size());
                   mask.setZero(previous.size(), previous.size());
                   cover_row = vector<int>(previous.size(),0);
                   cover_col = vector<int>(previous.size(),0);
                   path.setZero(previous.size()*2,2);
               }
               else if (detected.size()>previous.size())
               {
                   cost.setZero(detected.size(), detected.size());
                   mask.setZero(detected.size(), detected.size());
                   cover_row = vector<int>(detected.size(),0);
                   cover_col = vector<int>(detected.size(),0);
                   path.setZero(detected.size()*2,2);
               }

               for (int i=0;i<detected.size();i++)
               {
                   for (int j=0;j<previous.size();j++)
                   {
                       cost (i,j) = cv::norm ( detected[i] - previous[j] );
                   }
               }
        }

    public:
        hungarian(){};
        ~hungarian(){};
        vector<Match> solution(vector<MOT>measured, vector<MOT>previous)
        {
            //return the corresponding ids
            id_match.clear();
            cv::Point3d temp;
            vector<cv::Point3d> detected_pts, previous_pts;

            for(auto thing : measured)
            {
                temp = cv::Point3d (thing.pt_w(0), thing.pt_w(1), thing.pt_w(2));
                detected_pts.push_back(temp);
            }

            for(auto thing : previous)
            {
                temp = cv::Point3d (thing.pt_w(0), thing.pt_w(1), thing.pt_w(2));
                previous_pts.push_back(temp);
            }

            cost_generate(detected_pts, previous_pts);
//            for(auto o:measured)
//            {
//                temp = cv::Point (o.at<float>(0), o.at<float>(1));
//                detected_pts.push_back(temp);
//            }
//            for(auto o:previous)
//            {
//                temp = cv::Point (o., o.state_of_1obj.at<float>(1));
//                previous_pts.push_back(temp);
//            }

//            cost_generate(detected_pts,previous_pts);
            copy = cost;
//            cout<<copy<<endl;
//            cout<<"hiiiii"<<endl;

            bool done = false;
            step = 1;

            while (!done)
            {
                //cout << endl << step << endl << endl;
                switch (step)
                {
                case 1:
                    stp1(step);
                    break;
                case 2:
                    stp2(step);
                    break;
                case 3:
                    stp3(step);
                    break;
                case 4:
                    stp4(step);
                    break;
                case 5:
                    stp5(step);
                    break;
                case 6:
                    stp6(step);
                    break;
                case 7:
                    stp7();
                    done = true;
        //            cout<<"bye"<<endl;
                    break;
                }
            }

            return id_match;
        }
        vector<Match> id_match;
    };
};

