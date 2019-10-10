#include <iostream>
#include <sstream>  

#include <ros/ros.h> 
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/Config.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class GenerateCaliSamples
{
public:
    GenerateCaliSamples(ros::NodeHandle nh)
    {	
	nh.param<int>("img_mode", _img_mode, 1);
   	nh.param<int>("sample_mode", _sample_mode, 0);
	nh.param<int>("overall_count_left", _overall_count_left, 1);
    	nh.param<int>("overall_count_right", _overall_count_right, 1);
    	nh.param<int>("overall_count", _overall_count, 1);
	nh.param<double>("record_interval", _record_interval, 0.1);

        _exp_base = 20;
        
        // load f out path
        char* env = getenv("HOME");

        image_transport::ImageTransport it(nh);
        _param_sub = nh.subscribe("/camera/realsense2_camera_manager/parameter_updates", 2, 
                                  &GenerateCaliSamples::paramCallback, this);

        if(_img_mode == 0)
        {
            _infra_img_sub1 = it.subscribe("/camera/infra1/image_rect_raw", 1, &GenerateCaliSamples::infra1ImageCallback, this);
            _infra_img_sub2 = it.subscribe("/camera/infra2/image_rect_raw", 1, &GenerateCaliSamples::infra2ImageCallback, this); 
            _exp_name = "rs435_depth_exposure";
            _fout_path_left = std::string(env) + "/image_collections/leftimages/";
            _fout_path_right = std::string(env) + "/image_collections/rightimages/";
            printf(" Img Mode:: Infra\n");
        }
        else if(_img_mode == 1)
        {
            _rgb_img_sub = it.subscribe("/camera/color/image_raw", 1, &GenerateCaliSamples::rgbImageCallback, this); 
            _exp_name = "rs435_color_exposure";
            _fout_path = std::string(env) + "/image_collections/leftimages/";
            printf(" Img Mode:: RGB\n");
        }
        else
        {
            printf(" Unknown Img Mode:: 0 infra 1 rgb\n");
            exit(1);
        }

        _exp_value = 0;
        _sample_count = 0;
        _param_init = false;
        ros::Time _first_record_time = ros::Time(0);
        _latest_record_time = ros::Time(0);
        _latest_record_time_left = ros::Time(0);
        _latest_record_time_right = ros::Time(0);

        // exp out file
        std::string exp_out_file_name = _fout_path + "times.txt";
        std::string exp_out_file_name_left = _fout_path_left + "times.txt";
        std::string exp_out_file_name_right = _fout_path_right + "times.txt";
        _exp_outf = fopen(exp_out_file_name.c_str(), "w");
        _exp_outf_left = fopen(exp_out_file_name_left.c_str(), "w");
        _exp_outf_right = fopen(exp_out_file_name_right.c_str(), "w");
    }
    ~GenerateCaliSamples() {fclose(_exp_outf);}

protected:
    void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            printf("cv_bridge exception: %s\n", e.what());
            return;
        }
        
        ros::Time curr_t = ros::Time::now();
        if ((curr_t - _latest_record_time).toSec() > _record_interval)
        {
            fprintf(stderr, "->");
            std::stringstream file_name;
            file_name << _fout_path << _overall_count << "th_img" << ".jpg";
            cv::imwrite(file_name.str(), cv_ptr->image);

            // write exp value to file
            fprintf(_exp_outf, "%d %1.6lf %f\n", _overall_count, curr_t.toSec(), (double)_exp_value / 1000.0);
            fflush(_exp_outf);

            _overall_count++;
            _latest_record_time = curr_t;
        }
    }


    void infra1ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    { 
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            printf("cv_bridge exception: %s\n", e.what());
            return;
        }
        
        ros::Time curr_t = ros::Time::now();
        if ((curr_t - _latest_record_time_left).toSec() > _record_interval)
        {
            fprintf(stderr, "->");
            std::stringstream file_name_left;
            file_name_left << _fout_path_left << _overall_count_left << "th_img" << ".jpg";
            cv::imwrite(file_name_left.str(), cv_ptr->image);

            // write exp value to file
            fprintf(_exp_outf_left, "%d %1.6lf %f\n", _overall_count_left, curr_t.toSec(), (double)_exp_value / 1000.0);
            fflush(_exp_outf_left);

            _overall_count_left++;
            _latest_record_time_left = curr_t;
        }
    }

    void infra2ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    { 
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            printf("cv_bridge exception: %s\n", e.what());
            return;
        }
        
        ros::Time curr_t = ros::Time::now();
        if ((curr_t - _latest_record_time_right).toSec() > _record_interval)
        {
            fprintf(stderr, "->");
            std::stringstream file_name_right;
            file_name_right << _fout_path_right << _overall_count_right << "th_img" << ".jpg";
            cv::imwrite(file_name_right.str(), cv_ptr->image);

            // write exp value to file
            fprintf(_exp_outf_right, "%d %1.6lf %f\n", _overall_count_right, curr_t.toSec(), (double)_exp_value / 1000.0);
            fflush(_exp_outf_right);

            _overall_count_right++;
            _latest_record_time_right = curr_t;
        }
    }

    void paramCallback(const dynamic_reconfigure::ConfigConstPtr& msg)
    {
        // get current exposure value
        for(unsigned int i=0; i<msg->ints.size(); ++i)
        {
            if(msg->ints[i].name.compare(_exp_name) == 0)
            {
                int value = msg->ints[i].value * 20;
                _exp_value = value;
                break;
            }
        }

        // TODO get rs435_color_gain, rs435_color_gamma, rs435_color_enable_auto_exposure ...
    }
    
    ros::Subscriber _param_sub;
    image_transport::Subscriber _infra_img_sub1;
    image_transport::Subscriber _infra_img_sub2;
    image_transport::Subscriber _rgb_img_sub;

    int _img_mode;
    int _sample_mode;

    bool _param_init;
    int _sample_count;
    int _overall_count;
    int _overall_count_left;
    int _overall_count_right;

    std::string _exp_name;
    int _exp_base;
    int _exp_value;
    int _exp_next_value;

    ros::Time _first_record_time;
    ros::Time _latest_record_time;
    ros::Time _latest_record_time_left;
    ros::Time _latest_record_time_right;
    double _record_interval;

    // for output
    std::string _fout_path;
    std::string _fout_path_left;
    std::string _fout_path_right;

    FILE* _exp_outf;
    FILE* _exp_outf_left;
    FILE* _exp_outf_right;
};

int main(int argc, char **argv)
{ 
   ros::init(argc, argv, "generate_cali_samples");
   ros::NodeHandle nh("~");

   GenerateCaliSamples* _gcs = new GenerateCaliSamples(nh);
   
   ros::spin(); 

   delete _gcs;
}

