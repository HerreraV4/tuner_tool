#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/parameter.hpp"

using namespace std::chrono_literals;
bool flg_init = true;


const int num_parameters = 13;
const int num_gains = 12;

float scales[num_gains] = {1000.0, 10000.0, 10000.0, 1000.0, 10000.0, 10000.0, 1000.0, 10000.0, 10000.0, 1000.0, 10000.0, 10000.0};

int parameter_init[num_gains];

int limit_bar = 10000;

std::string parameters[num_parameters] = {"KP_Z", "KD_Z", "KI_Z", "KP_R", "KD_R", "KI_R", "KP_P", "KD_P", "KI_P", "KP_Y", "KD_Y", "KI_Y", "SAVE"};

class interface : public rclcpp::Node {
    public:
        interface() : Node("interface") {
            RCLCPP_INFO(this->get_logger(), "Start interface");

            YAML::Node config = YAML::LoadFile(file);
            for(int k = 0; k < num_parameters - 1; k++)
                parameter_init[k] = int(config["/robot_local"]["ros__parameters"][parameters[k]].as<float>() * scales[k]);
                        
            client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/robot_local/set_parameters");
            while (!client_->wait_for_service(1s)) {
               RCLCPP_INFO(this->get_logger(), "Waiting for the service...");
            }
            RCLCPP_INFO(this->get_logger(), "Service found!");
    
            cv::namedWindow("Gains", cv::WINDOW_AUTOSIZE);
            cv::setMouseCallback("Gains", mouseCallbackSave, this);
            cv::createTrackbar(parameters[0], "Gains", &kp_z, limit_bar, trackbarCallbackKpz, this);
            cv::createTrackbar(parameters[1], "Gains", &kd_z, limit_bar, trackbarCallbackKdz, this);
            cv::createTrackbar(parameters[2], "Gains", &ki_z, limit_bar, trackbarCallbackKiz, this);
            cv::createTrackbar(parameters[3], "Gains", &kp_r, limit_bar, trackbarCallbackKpr, this);
            cv::createTrackbar(parameters[4], "Gains", &kd_r, limit_bar, trackbarCallbackKdr, this);
            cv::createTrackbar(parameters[5], "Gains", &ki_r, limit_bar, trackbarCallbackKir, this);
            cv::createTrackbar(parameters[6], "Gains", &kp_p, limit_bar, trackbarCallbackKpp, this);
            cv::createTrackbar(parameters[7], "Gains", &kd_p, limit_bar, trackbarCallbackKdp, this);
            cv::createTrackbar(parameters[8], "Gains", &ki_p, limit_bar, trackbarCallbackKip, this);
            cv::createTrackbar(parameters[9], "Gains", &kp_y, limit_bar, trackbarCallbackKpy, this);
            cv::createTrackbar(parameters[10], "Gains", &kd_y, limit_bar, trackbarCallbackKdy, this);
            cv::createTrackbar(parameters[11], "Gains", &ki_y, limit_bar, trackbarCallbackKiy, this);
            
            
            for(int k = 0; k < num_gains; k++)
                cv::setTrackbarPos(parameters[k], "Gains", parameter_init[k]);
            
            image = cv::imread("/home/mk5/ros2_ws/src/tuner_tool/src/options.png", cv::IMREAD_COLOR);
            if (image.empty()) {
                RCLCPP_INFO(this->get_logger(), "Could not read image");
                return;
            }
    
            cv::imshow("Gains", image);
    
            auto timer_callback = [this]() -> void {
                cv::imshow("Gains", image);
                cv::waitKey(5);
                flg_init = false;
            };
            timer_ = this->create_wall_timer(10ms, timer_callback);
        }
    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::string file = "/home/mk5/ros2_ws/src/tuner_tool/cfg/gains.yaml";

        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;

        void set_param(const std::string &name, double value) {
            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            rclcpp::Parameter param(name, value);
            request->parameters.push_back(param.to_parameter_msg());
        
            auto future = client_->async_send_request(request,
                [this, name, value](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture result) {
                    try {
                        result.get();
                        RCLCPP_INFO(this->get_logger(), "Parameter %s has changed to: %.2f", name.c_str(), value);
                    } catch (const std::exception &e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to change parameter %s: %s", name.c_str(), e.what());
                    }
                }
            );
        }


        void modifyYaml(const std::string &filename, const std::string &param, double value) {
            YAML::Node config = YAML::LoadFile(filename);
            if (config["/robot_local"]["ros__parameters"][param]) {
                config["/robot_local"]["ros__parameters"][param] = value;
                std::ofstream fout(filename);
                fout << config;
                std::cout << "Parameter " << param << " recorded: " << std::endl;
            } else {
                std::cerr << "Parameter " << param << " unrecognized" << std::endl;
            }
        }
        static void resetCallback(int state, void *pointer)
        {
            std::cout << "Reset " << std::endl;
        }
        static void mouseCallbackSave(int event, int x, int y, int flags, void* userdata)
        {
            auto *self = static_cast<interface *>(userdata);
            if(x >= 150)
            {
                
                if(event == cv::EVENT_LBUTTONDOWN)
	            {
                    self->flg_save = 1.0;
                    self->set_param("SAVE", flg_save);
                    std::cout << "Data recorded " << std::endl;
	            }
                if(event == cv::EVENT_LBUTTONUP)
	            {
                    self->flg_save = 0.0;
                    self->set_param("SAVE", flg_save);
	            }
            }
            
        }
        static void trackbarCallbackKpz(int pos, void *userdata) {
            set_save(parameters[0], pos/scales[0], userdata);
        }
        static void trackbarCallbackKdz(int pos, void *userdata) {
            set_save(parameters[1], pos/scales[1], userdata);
        }
        static void trackbarCallbackKiz(int pos, void *userdata) {
            set_save(parameters[2], pos/scales[2], userdata);
        }
        static void trackbarCallbackKpr(int pos, void *userdata) {
            set_save(parameters[3], pos/scales[3], userdata);
        }
        static void trackbarCallbackKdr(int pos, void *userdata) {
            set_save(parameters[4], pos/scales[4], userdata);
        }
        static void trackbarCallbackKir(int pos, void *userdata) {
            set_save(parameters[5], pos/scales[5], userdata);
        }
        static void trackbarCallbackKpp(int pos, void *userdata) {
            set_save(parameters[6], pos/scales[6], userdata);
        }
        static void trackbarCallbackKdp(int pos, void *userdata) {
            set_save(parameters[7], pos/scales[7], userdata);
        }
        static void trackbarCallbackKip(int pos, void *userdata) {
            set_save(parameters[8], pos/scales[8], userdata);
        }
        static void trackbarCallbackKpy(int pos, void *userdata) {
            set_save(parameters[9], pos/scales[9], userdata);
        }
        static void trackbarCallbackKdy(int pos, void *userdata) {
            set_save(parameters[10], pos/scales[10], userdata);
        }
        static void trackbarCallbackKiy(int pos, void *userdata) {
            set_save(parameters[11], pos/scales[11], userdata);
        }
        static void set_save(std::string name_parameter, float value, void *userdata)
        {
            auto *self = static_cast<interface *>(userdata);
            self->set_param(name_parameter, value);
            self->modifyYaml(self->file, name_parameter, value);
        }
        
        

        static cv::Mat image;
        static int flg_save;
        
        static int kp_z, kd_z, ki_z;
        static int kp_r, kd_r, ki_r;
        static int kp_p, kd_p, ki_p;
        static int kp_y, kd_y, ki_y;
};




cv::Mat interface::image;
int interface::flg_save = 0;

int interface::kp_z = 0;
int interface::kd_z = 0;
int interface::ki_z = 0;

int interface::kp_r = 0;
int interface::kd_r = 0;
int interface::ki_r = 0;

int interface::kp_p = 0;
int interface::kd_p = 0;
int interface::ki_p = 0;

int interface::kp_y = 0;
int interface::kd_y = 0;
int interface::ki_y = 0;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<interface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
