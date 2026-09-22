#include "accounting.h"
#include "Encoder.h"
#include "button.h"
#include "Robotmainmove.h" 
#include "General.h"

SVE::SVE(std::shared_ptr<Encoderma> encoder, std::shared_ptr<BFE> buttons) : Node("system_monitor"),
            buttons_(buttons),
            robot_run(nullptr),
            encoder_1(encoder)
            {

        timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
               
                if(buttons_->get_stop_button_state() && robot_run) {
            status = true;
            
               robot_run->request_stop();

               bo = buttons_->reset_button_states();
                encoder_1->StopAll();    
                general_values_->LWaitingThread();

                executor.remove_node(robot_run);
                 
                 robot_run.reset();

               
                 if(!bo) {
             
                rclcpp::shutdown();
                
                return;
                }
 
                bo = false;
                status = false;
                }

               
               
                if(buttons_->get_start_button_state() && !robot_run) {
               status = true;
                robot_run = std::make_shared<Robotmain>(encoder_1);
                executor.add_node(robot_run);
                bo = buttons_->reset_button_states();
                
                if(!bo) {
                encoder_1->StopAll();    
                general_values_->LWaitingThread();

                executor.remove_node(robot_run);
                 
                 robot_run.reset();
                
                rclcpp::shutdown();
                
                return;
                }
 

                bo = false;
               status = false;
            }





            }
        );
    general_values_ = std::make_shared<Generalva>();
    
    }

SVE::~SVE()  
{
    if (robot_run) {
        encoder_1->StopAll();
        general_values_->LWaitingThread();
        
        executor.remove_node(robot_run);
        robot_run.reset();
    }
}


void SVE::executorAdder(rclcpp::Node::SharedPtr node) {
    executor.add_node(node);
}
    
void SVE::executorRemover(rclcpp::Node::SharedPtr node0) {
    executor.remove_node(node0);
}

void SVE::executorSpin_some() {
    executor.spin_some();
}

bool SVE::check_status() {
    return status;
}
