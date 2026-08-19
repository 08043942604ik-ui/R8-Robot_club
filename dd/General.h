#pragma once

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "Lidar.h"

class Generalva {
public : 
Generalva() = default;



void LWaitingThread() {
        
lidar_->stopLidar();
if(Turna_thread.joinable()) {
        Turna_thread.join(); //threadが来るまで待って回収する
    }//join()起動してるか確認



}

std::thread Turna_thread;
private : 

std::shared_ptr<Lidarnode> lidar_;


};
