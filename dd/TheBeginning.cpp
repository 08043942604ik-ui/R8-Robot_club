
void RobotDrive::TheBeginning() {  
int ka = 0;
    
if(d != 0.5) {
   
    encoders_.setspeedturn();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    d = 0.5;
    
}   
    s = encoders_.GetTotalDistance();
    RCLCPP_INFO(get_logger(), "s : %lf", s);
    if(s >= 181.0) {
 
   encoders_.setspeedstop();

    set = 0;
    begin = false;
}
}//TheBeginning,end
