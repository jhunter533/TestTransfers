int tbhSwitch=true;
double e;
double output;
double gain=.005;
double prevE=0;
double tbhV;
double targetRPM;

void tbhInputCallback(const std_msgs::Float64::ConstPtr& msg){
	targetRPM=msg->data;
	tbhSwtich=true;
}
int tbh(){
	ros::Rate rate(10);
	while(ros::ok()){
		if(tbhSwitch==true){
			e=targetRPM-RPM;
			output+=gain*e;
			if(output>1){
				output=1;
			}
			if(output<0){
				output=0;
			}
			if(std::signbit(e)!=std::signbit(prevE){
					output=.5*(output+tbhV);
					tbhV=output;
					prevE=e;
					}
					std_msgs::Float64 outputMsg;
					outputMsg.data=output;
					rpmPub.plublish(outputMsg);
			}
			else{
					output=0;
			}
			ros::spinOnce();
			rate.sleep();
			}
			return 1;
			}
			//rpmPub=nh.advertise<std_msgs::Float64>("/outputRPM",1);
			//ros::subscriber inputSub=nh.subscribe("/TargetRPM",1,tbhInputCallback);
			//std::thread tbhThread(tbh);
			//ros::sprin();
			//tbhThread.join();

