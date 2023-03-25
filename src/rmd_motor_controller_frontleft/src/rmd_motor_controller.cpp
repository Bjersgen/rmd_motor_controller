#include <rmd_motor_controller/rmd_motor_controller.h>
using namespace std;
using namespace serial;
serial::Serial ser; //声明串口对象 
std_msgs::Int16 target_angle;
ros::Publisher singleAngle_pub;
std_msgs::Float32 singleAngle;
int16_t signleAngle = 0;
uint8_t spinDirection;
int pre_speed = 0;

void RS_angleControl_3(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1)
{
	uint8_t TxData[10] = {0};
	
	TxData[0] = 0x3E;
	TxData[1] = 0xA5;
	TxData[2] = Motor_ID;
	TxData[3] = 4;
	TxData[4] = Checksumcrc(TxData,0,4);
	TxData[5] = spinDirection;
	TxData[6] = *((uint8_t *)(&angleControl_1));
	TxData[7] = *((uint8_t *)(&angleControl_1)+1);
	TxData[8] = 0x00;
	TxData[9] = Checksumcrc(TxData,5,4);

	serialSend(TxData,10);
}

void RS_angleControl_4(uint8_t Motor_ID, uint8_t spinDirection, uint16_t angleControl_1, uint32_t maxSpeed)
{
	uint8_t TxData[14] = {0};
	
	TxData[0] = 0x3E;
	TxData[1] = 0xA6;
	TxData[2] = Motor_ID;
	TxData[3] = 0x08;
	TxData[4] = Checksumcrc(TxData,0,4);
	TxData[5] = spinDirection;
	TxData[6] = *((uint8_t *)(&angleControl_1));
	TxData[7] = *((uint8_t *)(&angleControl_1)+1);
	TxData[8] = 0x00;
	TxData[9] = *((uint8_t *)(&maxSpeed));
	TxData[10] = *((uint8_t *)(&maxSpeed)+1);
	TxData[11] = *((uint8_t *)(&maxSpeed)+2);
	TxData[12] = *((uint8_t *)(&maxSpeed)+3);
	TxData[13] = Checksumcrc(TxData,5,8);

	serialSend(TxData,14);
}
/**
  * @brief  send a speed control frame via RS485 bus
  * @param  motor ID ,1-32
  * @param  speed,  0.01dps
  * @retval null
  */
void RS_speedControl(uint8_t Motor_ID, int32_t speedControl)
{
	uint8_t TxData[10] = {0};
	TxData[0] = 0x3E;
	TxData[1] = 0xA2;
	TxData[2] = Motor_ID;
	TxData[3] = 4;
	TxData[4] = Checksumcrc(TxData,0,4);
	TxData[5] = *((uint8_t *)(&speedControl));
	TxData[6] = *((uint8_t *)(&speedControl)+1);
	TxData[7] = *((uint8_t *)(&speedControl)+2);
	TxData[8] = *((uint8_t *)(&speedControl)+3);
	TxData[9] = Checksumcrc(TxData,5,4);
	serialSend(TxData,10);
}

/**
  * @brief  send a Motor_Off frame via RS485 bus
  * @param  motor ID ,1-32
  * @retval null
  */
void RS_Motor_Off(uint8_t Motor_ID)
{
	uint8_t TxData[5] = {0};
	
	TxData[0] = 0x3E;
	TxData[1] = 0x80;
	TxData[2] = Motor_ID;
	TxData[3] = 0x00;
	TxData[4] = Checksumcrc(TxData,0,4);
	serialSend(TxData,5);
}

void RS_Motor_On(uint8_t Motor_ID)
{
	uint8_t TxData[5] = {0};
	
	TxData[0] = 0x3E;
	TxData[1] = 0x88;
	TxData[2] = Motor_ID;
	TxData[3] = 0x00;
	TxData[4] = Checksumcrc(TxData,0,4);

	serialSend(TxData,5);
}

void RS_readSingleAngle(uint8_t Motor_ID){
	uint8_t TxData[5] = {0};
	
	TxData[0] = 0x3E;
	TxData[1] = 0x94;
	TxData[2] = Motor_ID;
	TxData[3] = 0x00;
	TxData[4] = Checksumcrc(TxData,0,4);

	serialSend(TxData,5);
}

void decodeRecData(uint8_t *data){
	
	uint8_t crc = Checksumcrc(data,0,4);
	if(crc != data[4])
		cout << "cmd crc error !" << endl;
	else{
		if(data[1] == 0x94){
			crc = Checksumcrc(data,5,2);
			if(crc != data[7])
				cout << "data crc error !" << endl;
			singleAngle.data = (((uint16_t)data[6] << 8) | ((uint16_t)data[5]))*0.01;
			singleAngle_pub.publish(singleAngle);
			singlePositionControl_cb();
		}
	}
}

/**
  * @brief  Checksumcrc
  * @param  buf start byte
  * @param  crc length.
  * @retval Checksumcrc.
  */
uint8_t Checksumcrc(uint8_t *aData, uint8_t StartIndex, uint8_t DataLength)
{
	uint8_t crc = 0;
	uint8_t i = 0;
	for(i=StartIndex; i<(StartIndex + DataLength); i++)
	{
	crc +=  aData[i] ;	  
	}	
	return crc;
}

void serialSend(uint8_t *TxData, uint8_t len){
	if(ser.isOpen()){ 
		ser.write(TxData,len);   //发送串口数据
		// cout << "serialsend: ";
      	// for (size_t i = 0; i < len; i++){
		// 	printf("%x ", (uint8_t)TxData[i]);
		// }
		// cout << endl;
	} 
	else{
		ROS_INFO_STREAM("Serial Port is close");
	}
}

void serialRec_cb(const ros::TimerEvent &event){
	// cout << "time " << endl;
	if(ser.available()){ 
		string result;
     	result = ser.read(ser.available()); 
		 
		const int size = result.size();
		uint8_t data[size];
		// cout << "serialRec : ";
      	for (size_t i = 0; i < size; i++){
			data[i] = (uint8_t)result[i];
			// printf("%x ", data[i]); 
		}
		// cout << endl;

		decodeRecData(data);
	}
}
void readSingleAngle_cb(const ros::TimerEvent &event){
	RS_readSingleAngle(1);
}

void cmdRec_cb_frontright(const std_msgs::Int16::ConstPtr &msg)
{
	int speed = msg->data;
	RS_speedControl(4,5000);	
}
void cmdRec_cb_frontleft(const std_msgs::Int16::ConstPtr &msg)
{
	int speed = msg->data;
	RS_speedControl(3,-5000);	//to be fixed
}
void cmdRec_cb_rearleft(const std_msgs::Int16::ConstPtr &msg)
{
	int speed = msg->data;
	RS_speedControl(2,-5000);	
}
void cmdRec_cb_rearright(const std_msgs::Int16::ConstPtr &msg)
{
	int speed = msg->data;
	RS_speedControl(1,5000);	
	std::cout<<"fuck"<<std::endl;
}
void singlePositionControl_cb()
{
	if(std::abs(singleAngle.data - 10.0) < 0.2){
		RS_angleControl_4(1,0,13000,10000);
	}
	else if(std::abs(singleAngle.data - 130.0) < 0.2){
		RS_angleControl_4(1,1,1000,10000);
	}
}

int main(int argc, char** argv){

	//初始化节点 
	ros::init(argc, argv, "rmd_motor_controller_frontleft_node"); 
	//声明节点句柄 

	ros::NodeHandle nh; 

	ros::Subscriber cmdRec_sub_frontleft = nh.subscribe<std_msgs::Int16>("/rmd/cmd/frontleft", 10, cmdRec_cb_frontleft); 
	ros::Subscriber cmdRec_sub_frontright = nh.subscribe<std_msgs::Int16>("/rmd/cmd/frontright", 10, cmdRec_cb_frontright); 
	ros::Subscriber cmdRec_sub_rearleft = nh.subscribe<std_msgs::Int16>("/rmd/cmd/rearleft", 10, cmdRec_cb_rearleft); 
	ros::Subscriber cmdRec_sub_rearright = nh.subscribe<std_msgs::Int16>("/rmd/cmd/rearright", 10, cmdRec_cb_rearright); 
	
	//ros::Timer serialRec_timer_ = nh.createTimer(ros::Duration(0.001), serialRec_cb);
	//ros::Timer readSingleAngle_timer_ = nh.createTimer(ros::Duration(0.03), readSingleAngle_cb);
	//singleAngle_pub = nh.advertise<std_msgs::Float32>("/rmd/singleAngle", 1000); 
	try{ 
	//设置串口属性，并打开串口 
		ser.setPort("/dev/ttyCH341USB0"); 
		ser.setBaudrate(115200); 
		serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
		ser.setTimeout(to); 
		ser.open(); 
	} 
	catch (serial::IOException& e){ 
		ROS_ERROR_STREAM("Unable to open port ");
	} 

	//检测串口是否已经打开，并给出提示信息 
	if(ser.isOpen()){ 
		cout << "Serial Port initialized" << endl; 
	} 
	else{ 
		cout << "fuck" << endl; 
		return -1; 
	} 
	ros::Rate loop_rate(50);
	RS_Motor_On(1);
	RS_Motor_On(2);
	RS_Motor_On(3);
	RS_Motor_On(4);
	ros::Duration(0.5).sleep(); 
	
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	//RS_angleControl_4(1,1,1000,10000);
	//ros::spin();
	
	RS_Motor_Off(1);
	return 0;
}
