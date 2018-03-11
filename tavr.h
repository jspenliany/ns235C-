/*
 * tavr.h
 *
 *  Created on: Jan 21, 2018
 *      Author: js
 */

#ifndef TAVR_H_
#define TAVR_H_

#include "tavr_pkt.h"


#include <agent.h>
#include <mobilenode.h>
#include <trace.h>
#include <classifier.h>




#define CURRENT_TIME		Scheduler::instance().clock();
#define JITTER				(Random::uniform()*0.5)

#define	VEHICULAR_INFO_PTB_TIMEOUT		10			//delay before send out msg to wired node for future broadcast (6 seconds)
#define	VEHICULAR_INFO_PTB_RECV_COUNTER	8			//msgs must be received before send to wired node for future broadcast,
													//including the above, collaboratively decide when to send info to all vehicular

#define TRAFFIC_ROW_AMOUNT				8
#define	TRAFFIC_COLUMN_AMOUNT			8



class TAVRagent;

class TAVR_pktTimer  : public TimerHandler {
public:
	TAVR_pktTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

class TAVRHello_pktTimer  : public TimerHandler {
public:
	TAVRHello_pktTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

/**
 * for vehicular binding bs-ip
 * */
class TAVRreplyHello_pktTimer  : public TimerHandler {
public:
	TAVRreplyHello_pktTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

class TAVRagent : public Agent{

	friend class TAVRHello_pktTimer;
	friend class TAVRreplyHello_pktTimer;
	friend class TAVR_pktTimer;

private:
	//self info
	nsaddr_t 	vehicle_ip_; //device ID----vehicle
	nsaddr_t 	all_ip_; //device ID----vehicle
	double 		vehicle_position_x_; //current position for instant hello msg
	double 		vehicle_position_y_; //
	double 		vehicle_speed_; //
	u_int8_t 	vehicle_direction_; //
	bool 		junction_;
	nsaddr_t 	bs_ip_;

	//hello msgs encounter
	u_int8_t	hello_amount_;
	double		firstHellorecv_ts_;

	//msg timer
	TAVRHello_pktTimer 			hello_timer_;
	TAVRreplyHello_pktTimer		reply_hello_timer_;




	//vehicular info
	nsaddr_t 	vehicle_ip_LIST_[VEHICULAR_AMOUNT]; //device ID----vehicle
//	nsaddr_t 	all_ip_; //device ID----vehicle
	double 		vehicle_position_x_LIST_[VEHICULAR_AMOUNT]; //
	double 		vehicle_position_y_LIST_[VEHICULAR_AMOUNT]; //
	double 		vehicle_speed_LIST_[VEHICULAR_AMOUNT]; //
	u_int8_t 	vehicle_direction_LIST_[VEHICULAR_AMOUNT]; //
	bool 		junction_LIST_[VEHICULAR_AMOUNT];
	nsaddr_t 	bs_ip_LIST_[VEHICULAR_AMOUNT];

	//traffic calculation
	u_int16_t		map_traffic[TRAFFIC_ROW_AMOUNT+1][TRAFFIC_COLUMN_AMOUNT+1];//for road segments
	cjunction_info	map_junc_list[TRAFFIC_ROW_AMOUNT*TRAFFIC_COLUMN_AMOUNT];// for junctions gps info
	double			map_X;
	double			map_Y;





	//
	MobileNode *node_;
	Trace *logtarget_;
	NsObject *port_dmux_;


	//otcl command methods for configuration parameter
	bool		conf_vehicle_;
	double		conf_scenario_Width_;
	double		conf_scenario_Height_;
	int			conf_scenario_rowc;
	int			conf_scenario_colc;
	int			conf_scenario_vehicular_amout;
	int			conf_scenario_base_amout;
	int			conf_node_id;

	int			conf_test_INET;//bits VALID: six bits, first FLAG, second BROADCAST, third SUBNET, last INTERNET

	int         conf_debug;


protected:
	inline nsaddr_t& vehicle_ip() {
		return vehicle_ip_;
	}
	inline double& vehicle_position_x() {
		return vehicle_position_x_;
	}
	inline double& vehicle_position_y() {
		return vehicle_position_y_;
	}
	inline double& vehicle_speed() {
		return vehicle_speed_;
	}
	inline u_int8_t& vehicle_direction() {
		return vehicle_direction_;
	}
	inline bool& junction() {
		return junction_;
	}
	inline nsaddr_t& bs_ip() {
		return bs_ip_;
	}


	inline nsaddr_t* vehicle_ip_LIST() {
		return vehicle_ip_LIST_;
	}
	inline double* vehicle_position_x_LIST() {
		return vehicle_position_x_LIST_;
	}
	inline double* vehicle_position_y_LIST() {
		return vehicle_position_y_LIST_;
	}
	inline double* vehicle_speed_LIST() {
		return vehicle_speed_LIST_;
	}
	inline u_int8_t* vehicle_direction_LIST() {
		return vehicle_direction_LIST_;
	}
	inline bool* junction_LIST() {
		return junction_LIST_;
	}
	inline nsaddr_t* bs_ip_LIST() {
		return bs_ip_LIST_;
	}
	inline u_int8_t& hello_amount() {
		return hello_amount_;
	}
	inline double& firstHellorecv_ts() {
		return firstHellorecv_ts_;
	}
	inline bool	triggerReplyHello(double delay){
		double tmp = CURRENT_TIME;
		return ((firstHellorecv_ts_ + delay) < tmp);
	}
	void update_vehicular_info();




public:
	double* 	cvehicle_position_x_LIST();
	double* 	cvehicle_position_y_LIST();
	double* 	cvehicle_speed_LIST();
	u_int8_t* 	cvehicle_direction_LIST();
	bool* 		cjunction_LIST();
	nsaddr_t* 	cbs_ip_LIST();

public:
	TAVRagent(nsaddr_t);
	int command(int argc, const char*const* argv);
	void recv(Packet*, Handler* callback = 0);
	void sendHello();
	void reset_Hello_timer();
	void replyHello();
	void reset_replyHello_timer();
	void recv_TAVR(Packet*);
	void forward_data(Packet*);
	void recv_Hello(Packet*);
	void recv_Wired(Packet*);
	void recv_Rabbitall();


	void base_sendBeacon();


//---------------test part ----------------------
	void print_bs();
protected:
	void print_addr(nsaddr_t);
	int  get_indexFromaddr(nsaddr_t);


};



#endif /* TAVR_H_ */
