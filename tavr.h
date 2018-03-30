/*
 * tavr.h
 *
 *  Created on: Jan 21, 2018
 *      Author: js
 */

#ifndef TAVR_H_
#define TAVR_H_

#include "tavr_traffic_pkt.h"


#include <agent.h>
#include <mobilenode.h>
#include <trace.h>
#include <classifier.h>
#include <string>
#include <classifier-port.h>

using namespace std;




#define CURRENT_TIME		Scheduler::instance().clock();



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

class TAVRagent : public Agent{

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
	TAVR_pktTimer 	tavr_timer_;
	u_int32_t	inner_seqno_;




	//vehicular info
	double 		vehicle_info_updateTime_LIST_[VEHICULAR_AMOUNT]; //
	nsaddr_t 	vehicle_ip_LIST_[VEHICULAR_AMOUNT]; //device ID----vehicle
//	nsaddr_t 	all_ip_; //device ID----vehicle
	double 		vehicle_position_x_LIST_[VEHICULAR_AMOUNT]; //
	double 		vehicle_position_y_LIST_[VEHICULAR_AMOUNT]; //
	double 		vehicle_speed_LIST_[VEHICULAR_AMOUNT]; //
	u_int8_t 	vehicle_direction_LIST_[VEHICULAR_AMOUNT]; //
	bool 		junction_LIST_[VEHICULAR_AMOUNT];
	nsaddr_t 	bs_ip_LIST_[VEHICULAR_AMOUNT];
	u_int8_t 	junc_row_prev_list[VEHICULAR_AMOUNT]; //
	u_int8_t 	junc_col_prev_list[VEHICULAR_AMOUNT]; //
	u_int8_t 	junc_row_next_list[VEHICULAR_AMOUNT]; //
	u_int8_t 	junc_col_next_list[VEHICULAR_AMOUNT]; //




	//
	MobileNode *node_;
	Trace *logtarget_;
	PortClassifier *dmux_;


	//otcl command methods for configuration parameter
	bool		conf_vehicle_;
	double		conf_scenario_Width_;
	double		conf_scenario_Height_;
	int			conf_scenario_rowc;
	int			conf_scenario_colc;
	int			conf_scenario_vehicular_amout;
	int			conf_scenario_base_amout;
	int			conf_node_id;
	int			conf_junc_row_bs;
	int			conf_junc_col_bs;

	double		conf_startx_;
	double		conf_starty_;
	int			conf_base_rowc;
	int			conf_base_colc;
	double		**junc_info_arrayX;
	double		**junc_info_arrayY;
	int			comm_id;


	int			conf_test_INET;//bits VALID: six bits, first FLAG, second BROADCAST, third SUBNET, last INTERNET

	int         conf_test_AXIS_ip;


	bool		debug_flag;


protected:

	inline nsaddr_t vehicle_ip() {
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
	inline nsaddr_t bs_ip() {
		return bs_ip_;
	}


	inline double* vehicle_info_updateTime_list() {
		return vehicle_info_updateTime_LIST_;
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
	inline u_int8_t* junc_row_prev_List() {
		return junc_row_prev_list;
	}
	inline u_int8_t* junc_col_prev_List() {
		return junc_col_prev_list;
	}
	inline u_int8_t* junc_row_next_List() {
		return junc_row_next_list;
	}
	inline u_int8_t* junc_col_next_List() {
		return junc_col_next_list;
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
	TAVRagent(nsaddr_t);
	~TAVRagent();
	int command(int argc, const char*const* argv);
	void recv(Packet*, Handler* callback = 0);
	void send_tavr_pkt();
	void reset_tavr_timer();
	void tavr_forward_data(Packet*);
	void recv_tavr_pkt(Packet*);
	void recv_Wired(Packet*);
	void recv_Hello(Packet*);
	void init_juncInfo();
	void print_bs();

	void base_sendBeacon();


//---------------test part ----------------------


};



#endif /* TAVR_H_ */
