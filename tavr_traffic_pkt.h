/*
 * tavr_traffic_pkt.h
 *
 *  Created on: Mar 16, 2018
 *      Author: js
 */

#ifndef AVANET_TAVR_TRAFFIC_PKT_H_
#define AVANET_TAVR_TRAFFIC_PKT_H_

#include <packet.h>
#include <random.h>


#define TAVRTYPE_HELLO  	0x01
#define TAVRTYPE_WIRED   	0x02

#define WIRED_NODE	0x00
#define BASE_NODE	0x01
#define MOBILE_NODE	0x02
#define COMM_NODE	0x04

#define VEH_HELLO_VALID_TIMES	2.2	//the vehicle have NOT report its info to wired for the laest xxx periods of timer
#define INFO_UP_INTERVAL_RATE	1.2 //the rate between valid info_up_interval and the info_up_timer_interval


#define WIRED_DIAMETER		7
#define	NETWORK_DIAMETER	50


#define		BASE_STATION_AMOUNT	20						//how many base sink nodes in the network

#define		VEHICULAR_AMOUNT	200					//how many vehicular in the network
#define		HELLO_INTERVAL		2						//interval for send out Hello msg( second)
#define 	TAVR_INTERVAL		20
#define		TAVR_MAX_HOPS		50						//hops that msg will tx
#define		INFO_UPDATE_DELAY	10000					// ms

#define TAVR_JITTER		(Random::uniform()*0.7)

struct hdr_route_tavr{
	u_int8_t	seqno_;
	static int offset_;
	inline static int& offset() {
		return offset_;
	}
	inline static hdr_route_tavr* access(const Packet* p) {
		return (hdr_route_tavr*) p->access(offset_);
	}
};

/**
 * from vehicular to wired
 * */
struct hdr_veh_hello{
	//to sink
	double		hello_tstamp_;	//msg Sent timestamp
	int16_t 	vehicle_id_; //device ID----vehicle----save storage
	int16_t 	wired_id_; //device ID----vehicle----save storage
	float 		vehicle_position_x_; //
	float	 	vehicle_position_y_; //
	float		vehicle_speed_; //----save storage
	u_int8_t 	vehicle_direction_; //

	//added by wired
	int16_t	bs_id_;
	u_int8_t 	junc_row_prev_; //
	u_int8_t 	junc_col_prev_; //
	u_int8_t 	junc_row_next_; //
	u_int8_t 	junc_col_next_; //

	//for route layer update info
	bool		update_info_;



	inline bool& update_info() {
		return update_info_;
	}
	inline double& hello_tstamp() {
		return hello_tstamp_;
	}
	inline int16_t& vehicle_id() {
		return vehicle_id_;
	}
	inline int16_t& wired_id() {
		return wired_id_;
	}
	inline float& vehicle_position_x() {
		return vehicle_position_x_;
	}
	inline float& vehicle_position_y() {
		return vehicle_position_y_;
	}
	inline float& vehicle_speed() {
		return vehicle_speed_;
	}
	inline u_int8_t& vehicle_direction() {
		return vehicle_direction_;
	}
	inline int16_t& bs_id() {
			return bs_id_;
		}
	inline u_int8_t& junc_row_prev() {
		return junc_row_prev_;
	}
	inline u_int8_t& junc_col_prev() {
		return junc_col_prev_;
	}
	inline u_int8_t& junc_row_next() {
		return junc_row_next_;
	}
	inline u_int8_t& junc_col_next() {
		return junc_col_next_;
	}

	static int offset_;
	inline static int& offset() {
		return offset_;
	}
	inline static hdr_veh_hello* access(const Packet* p) {
		return (hdr_veh_hello*) p->access(offset_);
	}
};



struct hdr_wired_infos{
	u_int32_t	info_seqno_;

	//to sink
	double		infos_tstamp_;	//msg Sent timestamp
	double		infos_tstamp_list[VEHICULAR_AMOUNT];	//msg Sent timestamp
	int16_t 	vehicle_id_list[VEHICULAR_AMOUNT]; //device ID----vehicle----save storage
	int16_t 	wired_id_list[VEHICULAR_AMOUNT]; //device ID----vehicle----save storage
	float 		vehicle_position_x_list[VEHICULAR_AMOUNT]; //
	float	 	vehicle_position_y_list[VEHICULAR_AMOUNT]; //
	float		vehicle_speed_list[VEHICULAR_AMOUNT]; //----save storage
	u_int8_t 	vehicle_direction_list[VEHICULAR_AMOUNT]; //

	//added by wired
	int16_t		bs_id_list[VEHICULAR_AMOUNT];
	u_int8_t 	junc_row_prev_list[VEHICULAR_AMOUNT]; //
	u_int8_t 	junc_col_prev_list[VEHICULAR_AMOUNT]; //
	u_int8_t 	junc_row_next_list[VEHICULAR_AMOUNT]; //
	u_int8_t 	junc_col_next_list[VEHICULAR_AMOUNT]; //

	inline u_int32_t& info_seqno() {
			return info_seqno_;
	}
	inline double& infos_tstamp() {
		return infos_tstamp_;
	}
	inline double* infos_tstamplist() {
		return infos_tstamp_list;
	}
	inline int16_t* vehicle_idlist() {
		return vehicle_id_list;
	}
	inline int16_t* wired_idlist() {
		return wired_id_list;
	}
	inline int16_t* bs_idlist() {
		return wired_id_list;
	}
	inline float* vehicle_position_xlist() {
		return vehicle_position_x_list;
	}
	inline float* vehicle_position_ylist() {
		return vehicle_position_y_list;
	}
	inline float* vehicle_speedlist() {
		return vehicle_speed_list;
	}
	inline u_int8_t* vehicle_directionlist() {
		return vehicle_direction_list;
	}
	inline u_int8_t* junc_row_prevlist() {
		return junc_row_prev_list;
	}
	inline u_int8_t* junc_col_prevlist() {
		return junc_col_prev_list;
	}
	inline u_int8_t* junc_row_nextlist() {
		return junc_row_next_list;
	}
	inline u_int8_t* junc_col_nextlist() {
		return junc_col_next_list;
	}


	static int offset_;
	inline static int& offset() {
		return offset_;
	}
	inline static hdr_wired_infos* access(const Packet* p) {
		return (hdr_wired_infos*) p->access(offset_);
	}
};



#endif /* AVANET_TAVR_TRAFFIC_PKT_H_ */
