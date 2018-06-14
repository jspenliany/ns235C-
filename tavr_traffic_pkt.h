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

//#define VEH_HELLO_VALID_TIMES	2.2	//the vehicle have NOT report its info to wired for the laest xxx periods of timer
//#define INFO_UP_INTERVAL_RATE	1.2 //the rate between valid info_up_interval and the info_up_timer_interval


#define WIRED_DIAMETER		7
#define	NETWORK_DIAMETER	50


#define		BASE_STATION_AMOUNT	20						//how many base sink nodes in the network

#define		VEHICULAR_AMOUNT	600					//how many vehicular in the network
#define 	TAVR_BS_INTERVAL	10
#define		TAVR_MAX_HOPS		50						//hops that msg will tx
#define		INFO_UPDATE_DELAY	10000					// ms



#define		APP_FROM			0x01
#define     TXL_FROM			0x02
#define		RTR_FROM			0x03
#define     HELLO_FROM			0x01
#define		WIRED_FROM			0x01

#define		DUP						16
#define		DLEFT					20
#define		DDOWN					24
#define		DRIGHT					28

#define     MOVE_UP				512				//00
#define		MOVE_LEFT			1024			//01
#define     MOVE_DOWN			1536			//10
#define		MOVE_RIGHT			2048			//11

#define		MOVE_NULL			4096

#define TAVR_APP_ALL_TX_JITTER		(Random::uniform()*0.2)

#define TAVR_APP_COMM_TX_JITTER		(Random::uniform()*0.5)
#define TAVR_APP_WIRED_TX_JITTER	(Random::uniform()*0.3)
#define TAVR_APP_MOBI_TX_JITTER		(Random::uniform()*0.3)

#define TAVR_RTR_BASE_JITTER	(Random::uniform()*0.1)
#define TAVR_RTR_WIRED_RET_JITTER	(Random::uniform()*0.2)
#define TAVR_RTR_HELLO_RET_JITTER	(Random::uniform()*0.2)

#define		NET_ROW_MAX		20
#define		NET_COL_MAX		20

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

struct hdr_app_tavr{
	u_int8_t	seqno_;
	static int offset_;
	inline static int& offset() {
		return offset_;
	}
	inline static hdr_app_tavr* access(const Packet* p) {
		return (hdr_app_tavr*) p->access(offset_);
	}
};

/**
 * from vehicular to wired
 * */
struct hdr_veh_hello{
	int8_t		send_seqno_; //----save storage
	int16_t 	vehicle_id_; //device ID----vehicle----save storage
	int16_t 	vehicle_position_x_; //
	int16_t		vehicle_position_y_; //
	int16_t		vehicle_speed_; //----save storage


	inline int16_t& vehicle_id() {
		return vehicle_id_;
	}
	inline int16_t& vehicle_position_x() {
		return vehicle_position_x_;
	}
	inline int16_t& vehicle_position_y() {
		return vehicle_position_y_;
	}
	inline int16_t& vehicle_speed() {
		return vehicle_speed_;
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
	int16_t 	vehicle_id_list[VEHICULAR_AMOUNT]; //device ID----vehicle----save storage
	int16_t 	vehicle_position_x_list[VEHICULAR_AMOUNT]; //
	int16_t		vehicle_position_y_list[VEHICULAR_AMOUNT]; //
	int16_t		vehicle_speed_list[VEHICULAR_AMOUNT]; //----save storage
	int16_t		recv_seqno_list[VEHICULAR_AMOUNT]; //----save storage

	inline int16_t* vehicle_idlist() {
		return vehicle_id_list;
	}
	inline int16_t* vehicle_position_xlist() {
		return vehicle_position_x_list;
	}
	inline int16_t* vehicle_position_ylist() {
		return vehicle_position_y_list;
	}
	inline int16_t* vehicle_speedlist() {
		return vehicle_speed_list;
	}
	inline int16_t* recv_seqnolist() {
		return recv_seqno_list;
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
