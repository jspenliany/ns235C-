/*
 * tavr_pkt.h
 *
 *  Created on: Jan 20, 2018
 *      Author: js
 */

#ifndef TAVR_PKT_H_
#define TAVR_PKT_H_

#include <packet.h>


#define TAVRTYPE_HELLO  	0x01
#define TAVRTYPE_WIRED   	0x02
#define TAVRTYPE_DATA   	0x04
#define TAVRTYPE_TAVR   	0x08




#define		BASE_STATION_AMOUNT	20						//how many base sink nodes in the network

#define		VEHICULAR_AMOUNT	100					//how many vehicular in the network
#define		HELLO_INTERVAL		2						//interval for send out Hello msg( second)
#define		TAVR_MAX_HOPS		50						//hops that msg will tx


#define HDR_VEH_HELLO(p) 	((struct hdr_veh_hello*)hdr_tavr::access(p))
#define HDR_WIRED_INFO(p) 	((struct hdr_wired_info*)hdr_tavr::access(p))
#define HDR_TAVR_DATA(p) 	((struct hdr_tavr_data*)hdr_tavr::access(p))
#define HDR_TAVR(p) 		((struct hdr_tavr*)hdr_tavr::access(p))



struct hdr_tavr {
        u_int8_t        tavr_type_;
	/*
        u_int8_t        ah_reserved[2];
        u_int8_t        ah_hopcount;
	*/
		// Header access methods
    inline u_int8_t& tavr_type(){return tavr_type_;}
	static int offset_; // required by PacketHeaderManager
	inline static int& offset() { return offset_; }
	inline static hdr_tavr* access(const Packet* p) {
		return (hdr_tavr*) p->access(offset_);
	}
};


struct hdr_veh_hello{
	u_int8_t		rvh_type_;
	//to sink
	double		hello_tstamp_;	//msg Sent timestamp
	nsaddr_t 	vehicle_ip_; //device ID----vehicle
	nsaddr_t 	wired_ip_; //device ID----vehicle
	double 		vehicle_position_x_; //
	double	 	vehicle_position_y_; //
	double	 	vehicle_speed_; //
	u_int8_t 	vehicle_direction_; //

	//following will be added by base station
	bool		junction_;
	nsaddr_t	bs_ip_;




	inline double& hello_tstamp() {
		return hello_tstamp_;
	}
	inline nsaddr_t& vehicle_ip() {
		return vehicle_ip_;
	}
	inline nsaddr_t& wired_ip() {
		return wired_ip_;
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
	inline int size() {
		return (1 + 4 + 4 + 4 + 2 + 2 + 4 + 1 + 1 + 2);
	}

/*	static int offset_;
	inline static int& offset() {
		return offset_;
	}
	inline static hdr_veh_hello* access(const Packet* p) {
		return (hdr_veh_hello*) p->access(offset_);
	}*/
};



struct hdr_wired_info{
	u_int8_t		rwi_type_;
	double 		wiredinfo_tstamp_;	//msg Sent timestamp
	nsaddr_t 	vehicle_ip_LIST_[VEHICULAR_AMOUNT]; //device ID----vehicle
	nsaddr_t 	all_ip_; //device ID----vehicle
	double	 	vehicle_position_x_LIST_[VEHICULAR_AMOUNT]; //
	double	 	vehicle_position_y_LIST_[VEHICULAR_AMOUNT]; //
	double 		vehicle_speed_LIST_[VEHICULAR_AMOUNT]; //
	u_int8_t 	vehicle_direction_LIST_[VEHICULAR_AMOUNT]; //
	bool		junction_LIST_[VEHICULAR_AMOUNT];
	nsaddr_t	bs_ip_LIST_[VEHICULAR_AMOUNT];




	inline double& wiredinfo_tstamp() {
		return wiredinfo_tstamp_;
	}
	inline nsaddr_t* vehicle_ip_LIST() {
		return vehicle_ip_LIST_;
	}
	inline nsaddr_t& all_ip() {
			return all_ip_;
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
	inline int size() {
		return (4 + 4 + 4 + 4 + 1 + 1 + 4) * VEHICULAR_AMOUNT + 4 + 4 + 1;
	}

/*	static int offset_;
	inline static int& offset() {
		return offset_;
	}
	inline static hdr_wired_info* access(const Packet* p) {
		return (hdr_wired_info*) p->access(offset_);
	}*/
};





struct cjunction_info {
	u_int16_t	junc_id_;
	double		junc_x_;
	double		junc_y_;
	double		junc_z_;
};

struct hdr_tavr_data{
	u_int8_t		rtd_type_;
	u_int32_t		pkt_create_id_; 	//msg ID
	u_int32_t		pkt_seq_id_;    	//sequence ID
	u_int8_t		pkt_ttl_;			//msg Lifetime;
	double			pkt_sent_tstamp_;	//msg Sent timestamp
	double			pkt_gene_tstamp_;	//msg Generated timestamp
	nsaddr_t		pkt_src_;			//msg Source address
	nsaddr_t		pkt_dst_;			//msg Destination address
	nsaddr_t		pkt_next_vehicle_;	//msg Destination address
	cjunction_info	pkt_next_junc_;		//relay destinaiton
	u_int8_t		pkt_tx_count_list_[TAVR_MAX_HOPS];  //msg transmission couter on MAC-layer
	nsaddr_t		pkt_relay_list_[TAVR_MAX_HOPS]; //relay vehicular addrsses in one packet transmission
	u_int8_t		pkt_junction_list_[TAVR_MAX_HOPS];//junction traversed info
	double			pkt_delay_perhop_list_[TAVR_MAX_HOPS];//delay info of each hop

	inline 	u_int32_t&	pkt_create_id()		{return pkt_create_id_;}
	inline 	u_int32_t&	pkt_seq_id()		{return pkt_seq_id_;}
	inline 	double&		pkt_sent_tstamp()	{return pkt_sent_tstamp_;}
	inline 	double&		pkt_gene_tstamp()	{return pkt_gene_tstamp_;}
	inline 	nsaddr_t&	pkt_src()			{return pkt_src_;}
	inline 	nsaddr_t&	pkt_dst()			{return pkt_dst_;}
	inline 	nsaddr_t&	pkt_next_vehicle()		{return pkt_next_vehicle_;}
	inline 	cjunction_info&	pkt_next_junc()		{return pkt_next_junc_;}
	inline 	u_int8_t*	pkt_tx_count_list()		{return pkt_tx_count_list_;}
	inline 	nsaddr_t*	pkt_relay_list()		{return pkt_relay_list_;}
	inline 	u_int8_t*	pkt_junction_list()		{return pkt_junction_list_;}
	inline 	double*		pkt_delay_perhop_list()	{return pkt_delay_perhop_list_;}

	inline int size() {
			return (1 + 4 + 1 + 4) * TAVR_MAX_HOPS + 4 + 4 + 1 + 4 * 5 + 15;
	}

/*	static int 		offset_;
	inline static int&	offset()	{return offset_;}
	inline static hdr_tavr*	access(const Packet* p)	{
		return (hdr_tavr*) p->access(offset_);
	}*/
};

union hdr_all_tavr {
	hdr_tavr			tavr;
	hdr_veh_hello    	tavrhello;
	hdr_wired_info  	wiredinfo;
	hdr_tavr_data     	tavrdata;
};

#endif /* TAVR_PKT_H_ */
