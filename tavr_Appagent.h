/*
 * tavrApp.h
 *
 *  Created on: Mar 15, 2018
 *      Author: js
 */

#ifndef TAVR_APPAGENT_H_
#define TAVR_APPAGENT_H_

#include <udp.h>
#include "tavr_traffic_pkt.h"
#include "tavr_app.h"
#include "tavr_file.h"


class TAVRAppAgent:public UdpAgent {
public:
	TAVRAppAgent();
	TAVRAppAgent(packet_t);
	~TAVRAppAgent();
//	TAVRAppAgent(int type_code, int index);
/*	virtual int supporttavrApp() {return 1;}
	virtual void enabletavrApp() {support_tavr_app = 1;}*/
	virtual void send(){};
	virtual void sendmsg(int nbytes, const char *flags=0);
	void recv(Packet*, Handler*);
	virtual void send_msg(hdr_wired_infos*);
	virtual void send_msg(hdr_veh_hello*);
	int command(int argc, const char*const* argv);
protected:

private:
	void send_appdata();
	void wired_send();
	void mobile_send();
	void comm_send();

	void recv_hello(int nodeType, Packet*);
	void recv_wired(int nodeType, Packet*);

	void packet_clean(Packet*);
private:

	hdr_wired_infos app_msg;
	//installation type index
		int		type_of_node;
	//vehicle oriented
		int		node_id;
		int		comm_id;
	int running_;
//installation type index
//vehicle oriented
	int		seqno_;
	static TAVRfile file_op;
	static bool	file_exist;
	bool	debug_file;
	double	file_wirte_Time;

//for app-------------------
	double 	vehicle_info_updateTime_LIST_[VEHICULAR_AMOUNT]; //for both comm node and common wired nodes recording the time that packet recv
	int16_t  vehicle_fresh_LIST_[VEHICULAR_AMOUNT]; //for common wired node recording the fresh degree of info since the last recv---------range 0~127.-128~-1
	int	support_tavr_app;//------------

	double  wired_send_interval;
	double  hello_send_interval;
	int		current_veh_num;

};


#endif /* AVANET_TAVR_APPAGENT_H_ */
