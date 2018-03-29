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



class TAVRAppAgent:public UdpAgent {
public:
	TAVRAppAgent();
	TAVRAppAgent(packet_t);
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

	void recv_tavr_pkt(Packet*);
	void wired_recv(Packet*);
	void mobile_recv(Packet*);
	void comm_recv(Packet*);

	void wired_recv_mobile(Packet*);
	void wired_recv_base(Packet*);
	void wired_recv_wired(Packet*);

	int get_domain(nsaddr_t);

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


//for app-------------------
	int	support_tavr_app;//------------

};


#endif /* AVANET_TAVR_APPAGENT_H_ */
