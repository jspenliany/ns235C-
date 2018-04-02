/*
 * tavrApp.cc
 *
 *  Created on: Mar 15, 2018
 *      Author: js
 */
#include "tavr_traffic_pkt.h"
#include "tavr_Appagent.h"
#include <udp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <packet.h>
#include <address.h>
#include <ip.h>
#include <rtp.h>




static class TAVRAppAgentClass : public TclClass {
 public:
	TAVRAppAgentClass() : TclClass("Agent/UDP/TAVRAppAgent") {}
	TclObject* create(int, const char*const*) {
		return (new TAVRAppAgent());
	}
} class_tavr_appAgent;



TAVRAppAgent::TAVRAppAgent():UdpAgent(),running_(0),seqno_(0){

}

TAVRAppAgent::TAVRAppAgent(packet_t type):UdpAgent(type),running_(0),seqno_(0){

}

int
TAVRAppAgent::command(int argc, const char*const* argv){
	if (argc == 2) {

	} else if (argc == 3) {
//		Tcl& tcl = Tcl::instance();
		if (strcmp(argv[1], "node_id") == 0) {
			node_id = atoi(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "node_type") == 0) {
			type_of_node = atoi(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "comm_id") == 0) {
			comm_id = atoi(argv[2]);
			return (TCL_OK);
		}
	}
	return (Agent::command(argc, argv));
}



void
TAVRAppAgent::sendmsg(int nbytes, const char *flags){
//	app_msg = flags;
//	send_appdata();
//	idle();
}

void
TAVRAppAgent::send_msg(hdr_veh_hello* hvh){
	int i = hvh->vehicle_id();

//	for(int i=0; i< VEHICULAR_AMOUNT; i++){
		app_msg.vehicle_id_list[i] = hvh->vehicle_id();
		app_msg.vehicle_direction_list[i] = hvh->vehicle_direction();
		app_msg.vehicle_position_x_list[i] = hvh->vehicle_position_x();
		app_msg.vehicle_position_y_list[i] = hvh->vehicle_position_y();
		app_msg.vehicle_speed_list[i] = hvh->vehicle_speed();
		app_msg.wired_id_list[i] = hvh->wired_id();
		app_msg.bs_id_list[i] = hvh->bs_id();
		app_msg.junc_col_next_list[i] =hvh->junc_col_next();
		app_msg.junc_row_next_list[i] = hvh->junc_row_next();
		app_msg.junc_col_prev_list[i] = hvh->junc_col_prev();
		app_msg.junc_row_prev_list[i] = hvh->junc_row_prev();
		app_msg.infos_tstamp_list[i] = hvh->hello_tstamp();
//	}
	send_appdata();
	idle();
}

void
TAVRAppAgent::send_msg(hdr_wired_infos* hvh){
	if(type_of_node == COMM_NODE)fprintf(stdout,"---***TAVRAppAgent::send_msg start ");
	for(int i=0; i< VEHICULAR_AMOUNT; i++){
		app_msg.vehicle_id_list[i] = hvh->vehicle_id_list[i];
		app_msg.vehicle_direction_list[i] = hvh->vehicle_direction_list[i];
		app_msg.vehicle_position_x_list[i] = hvh->vehicle_position_x_list[i];
		app_msg.vehicle_position_y_list[i] = hvh->vehicle_position_y_list[i];
		app_msg.vehicle_speed_list[i] = hvh->vehicle_speed_list[i];
		app_msg.wired_id_list[i] = hvh->wired_id_list[i];
		app_msg.bs_id_list[i] = hvh->bs_id_list[i];
		app_msg.junc_col_next_list[i] =hvh->junc_col_next_list[i];
		app_msg.junc_row_next_list[i] = hvh->junc_row_next_list[i];
		app_msg.junc_col_prev_list[i] = hvh->junc_col_prev_list[i];
		app_msg.junc_row_prev_list[i] = hvh->junc_row_prev_list[i];
		app_msg.infos_tstamp_list[i] = hvh->infos_tstamp_list[i];
	}
	if(type_of_node == COMM_NODE)fprintf(stdout,"---***TAVRAppAgent::send_msg before send_data ");
	send_appdata();
	if(type_of_node == COMM_NODE)fprintf(stdout,"----***TAVRAppAgent::send_msg after type_of_node=%d\n",type_of_node);
	idle();
}

void
TAVRAppAgent::send_appdata(){
	fprintf(stdout,"----***AppAgent::send_appdata start");
	switch(type_of_node){
	case WIRED_NODE:
		wired_send();//from veh to commander
		break;
	case BASE_NODE:break;
	case MOBILE_NODE:
		mobile_send();//to commander
		break;
	case COMM_NODE:
		comm_send();//from commander to veh
		break;
	default:
		fprintf(stderr,"error AppAgent::send_appdata  type_of_node=%d",type_of_node);
		exit(1);
	}
	fprintf(stdout,"--send_appdata--finish type_of_node=%d",type_of_node);
}




void
TAVRAppAgent::comm_send(){//mac broadcast & IP broadcast
	fprintf(stdout," TAVRAppAgent::comm_send TAVRAppAgent comm_send will start......!!");
	if(!app_){
		fprintf(stderr," TAVRAppAgent::comm_send TAVRAppAgent comm_send ERROR no app attached!!");
		exit(1);
	}
	for(int wired_i=0; wired_i < node_id; wired_i++){
		Packet *p = allocpkt();
		struct hdr_cmn* ch = HDR_CMN(p);
		struct hdr_ip*	ih = HDR_IP(p);
		struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);

		packet_clean(p);

		ch->ptype() = PT_TAVRWIRED;
		ch->addr_type() = NS_AF_ILINK;
		ch->direction() = hdr_cmn::DOWN;
		ch->timestamp() = Scheduler::instance().clock();
		char cip[14] = {'\0'};//may be a bug
		sprintf(cip,"0.0.%d",node_id);

		fprintf(stdout," -comm_send- cip=%s -*- ",cip);

//	ch->prev_hop_ =Address::instance().str2addr(cip);

		ih->ttl_ = NETWORK_DIAMETER;

		double curTime = Scheduler::instance().clock();

		for(int i=0; i< VEHICULAR_AMOUNT; i++){
	//		if(curTime - app_->get_laest_update_tstamp_list()[i] - intervalRate*app_->get_send_interval() < 0.001){
			if(app_msg.vehicle_idlist()[i] > -1){
				reply->vehicle_id_list[i] = app_msg.vehicle_idlist()[i];
				reply->vehicle_direction_list[i] = app_msg.vehicle_directionlist()[i];
				reply->vehicle_position_x_list[i] = app_msg.vehicle_position_xlist()[i];
				reply->vehicle_position_y_list[i] = app_msg.vehicle_position_ylist()[i];
				reply->vehicle_speed_list[i] = app_msg.vehicle_speedlist()[i];
				reply->wired_id_list[i] = app_msg.wired_idlist()[i];
				reply->bs_id_list[i] = app_msg.bs_idlist()[i];
				reply->junc_col_next_list[i] = app_msg.junc_col_nextlist()[i];
				reply->junc_row_next_list[i] = app_msg.junc_row_nextlist()[i];
				reply->junc_col_prev_list[i] = app_msg.junc_col_prevlist()[i];
				reply->junc_row_prev_list[i] = app_msg.junc_row_prevlist()[i];
				reply->infos_tstamp_list[i] = app_msg.infos_tstamplist()[i];
			}else{
			}
		}
		reply->infos_tstamp() = curTime;

		reply->info_seqno() = ++seqno_;
		char nip[14] = {'\0'};//may be a bug
		sprintf(nip,"0.0.%d",wired_i);
		ch->next_hop() = (u_int32_t)Address::instance().str2addr(nip);
		char dip[14] = {'\0'};//may be a bug
		sprintf(dip,"1.0.%d",wired_i);
		ih->daddr() = (u_int32_t)Address::instance().str2addr(nip);
		target_->recv(p);
	}
}

void
TAVRAppAgent::wired_send(){//mac broadcast & IP unicast--------------------success
	if(!app_){
		fprintf(stderr," TAVRAppAgent::wired_send TAVRAppAgent wired_send ERROR no app attached!!");
		exit(1);
	}
	if(type_of_node == COMM_NODE){
		fprintf(stderr," TAVRAppAgent::wired_send TAVRAppAgent wired_send ERROR comm try to send msg from wired channel!!");
		exit(1);
	}
	Packet *p = allocpkt();
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_ip*	ih = HDR_IP(p);
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);

	packet_clean(p);



	char nip[14] = {'\0'};//may be a bug
	int	 nid = node_id;
	sprintf(nip,"1.0.%d",nid);
	ch->next_hop() = (u_int32_t)Address::instance().str2addr(nip);
	ch->ptype() = PT_TAVRWIRED;
	ch->addr_type() = NS_AF_ILINK;
	ch->timestamp() = Scheduler::instance().clock();
	char cip[14] = {'\0'};//may be a bug
	sprintf(cip,"0.0.%d",node_id);
	fprintf(stdout," -wired_send- cip=%s -*- ",cip);

	ch->prev_hop_ =(u_int32_t)Address::instance().str2addr(cip);

	ih->saddr() = (u_int32_t)Address::instance().str2addr(cip);

	char dip[14] = {'\0'};//may be a bug
	int	 did = comm_id + 2;
	sprintf(dip,"1.0.%d",did);
	fprintf(stdout," -wired_send- dip=%s -*- ",dip);
	ih->daddr() = (u_int32_t)Address::instance().str2addr(dip);
	ih->dport() = this->port();
	ih->ttl_ = 4;


	for(int i=0; i< VEHICULAR_AMOUNT; i++){
//		if(curTime - app_->get_laest_update_tstamp_list()[i] - intervalRate*app_->get_send_interval() < 0.001){
		if(app_msg.vehicle_id_list[i] > -1){
			reply->vehicle_id_list[i] = app_msg.vehicle_idlist()[i];
			reply->vehicle_direction_list[i] = app_msg.vehicle_directionlist()[i];
			reply->vehicle_position_x_list[i] = app_msg.vehicle_position_xlist()[i];
			reply->vehicle_position_y_list[i] = app_msg.vehicle_position_ylist()[i];
			reply->vehicle_speed_list[i] = app_msg.vehicle_speedlist()[i];
			reply->wired_id_list[i] = app_msg.wired_idlist()[i];
			reply->bs_id_list[i] = app_msg.bs_idlist()[i];
			reply->junc_col_next_list[i] = app_msg.junc_col_nextlist()[i];
			reply->junc_row_next_list[i] = app_msg.junc_row_nextlist()[i];
			reply->junc_col_prev_list[i] = app_msg.junc_col_prevlist()[i];
			reply->junc_row_prev_list[i] = app_msg.junc_row_prevlist()[i];
			reply->infos_tstamplist()[i] = app_msg.infos_tstamplist()[i];
		}else{//current vehicle NOT in this range
			reply->vehicle_id_list[i] = -100;
			reply->vehicle_direction_list[i] = -100;
			reply->vehicle_position_x_list[i] = -100;
			reply->vehicle_position_y_list[i] = -100;
			reply->vehicle_speed_list[i] = -100;
			reply->wired_id_list[i] = -100;
			reply->bs_id_list[i] = -100;
			reply->junc_col_next_list[i] =-100;
			reply->junc_row_next_list[i] = -100;
			reply->junc_col_prev_list[i] = -100;
			reply->junc_row_prev_list[i] = -100;
		}
	}
	reply->infos_tstamp() = Scheduler::instance().clock();
	reply->info_seqno() = ++seqno_;
	target_->recv(p);
}

void
TAVRAppAgent::mobile_send(){
	if(!app_){
		fprintf(stderr," TAVRAppAgent::mobile_send TAVRAppAgent comm_send ERROR no app attached!!");
		exit(1);
	}
	Packet *p = allocpkt();
	struct hdr_cmn* ch = HDR_CMN(p);
	ch->ptype() = PT_TAVRHELLO;
	struct hdr_ip*	ih = HDR_IP(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);

	packet_clean(p);


	unsigned int size = sizeof(hdr_veh_hello) * VEHICULAR_AMOUNT;
	ch->size() = (int)size;
	fprintf(stdout," -mobile_send- dip=%s -*-sizeof=%u %d",Address::instance().print_nodeaddr(ih->daddr()),size,(int)size);
	ch->timestamp() = Scheduler::instance().clock();

	hdr_rtp* rh = hdr_rtp::access(p);
	rh->flags() = 0;
	rh->seqno() = ++seqno_;

	ih->ttl_ = 6;//since the closest wired node is at least two hops from current vehicle
		hello->hello_seqno() = ++seqno_;
		hello->vehicle_id() = node_id;
		hello->vehicle_direction() = app_msg.vehicle_directionlist()[node_id];
		hello->vehicle_position_x() = app_msg.vehicle_position_xlist()[node_id];
		hello->vehicle_position_y() = app_msg.vehicle_position_ylist()[node_id];
		hello->vehicle_speed() = app_msg.vehicle_speedlist()[node_id];
		hello->wired_id() = app_msg.wired_idlist()[node_id];
		hello->bs_id() = app_msg.bs_idlist()[node_id];
		hello->junc_col_next() = app_msg.junc_col_nextlist()[node_id];
		hello->junc_row_next() = app_msg.junc_row_nextlist()[node_id];
		hello->junc_col_prev() = app_msg.junc_col_prevlist()[node_id];
		hello->junc_row_prev() = app_msg.junc_row_prevlist()[node_id];
		hello->hello_tstamp() = Scheduler::instance().clock();
	target_->recv(p);
}

void
TAVRAppAgent::recv(Packet* pkt, Handler*)
{
	char tmpStr[300] = {'\0'};
	hdr_cmn* cmnh = hdr_cmn::access(pkt);
	hdr_ip*  iph = hdr_ip::access(pkt);

	if (app_) {
		// If an application is attached, pass the data to the app

		if(cmnh->ptype() == PT_TAVRHELLO || cmnh->ptype() == PT_TAVRWIRED){
			if(cmnh->ptype() == PT_TAVRHELLO){
				switch(type_of_node){
				case WIRED_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  sip=%s, WIRED cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				case BASE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  sip=%s, BASE cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				case MOBILE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  sip=%s, MOBILE cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				case COMM_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  sip=%s, COMM cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				default:
					fprintf(stderr," AppAgent::recv HELLO reach application layer!! type ERROR");
					exit(1);
				}

				fprintf(stdout,tmpStr);
				recv_hello(type_of_node,pkt);

			}else if(cmnh->ptype() == PT_TAVRWIRED){
				switch(type_of_node){
				case WIRED_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  sip=%s, WIRED cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				case BASE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  sip=%s, BASE cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				case MOBILE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  sip=%s, MOBILE cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				case COMM_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  sip=%s, COMM cid=%d, dip=%s",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()));
					break;
				default:
					fprintf(stderr," AppAgent::recv WIRED reach application layer!! type ERROR");
					exit(1);
				}

				fprintf(stdout,tmpStr);
				recv_wired(type_of_node,pkt);
			}
		}else{
			fprintf(stderr," TAVRAppAgent::recv  ERROR unkonwn package reach application layer!!");
			exit(1);
		}
	}else{
		fprintf(stderr," TAVRAppAgent::recv TAVRAppAgent comm_send ERROR no app attached!!");
		exit(1);
	}
	Packet::free(pkt);
}


void
TAVRAppAgent::recv_hello(int nt, Packet* p){
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);
	switch(type_of_node){
	case WIRED_NODE:
		fprintf(stderr," AppAgent::recv HELLO reach application layer!! a common wired node recv");
		exit(1);
		break;
	case BASE_NODE:
		fprintf(stderr," AppAgent::recv HELLO reach application layer!! a base node recv");
		exit(1);
		break;
	case MOBILE_NODE:
		fprintf(stdout," . a vehicle recv . ");
		app_->recv_msg(MOBILE_NODE,hello);
		break;
	case COMM_NODE:
		fprintf(stdout," . The comm node recv . ");
		app_->recv_msg(COMM_NODE,hello);
		break;
	default:
		fprintf(stderr," AppAgent::recv HELLO reach application layer!! type ERROR");
		exit(1);
	}
}

void
TAVRAppAgent::recv_wired(int nt, Packet* p){
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);
	switch(type_of_node){
	case WIRED_NODE:
		fprintf(stdout," AppAgent. a common wired node recv . ");
		app_->recv_msg(WIRED_NODE,reply);
		break;
	case BASE_NODE:
		fprintf(stderr," AppAgent::recv_wired HELLO reach application layer!! a base node recv");
		exit(1);
	case MOBILE_NODE:
		fprintf(stdout," AppAgent. a common vehicle recv . ");
		app_->recv_msg(MOBILE_NODE,reply);
		break;
	case COMM_NODE:
		fprintf(stdout," AppAgent. The comm recv . ");
		break;
	default:
		fprintf(stderr," AppAgent::recv_wired HELLO reach application layer!! type ERROR");
		exit(1);
	}
}

void
TAVRAppAgent::packet_clean(Packet* p){
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);

	for(int i=0; i< VEHICULAR_AMOUNT; i++){
		reply->vehicle_id_list[i] = -100;
		reply->vehicle_direction_list[i] = -100;
		reply->vehicle_position_x_list[i] = -100;
		reply->vehicle_position_y_list[i] = -100;
		reply->vehicle_speed_list[i] = -100;
		reply->wired_id_list[i] = -100;
		reply->bs_id_list[i] = -100;
		reply->junc_col_next_list[i] =-100;
		reply->junc_row_next_list[i] = -100;
		reply->junc_col_prev_list[i] = -100;
		reply->junc_row_prev_list[i] = -100;
		reply->infos_tstamp_list[i] = -100;
	}

	hello->vehicle_id() = -100;
	hello->vehicle_direction() = -100;
	hello->vehicle_position_x() = -100;
	hello->vehicle_position_y() = -100;
	hello->vehicle_speed() = -100;
	hello->wired_id() = -100;
	hello->bs_id() = -100;
	hello->junc_col_next() = -100;
	hello->junc_row_next() = -100;
	hello->junc_col_prev() = -100;
	hello->junc_row_prev() = -100;
	hello->hello_tstamp() = -100;
	hello->update_info() = false;
}
/*
void
TAVRAppAgent::wired_send(){//mac broadcast & IP unicast--------------------success
	if(!app_){
		fprintf(stderr," TAVRAppAgent::wired_send TAVRAppAgent wired_send ERROR no app attached!!");
		exit(1);
	}
	if(type_of_node == COMM_NODE){
		fprintf(stderr," TAVRAppAgent::wired_send TAVRAppAgent wired_send ERROR comm try to send msg from wired channel!!");
		exit(1);
	}
	Packet *p = allocpkt();
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_ip*	ih = HDR_IP(p);
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);

	packet_clean(p);



	char dip[14] = {'\0'};//may be a bug
	int	 did = comm_id;
	sprintf(dip,"0.0.%d",did);
	ch->next_hop() = (u_int32_t)Address::instance().str2addr(dip);
	ch->ptype() = PT_TAVRWIRED;
	ch->addr_type() = NS_AF_ILINK;
	ch->timestamp() = Scheduler::instance().clock();
	char cip[14] = {'\0'};//may be a bug
	sprintf(cip,"0.0.%d",node_id);
	fprintf(stdout," -wired_send- cip=%s -*- ",cip);

	ch->prev_hop_ =(u_int32_t)Address::instance().str2addr(cip);

	ih->saddr() = (u_int32_t)Address::instance().str2addr(cip);
	fprintf(stdout," -wired_send- dip=%s -*- ",dip);
	ih->daddr() = (u_int32_t)Address::instance().str2addr(dip);
	ih->dport() = this->port();
	ih->ttl_ = WIRED_DIAMETER;


	for(int i=0; i< VEHICULAR_AMOUNT; i++){
//		if(curTime - app_->get_laest_update_tstamp_list()[i] - intervalRate*app_->get_send_interval() < 0.001){
		if(app_msg.vehicle_id_list[i] > -1){
			reply->vehicle_id_list[i] = app_msg.vehicle_idlist()[i];
			reply->vehicle_direction_list[i] = app_msg.vehicle_directionlist()[i];
			reply->vehicle_position_x_list[i] = app_msg.vehicle_position_xlist()[i];
			reply->vehicle_position_y_list[i] = app_msg.vehicle_position_ylist()[i];
			reply->vehicle_speed_list[i] = app_msg.vehicle_speedlist()[i];
			reply->wired_id_list[i] = app_msg.wired_idlist()[i];
			reply->bs_id_list[i] = app_msg.bs_idlist()[i];
			reply->junc_col_next_list[i] = app_msg.junc_col_nextlist()[i];
			reply->junc_row_next_list[i] = app_msg.junc_row_nextlist()[i];
			reply->junc_col_prev_list[i] = app_msg.junc_col_prevlist()[i];
			reply->junc_row_prev_list[i] = app_msg.junc_row_prevlist()[i];
			reply->infos_tstamplist()[i] = app_msg.infos_tstamplist()[i];
		}else{//current vehicle NOT in this range
			reply->vehicle_id_list[i] = -100;
			reply->vehicle_direction_list[i] = -100;
			reply->vehicle_position_x_list[i] = -100;
			reply->vehicle_position_y_list[i] = -100;
			reply->vehicle_speed_list[i] = -100;
			reply->wired_id_list[i] = -100;
			reply->bs_id_list[i] = -100;
			reply->junc_col_next_list[i] =-100;
			reply->junc_row_next_list[i] = -100;
			reply->junc_col_prev_list[i] = -100;
			reply->junc_row_prev_list[i] = -100;
		}
	}
	reply->infos_tstamp() = Scheduler::instance().clock();
	reply->info_seqno() = ++seqno_;
	target_->recv(p);
}*/
