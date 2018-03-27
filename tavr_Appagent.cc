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
#include <packet.h>
#include <address.h>
#include <ip.h>



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
TAVRAppAgent::send_appdata(){
	fprintf(stdout,"\n----***TAVRAppAgent::send_appdata start type_of_node=%d",type_of_node);
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
		fprintf(stderr,"error TAVRAppAgent::send  type_of_node=%d",type_of_node);
		exit(1);
	}
	fprintf(stdout,"----***TAVRAppAgent::send_appdata finish type_of_node=%d",type_of_node);
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
	if(type_of_node == COMM_NODE)fprintf(stdout,"----***TAVRAppAgent::send_msg start ");
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
	if(type_of_node == COMM_NODE)fprintf(stdout,"----***TAVRAppAgent::send_msg before send_data ");
	send_appdata();
	if(type_of_node == COMM_NODE)fprintf(stdout,"----***TAVRAppAgent::send_msg after type_of_node=%d\n",type_of_node);
	idle();
}

void
TAVRAppAgent::comm_send(){//mac broadcast & IP broadcast
	fprintf(stdout," TAVRAppAgent::comm_send TAVRAppAgent comm_send will start......!!");
	if(!app_){
		fprintf(stderr," TAVRAppAgent::comm_send TAVRAppAgent comm_send ERROR no app attached!!");
		exit(1);
	}
	Packet *p = allocpkt();
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_ip*	ih = HDR_IP(p);
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);

	packet_clean(p);

	ch->next_hop() = IP_BROADCAST;
	ch->ptype() = PT_TAVRWIRED;
	ch->addr_type() = NS_AF_ILINK;
	ch->iface() = -2;
	ch->direction() = hdr_cmn::DOWN;
	ch->timestamp() = Scheduler::instance().clock();
	char cip[14] = {'\0'};//may be a bug
	sprintf(cip,"0.0.%d",node_id);

	fprintf(stdout," -comm_send- cip=%s -*- ",cip);

	ch->prev_hop_ =Address::instance().str2addr(cip);

	ih->saddr() = Address::instance().str2addr(cip);
	ih->daddr() = Address::instance().str2addr("0.0.2");
//	ih->sport() = RT_PORT;
	ih->dport() = this->port();
	ih->ttl_ = NETWORK_DIAMETER;

	double curTime = Scheduler::instance().clock();
//	double intervalRate = INFO_UP_INTERVAL_RATE;

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
	seqno_++;
	reply->info_seqno() = seqno_;
	fprintf(stdout," TAVRAppAgent::comm_send TAVRAppAgent comm_send will end before target_......!!");
	target_->recv(p);
	fprintf(stdout," TAVRAppAgent::comm_send TAVRAppAgent comm_send will end......!!");
}

void
TAVRAppAgent::wired_send(){//mac broadcast & IP unicast
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
//	ch->iface() = -2;
//	ch->direction() = hdr_cmn::DOWN;
	ch->timestamp() = Scheduler::instance().clock();
	char cip[14] = {'\0'};//may be a bug
	sprintf(cip,"0.0.%d",node_id);

	fprintf(stdout," -wired_send- cip=%s -*- ",cip);

	ch->prev_hop_ =(u_int32_t)Address::instance().str2addr(cip);

	ih->saddr() = (u_int32_t)Address::instance().str2addr(cip);


	fprintf(stdout," -wired_send- dip=%s -*- ",dip);

	ih->daddr() = (u_int32_t)Address::instance().str2addr(dip);
//	ih->sport() = RT_PORT;
	ih->dport() = this->port();
	ih->ttl_ = WIRED_DIAMETER;

//	double curTime = Scheduler::instance().clock();
//	double intervalRate = INFO_UP_INTERVAL_RATE;

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
	seqno_++;
	reply->info_seqno() = seqno_;
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

	char dip[14] = {'\0'};//may be a bug
	int	 did = comm_id;
	sprintf(dip,"0.0.%d",did);

	ch->next_hop() = (u_int32_t)Address::instance().str2addr(dip);
	ch->addr_type() = NS_AF_ILINK;
	ch->iface() = -2;
	ch->direction() = hdr_cmn::DOWN;
	ch->timestamp() = Scheduler::instance().clock();
	char cip[14] = {'\0'};//may be a bug

	sprintf(cip,"2.0.%d",node_id);

	fprintf(stdout," -mobile_send- cip=%s -*- ",cip);

	ch->prev_hop_ =Address::instance().str2addr(cip);

	ih->saddr() = Address::instance().str2addr(cip);


	fprintf(stdout," -mobile_send- dip=%s -*- ",dip);

	ih->daddr() = Address::instance().str2addr(dip);
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl_ = 2;//since the closest wired node is at least two hops from current vehicle
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
	if (app_ ) {
		// If an application is attached, pass the data to the app
		hdr_cmn* h = hdr_cmn::access(pkt);
		if(h->ptype() == PT_TAVRHELLO || h->ptype() == PT_TAVRWIRED)
			recv_tavr_pkt(pkt);
		else{
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
TAVRAppAgent::recv_tavr_pkt(Packet* p){
	if(!app_){
		fprintf(stderr," TAVRAppAgent::recv_tavr_pkt TAVRAppAgent comm_send ERROR no app attached!!");
		exit(1);
	}
//	int type_of_node = app_->get_type_node();
	if(running_ == 1){
		switch(type_of_node){
		case WIRED_NODE:
			wired_recv(p);//from veh to commander
			break;
		case BASE_NODE:break;
		case MOBILE_NODE:
			mobile_recv(p);//to commander
			break;
		case COMM_NODE:
			comm_recv(p);//from commander to veh
			break;
		default:
			fprintf(stderr,"\nrecv_tavr_pkt() error type_of_node=%d",type_of_node);
			exit(1);
		}
//		reply_timer.resched(reply_interval_);
	}
}

void
TAVRAppAgent::wired_recv(Packet* p){
	struct hdr_ip*	ih = HDR_IP(p);
	switch(get_domain(ih->saddr())){
	case WIRED_NODE://from comm to wired & from wired to comm
		wired_recv_wired(p);
		break;
	case BASE_NODE://from base to comm
		wired_recv_base(p);
		break;
	case MOBILE_NODE://from mobile to base
		wired_recv_mobile(p);
		break;
	default:
		fprintf(stderr,"\nrecv_tavr_pkt() error type_of_node=%d",type_of_node);
		exit(1);
	}
}
/**
 *
 * here, a potential bug, whether the daddr() of the ip header changes.
 * */
void
TAVRAppAgent::wired_recv_wired(Packet* p){
//	struct hdr_ip*	ih = HDR_IP(p);
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);

//	double curTime = Scheduler::instance().clock();
//	double intervalRate = INFO_UP_INTERVAL_RATE;

	app_->recv_msg(TAVRTYPE_WIRED,reply);
/*
	if(Address::instance().get_lastaddr(ih->saddr()) == COMM_ID){//direct from comm, indirect from comm
		for(int i=0; i< VEHICULAR_AMOUNT; i++){
			if(reply->vehicle_id_list[i] > -1){//may be update with a early info of current vehicle since the tstamp....
				app_->vehicle_id_list[i] = reply->vehicle_id_list[i];
				app_->vehicle_direction_list[i] = reply->vehicle_direction_list[i];
				app_->vehicle_position_x_list[i] = reply->vehicle_position_x_list[i];
				app_->vehicle_position_y_list[i] = reply->vehicle_position_y_list[i];
				app_->vehicle_speed_list[i] = reply->vehicle_speed_list[i];
				app_->wired_id_list[i] = reply->wired_id_list[i];
				app_->bs_id_list[i] = reply->bs_id_list[i];
				app_->junc_col_next_list[i] = reply->junc_col_next_list[i];
				app_->junc_row_next_list[i] = reply->junc_row_next_list[i];
				app_->junc_col_prev_list[i] = reply->junc_col_prev_list[i];
				app_->junc_row_prev_list[i] = reply->junc_row_prev_list[i];
				app_->laest_update_tstamp_list[i] = reply->infos_tstamp_[i];
			}else{//current vehicle NOT in this range
				char ipStri[50] = {'\0'};
				sprintf(ipStri,"2.0.%d",i);
				fprintf(stdout,"\n --==---===---===---===vehicle of IP=%s is lost in the scenario",ipStri);
				app_->vehicle_id_list[i] = -100;
				app_->vehicle_direction_list[i] = -100;
				app_->vehicle_position_x_list[i] = -100;
				app_->vehicle_position_y_list[i] = -100;
				app_->vehicle_speed_list[i] = -100;
				app_->wired_id_list[i] = -100;
				app_->bs_id_list[i] = -100;
				app_->junc_col_next_list[i] =-100;
				app_->junc_row_next_list[i] = -100;
				app_->junc_col_prev_list[i] = -100;
				app_->junc_row_prev_list[i] = -100;
				app_->laest_update_tstamp_list[i] = -100;
			}
		}
	}else if(get_domain(ih->saddr()) == WIRED_NODE){//from wired to comm
		for(int i=0; i< VEHICULAR_AMOUNT; i++){
			if(reply->vehicle_id_list[i] > -1 && (app_->laest_update_tstamp_list[i] < reply->infos_tstamp_[i])){
				app_->vehicle_id_list[i] = reply->vehicle_id_list[i];
				app_->vehicle_direction_list[i] = reply->vehicle_direction_list[i];
				app_->vehicle_position_x_list[i] = reply->vehicle_position_x_list[i];
				app_->vehicle_position_y_list[i] = reply->vehicle_position_y_list[i];
				app_->vehicle_speed_list[i] = reply->vehicle_speed_list[i];
				app_->wired_id_list[i] = reply->wired_id_list[i];
				app_->bs_id_list[i] = reply->bs_id_list[i];
				app_->junc_col_next_list[i] = reply->junc_col_next_list[i];
				app_->junc_row_next_list[i] = reply->junc_row_next_list[i];
				app_->junc_col_prev_list[i] = reply->junc_col_prev_list[i];
				app_->junc_row_prev_list[i] = reply->junc_row_prev_list[i];
				app_->laest_update_tstamp_list[i] = reply->infos_tstamp_[i];
			}
		}
		//routing layer deals with the retransmit of such packets
	}
	//I have no idea with how to deal such packets
//	Packet::free(p);
	*/
}

void
TAVRAppAgent::wired_recv_base(Packet* p){
	struct hdr_ip*	ih = HDR_IP(p);
//	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);

//	double curTime = Scheduler::instance().clock();
//	double intervalRate = INFO_UP_INTERVAL_RATE;

	if(get_domain(ih->saddr()) == WIRED_NODE){//from base
		char ipStr[50] = {'\0'};
		sprintf(ipStr,"0.0.%d",node_id);
		fprintf(stdout,"\n --==---===---wired_recv_base===---===IP=%s send a ERROR msg to %s",
				Address::instance().print_nodeaddr(ih->saddr()),
				ipStr);
	}else if(get_domain(ih->saddr()) == MOBILE_NODE && (hello->vehicle_id() > -1)){

		app_->recv_msg(TAVRTYPE_HELLO,hello);
/*

		int i = Address::instance().get_lastaddr(ih->saddr());

		if(app_->laest_update_tstamp_list[i] < hello->hello_tstamp()){
			app_->vehicle_id_list[i] = hello->vehicle_id();
			app_->vehicle_direction_list[i] = hello->vehicle_direction();
			app_->vehicle_position_x_list[i] = hello->vehicle_position_x();
			app_->vehicle_position_y_list[i] = hello->vehicle_position_y();
			app_->vehicle_speed_list[i] = hello->vehicle_speed();
			app_->wired_id_list[i] = hello->wired_id();//this will update at routing layer
			app_->bs_id_list[i] = hello->bs_id();
			app_->junc_col_next_list[i] = hello->junc_col_next();
			app_->junc_row_next_list[i] = hello->junc_row_next();
			app_->junc_col_prev_list[i] = hello->junc_col_prev();
			app_->junc_row_prev_list[i] = hello->junc_row_prev();
			app_->laest_update_tstamp_list[i] = hello->hello_tstamp();
		}
*/

	}else{
		fprintf(stderr," error exists in method wired_recv_base()");
		exit(1);
	}
	//I have no idea with how to deal such packets
//	Packet::free(p);
}

void
TAVRAppAgent::wired_recv_mobile(Packet* p){
//	struct hdr_ip*	ih = HDR_IP(p);
//	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);
	//I have no idea with how to deal such packets
//	int i = Address::instance().get_lastaddr(ih->saddr());

	app_->recv_msg(TAVRTYPE_HELLO,hello);

/*	if(app_->laest_update_tstamp_list[i] < hello->hello_tstamp()){
		app_->vehicle_id_list[i] = hello->vehicle_id();
		app_->vehicle_direction_list[i] = hello->vehicle_direction();
		app_->vehicle_position_x_list[i] = hello->vehicle_position_x();
		app_->vehicle_position_y_list[i] = hello->vehicle_position_y();
		app_->vehicle_speed_list[i] = hello->vehicle_speed();
		app_->wired_id_list[i] = hello->wired_id();//this will update at routing layer
		app_->bs_id_list[i] = hello->bs_id();
		app_->junc_col_next_list[i] = hello->junc_col_next();
		app_->junc_row_next_list[i] = hello->junc_row_next();
		app_->junc_col_prev_list[i] = hello->junc_col_prev();
		app_->junc_row_prev_list[i] = hello->junc_row_prev();
		app_->laest_update_tstamp_list[i] = hello->hello_tstamp();
	}*/
//	Packet::free(p);
}

void
TAVRAppAgent::mobile_recv(Packet* p){
	struct hdr_cmn* ch = HDR_CMN(p);
//	struct hdr_ip*	ih = HDR_IP(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);


	if(ch->ptype() == PT_TAVRHELLO){//from neighbor

		app_->recv_msg(TAVRTYPE_HELLO,hello);
/*
		int i = Address::instance().get_lastaddr(ih->saddr());
		app_->vehicle_id_list[i] = hello->vehicle_id();
		app_->vehicle_direction_list[i] = hello->vehicle_direction();
		app_->vehicle_position_x_list[i] = hello->vehicle_position_x();
		app_->vehicle_position_y_list[i] = hello->vehicle_position_y();
		app_->vehicle_speed_list[i] = hello->vehicle_speed();
		app_->wired_id_list[i] = hello->wired_id();//this will update at routing layer
		app_->bs_id_list[i] = hello->bs_id();
		app_->junc_col_next_list[i] = hello->junc_col_next();
		app_->junc_row_next_list[i] = hello->junc_row_next();
		app_->junc_col_prev_list[i] = hello->junc_col_prev();
		app_->junc_row_prev_list[i] = hello->junc_row_prev();
		app_->laest_update_tstamp_list[i] = hello->hello_tstamp();*/
	}else if(ch->ptype() == PT_TAVRWIRED){//from commander

		app_->recv_msg(TAVRTYPE_WIRED,reply);
/*
		for(int i=0; i < VEHICULAR_AMOUNT; i++){
			if((reply->vehicle_id_list[i] > -1) && (reply->infos_tstamp_[i] - app_->laest_update_tstamp_list[i] > 0.0)){//the most fresh data will be stored
				app_->vehicle_id_list[i] = reply->vehicle_id_list[i];
				app_->vehicle_direction_list[i] = reply->vehicle_direction_list[i];
				app_->vehicle_position_x_list[i] = reply->vehicle_position_x_list[i];
				app_->vehicle_position_y_list[i] = reply->vehicle_position_y_list[i];
				app_->vehicle_speed_list[i] = reply->vehicle_speed_list[i];
				app_->wired_id_list[i] = reply->wired_id_list[i];
				app_->bs_id_list[i] = reply->bs_id_list[i];
				app_->junc_col_next_list[i] = reply->junc_col_next_list[i];
				app_->junc_row_next_list[i] = reply->junc_row_next_list[i];
				app_->junc_col_prev_list[i] = reply->junc_col_prev_list[i];
				app_->junc_row_prev_list[i] = reply->junc_row_prev_list[i];
				app_->laest_update_tstamp_list[i] = reply->infos_tstamp_[i];
			}
		}*/
	}else{

	}
}



void
TAVRAppAgent::comm_recv(Packet* p){
	struct hdr_cmn* ch = HDR_CMN(p);
	struct hdr_ip*	ih = HDR_IP(p);
//	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);
	int cid = comm_id;
	char ipStr[50] = {'\0'};
	sprintf(ipStr,"0.0.%d",cid);

	fprintf(stdout," -comm_recv- ipStr=%s -*- ",ipStr);

	if(ih->saddr() == Address::instance().str2addr(ipStr)){

	}else{
		if(ch->ptype() == PT_TAVRWIRED){//from commander

			app_->recv_msg(TAVRTYPE_WIRED,reply);

/*			for(int i=0; i < VEHICULAR_AMOUNT; i++){
				if((reply->vehicle_id_list[i] > -1) && (reply->infos_tstamp_[i] - app_->laest_update_tstamp_list[i] > 0.0)){//the most fresh data will be stored
					app_->vehicle_id_list[i] = reply->vehicle_id_list[i];
					app_->vehicle_direction_list[i] = reply->vehicle_direction_list[i];
					app_->vehicle_position_x_list[i] = reply->vehicle_position_x_list[i];
					app_->vehicle_position_y_list[i] = reply->vehicle_position_y_list[i];
					app_->vehicle_speed_list[i] = reply->vehicle_speed_list[i];
					app_->wired_id_list[i] = reply->wired_id_list[i];
					app_->bs_id_list[i] = reply->bs_id_list[i];
					app_->junc_col_next_list[i] = reply->junc_col_next_list[i];
					app_->junc_row_next_list[i] = reply->junc_row_next_list[i];
					app_->junc_col_prev_list[i] = reply->junc_col_prev_list[i];
					app_->junc_row_prev_list[i] = reply->junc_row_prev_list[i];
					app_->laest_update_tstamp_list[i] = reply->infos_tstamp_[i];
				}
			}*/
		}

	}
}







int
TAVRAppAgent::get_domain(nsaddr_t cip){
	int* pNodeShift = Address::instance().NodeShift_;
	int* pNodeMask = Address::instance().NodeMask_;
	int  IPdomain = 0;

 	IPdomain = cip >> pNodeShift[1];
 	IPdomain = IPdomain & pNodeMask[1];

	delete [] pNodeShift;
	delete [] pNodeMask;
 	return IPdomain;
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
