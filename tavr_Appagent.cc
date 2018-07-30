/*
 * tavrApp.cc
 *
 *  Created on: Mar 15, 2018
 *      Author: js
 */
#include "tavr_traffic_pkt.h"
#include "tavr_Appagent.h"
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

TAVRfile TAVRAppAgent::file_op;
bool TAVRAppAgent::file_exist = false;

TAVRAppAgent::TAVRAppAgent():UdpAgent(PT_UDP),running_(0),seqno_(0){
	for(int i=0; i < VEHICULAR_AMOUNT; i++){
		vehicle_info_updateTime_LIST_[i] = 0.0001;
		vehicle_fresh_LIST_[i] = 0;
	}
	if(!TAVRAppAgent::file_exist){
		TAVRAppAgent::file_op.init_file("xxxx_tavr",102011);
		TAVRAppAgent::file_op.txl_write("------------this is transport layer CONTROL file................\n");
		TAVRAppAgent::file_op.helmsg_write("------------this is transport layer HELLO file................\n");
		TAVRAppAgent::file_op.wirmsg_write("------------this is transport layer WIRED file................\n");
		TAVRAppAgent::file_exist = true;
	}

	debug_file = false;
}

TAVRAppAgent::TAVRAppAgent(packet_t type):UdpAgent(type),running_(0),seqno_(0){

}

TAVRAppAgent::~TAVRAppAgent(){
	TAVRAppAgent::file_op.close();
}

int
TAVRAppAgent::command(int argc, const char*const* argv){
	if (argc == 2) {
	}else if (argc == 3) {
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
		}else if (strcmp(argv[1], "wired_interval") == 0) {
			wired_send_interval = atof(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "hello_interval") == 0) {
			hello_send_interval = atof(argv[2]);
			return (TCL_OK);
		}else if (strcasecmp (argv[1], "vehi-num") == 0) {
			current_veh_num =  atoi(argv[2]);
			return TCL_OK;
		}else if (strcmp(argv[1], "tavr_debug") == 0) {
			if (strcmp(argv[2], "true") == 0)debug_file = true;
			else debug_file = false;
			return (TCL_OK);
		}else if (strcmp(argv[1], "tavr_fileTime") == 0) {
			file_wirte_Time = atof(argv[2]);
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
		app_msg.vehicle_position_x_list[i] = hvh->vehicle_position_x();
		app_msg.vehicle_position_y_list[i] = hvh->vehicle_position_y();
		app_msg.vehicle_speed_list[i] = hvh->vehicle_speed();
		app_msg.recv_seqno_list[i] = 0;

//	}
	send_appdata();
	idle();
}

void
TAVRAppAgent::send_msg(hdr_wired_infos* hvh){

	if(current_veh_num > VEHICULAR_AMOUNT){
		fprintf(stderr,"error TAVRAppAgent::send_msg (info) ...veh_num overflow...");
		exit(1);
	}

//	if(type_of_node == COMM_NODE)fprintf(stdout,"---***TAVRAppAgent::send_msg start ");
	double curTime = Scheduler::instance().clock();

	double fresh_Degree = 0.00001;
	//decide the fresh degree of info
	if(type_of_node == COMM_NODE){
		for(int i=0; i< current_veh_num; i++){
			app_msg.vehicle_id_list[i] = hvh->vehicle_id_list[i];

			app_msg.vehicle_position_x_list[i] = hvh->vehicle_position_x_list[i];
			app_msg.vehicle_position_y_list[i] = hvh->vehicle_position_y_list[i];
			app_msg.vehicle_speed_list[i] = hvh->vehicle_speed_list[i];

			fresh_Degree = (curTime - vehicle_info_updateTime_LIST_[i]) / hello_send_interval;
			if(fresh_Degree -30000.0001 > 0.001){
				fprintf(stderr,"error TAVRAppAgent::send_msg (info) ...COMM.... The fresh degree of veh=%d is beyond the range",i);
				exit(1);
			}

			app_msg.recv_seqno_list[i] = (int16_t)floor(fresh_Degree);
		}
	}else if(type_of_node == WIRED_NODE){
		for(int i=0; i< current_veh_num; i++){
			app_msg.vehicle_id_list[i] = hvh->vehicle_id_list[i];

			app_msg.vehicle_position_x_list[i] = hvh->vehicle_position_x_list[i];
			app_msg.vehicle_position_y_list[i] = hvh->vehicle_position_y_list[i];
			app_msg.vehicle_speed_list[i] = hvh->vehicle_speed_list[i];


			fresh_Degree = (curTime - vehicle_info_updateTime_LIST_[i]) / hello_send_interval;// Alert, this interval is the comm timer, but here is common interval
			if(hvh->recv_seqno_list[i] < -0.001){
				fprintf(stderr,"error AppAgent::send_msg (info) ...common.... The fresh degree of veh=%d is Negative",i);
				exit(1);
			}else if(fresh_Degree -60000.0001 +  hvh->recv_seqno_list[i]> 0.001){
				fprintf(stderr,"error AppAgent::send_msg (info) ...common.... The fresh degree of veh=%d is beyond the range",i);
				exit(1);
			}

			app_msg.recv_seqno_list[i] = hvh->recv_seqno_list[i] + (int16_t)floor(fresh_Degree);
		}
	}else{
		fprintf(stderr,"error AppAgent::send_msg (info)  type_of_node=%d is NOT valid to send wired info",type_of_node);
		exit(1);
	}

	send_appdata();
	idle();
}

void
TAVRAppAgent::send_appdata(){
//	fprintf(stdout,"----***AppAgent::send_appdata start");
	switch(type_of_node){
	case WIRED_NODE:
		wired_send();//from veh to commander
//		fprintf(stdout,"--send_appdata--finish WIRED ");
		break;
	case BASE_NODE:break;
	case MOBILE_NODE:
		mobile_send();//to commander
//		fprintf(stdout,"--send_appdata--finish VEHICLE ");
		break;
	case COMM_NODE:
		comm_send();//from commander to veh
//		fprintf(stdout,"--send_appdata--finish COMM ");
		break;
	default:
		fprintf(stderr,"error AppAgent::send_appdata  type_of_node=%d",type_of_node);
		exit(1);
	}
}




void
TAVRAppAgent::comm_send(){//mac broadcast & IP broadcast
//	fprintf(stdout," TAVRAppAgent::comm_send will start......!!");
	char tmpStr[200];
	if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		sprintf(tmpStr,"\n\n TAVRAppAgent::comm_send will start......%0.4f!!\n",Scheduler::instance().clock());
		TAVRAppAgent::file_op.txl_write(tmpStr);
	}
	if(!app_){
		fprintf(stderr," TAVRAppAgent::comm_send TAVRAppAgent comm_send ERROR no app attached!!");
		exit(1);
	}

	if(current_veh_num > VEHICULAR_AMOUNT){
		fprintf(stderr,"error TAVRAppAgent::comm_send () ...veh_num overflow...");
		exit(1);
	}

	if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		sprintf(tmpStr,"\n comm_send -id-----x----y---v--recvSEQ-list %0.4f , the next retransmits will start\n",Scheduler::instance().clock());
		TAVRAppAgent::file_op.txl_write(tmpStr);
	}


	int netID = 0;
	switch(type_of_node){
	case MOBILE_NODE:netID = node_id + comm_id*2+1;break;
	case WIRED_NODE:netID = node_id;break;
	case COMM_NODE:netID = comm_id;break;
	}

	int wired_i= 0;
	for(; wired_i < node_id; wired_i++){
//	for(; wired_i < node_id; wired_i++){//debug 0709 for check AP receive
		Packet *p = allocpkt();
		struct hdr_cmn* ch = HDR_CMN(p);
		struct hdr_ip*	ih = HDR_IP(p);
		struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);

		packet_clean(p);

		ch->ptype() = PT_TAVRWIRED;
		ch->addr_type() = NS_AF_ILINK;
		unsigned int size = sizeof(hdr_veh_hello) * VEHICULAR_AMOUNT;
		ch->size() = (int)size;

		ch->direction() = hdr_cmn::DOWN;
		ch->timestamp() = Scheduler::instance().clock();
		char cip[14] = {'\0'};//may be a bug
		sprintf(cip,"0.0.%d",node_id);

		ih->ttl_ = NETWORK_DIAMETER;

		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001) && wired_i == 0){
			sprintf(tmpStr,"\n comm_send -id-----x----y---v--recvSEQ-list %0.4f \n",Scheduler::instance().clock());
			TAVRAppAgent::file_op.wirmsg_write(tmpStr);
		}
		int seqNO = 0;
		for(int i=0; i< current_veh_num; i++){
	//		if(curTime - app_->get_laest_update_tstamp_list()[i] - intervalRate*app_->get_send_interval() < 0.001){
			if(app_msg.vehicle_idlist()[i] > -1){
				reply->vehicle_id_list[i] = app_msg.vehicle_idlist()[i];

				reply->vehicle_position_x_list[i] = app_msg.vehicle_position_xlist()[i];
				reply->vehicle_position_y_list[i] = app_msg.vehicle_position_ylist()[i];
				reply->vehicle_speed_list[i] = app_msg.vehicle_speedlist()[i];
				reply->recv_seqno_list[i] = app_msg.recv_seqnolist()[i];


			}else{
				reply->vehicle_id_list[i] = -100;

				reply->vehicle_position_x_list[i] = -100;
				reply->vehicle_position_y_list[i] = -100;
				reply->vehicle_speed_list[i] = -100;
				reply->recv_seqno_list[i] = 111;
			}
			if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001) && wired_i == 0){
				sprintf(tmpStr,"comm_send\tid=%d\tx=%d\ty=%d\tv%d\tseq=%d\tstime=%0.4f\n",(i + comm_id*2+1),reply->vehicle_position_x_list[i],reply->vehicle_position_y_list[i],reply->vehicle_speed_list[i],reply->recv_seqno_list[i],ch->timestamp());
				TAVRAppAgent::file_op.wirmsg_write(tmpStr);
			}
		}
		char nip[14] = {'\0'};//may be a bug
		sprintf(nip,"0.0.%d",wired_i);
		ch->next_hop() = (u_int32_t)Address::instance().str2addr(nip);
//		sprintf(nip,"1.0.%d",wired_i);//---check 0709 for AP receive
		ih->daddr() = (u_int32_t)Address::instance().str2addr(nip);
		ih->dport() = this->port();
		Scheduler::instance().schedule(target_,p,TAVR_APP_COMM_TX_JITTER);
	}
}

void
TAVRAppAgent::wired_send(){//mac broadcast & IP unicast--------------------success
	char tmpStr[200];
	if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		sprintf(tmpStr,"\n TAVRAppAgent::wired_send will start...%d..%0.4f!!",node_id,Scheduler::instance().clock());
		TAVRAppAgent::file_op.txl_write(tmpStr);
	}
	if(!app_){
		fprintf(stderr," TAVRAppAgent::wired_send TAVRAppAgent wired_send ERROR no app attached!!");
		exit(1);
	}
	if(type_of_node == COMM_NODE){
		fprintf(stderr," TAVRAppAgent::wired_send TAVRAppAgent wired_send ERROR comm try to send msg from wired channel!!");
		exit(1);
	}

	if(current_veh_num > VEHICULAR_AMOUNT){
		fprintf(stderr,"error TAVRAppAgent::wired_send () ...veh_num overflow...");
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

	unsigned int size = sizeof(hdr_veh_hello) * VEHICULAR_AMOUNT;
	ch->size() = (int)size;

	ch->timestamp() = Scheduler::instance().clock();
	char cip[14] = {'\0'};//may be a bug
	sprintf(cip,"0.0.%d",node_id);
	ch->prev_hop_ =(u_int32_t)Address::instance().str2addr(cip);

	ih->saddr() = (u_int32_t)Address::instance().str2addr(cip);

	char dip[14] = {'\0'};//may be a bug
	int	 did = comm_id + 2;
	sprintf(dip,"1.0.%d",did);
	ih->daddr() = (u_int32_t)Address::instance().str2addr(dip);
	ih->dport() = this->port();
	ih->ttl_ = 4;
	if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		sprintf(tmpStr,"\n %d wired_send -id-----x----y---v--recvSEQ-list \tctime=%0.4f\n",(node_id + comm_id*2+1),Scheduler::instance().clock());
		TAVRAppAgent::file_op.wirmsg_write(tmpStr);
	}
	int seqNO = 0;
	for(int i=0; i< current_veh_num; i++){
//		if(curTime - app_->get_laest_update_tstamp_list()[i] - intervalRate*app_->get_send_interval() < 0.001){
		if(app_msg.vehicle_id_list[i] > -1){
			reply->vehicle_id_list[i] = app_msg.vehicle_idlist()[i];

			reply->vehicle_position_x_list[i] = app_msg.vehicle_position_xlist()[i];
			reply->vehicle_position_y_list[i] = app_msg.vehicle_position_ylist()[i];
			reply->vehicle_speed_list[i] = app_msg.vehicle_speedlist()[i];
			reply->recv_seqno_list[i] = app_msg.recv_seqnolist()[i];

		}else{//current vehicle NOT in this range
			reply->vehicle_id_list[i] = -100;

			reply->vehicle_position_x_list[i] = -100;
			reply->vehicle_position_y_list[i] = -100;
			reply->vehicle_speed_list[i] = -100;

		}
//		if(node_id == 0)fprintf(stdout,"** -%d----%d----%d---%d---%d**\n",i,reply->vehicle_position_x_list[i],reply->vehicle_position_y_list[i],reply->vehicle_speed_list[i],reply->recv_seqno_list[i]);
		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
			sprintf(tmpStr,"**wired_send \tid=%d\tx=%d\ty=%d\tv=%d\tseq=%d\tfresh=%0.4f\n",(i + comm_id*2+1),reply->vehicle_position_x_list[i],reply->vehicle_position_y_list[i],reply->vehicle_speed_list[i],reply->recv_seqno_list[i],vehicle_info_updateTime_LIST_[i]);
			TAVRAppAgent::file_op.wirmsg_write(tmpStr);
		}
	}
	Scheduler::instance().schedule(target_,p,TAVR_APP_WIRED_TX_JITTER);
}

void
TAVRAppAgent::mobile_send(){
	if(!app_){
		fprintf(stderr," TAVRAppAgent::mobile_send TAVRAppAgent comm_send ERROR no app attached!!");
		exit(1);
	}

	if(current_veh_num > VEHICULAR_AMOUNT){
		fprintf(stderr,"error TAVRAppAgent::mobile_send () ...veh_num overflow...");
		exit(1);
	}

	Packet *p = allocpkt();
	struct hdr_cmn* ch = HDR_CMN(p);
	ch->ptype() = PT_TAVRHELLO;
	struct hdr_ip*	ih = HDR_IP(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);

	packet_clean(p);


	unsigned int size = sizeof(hdr_veh_hello) * VEHICULAR_AMOUNT + sizeof(hdr_veh_hello);
	ch->size() = (int)size;
	ch->timestamp() = Scheduler::instance().clock();

	hdr_rtp* rh = hdr_rtp::access(p);
	rh->flags() = 0;
	rh->seqno() = ++seqno_;

	ih->ttl_ = 6;//since the closest wired node is at least two hops from current vehicle
//		hello->hello_seqno() = seqno_;
		hello->vehicle_id() = app_msg.vehicle_idlist()[node_id];
		hello->vehicle_position_x() = app_msg.vehicle_position_xlist()[node_id];
		hello->vehicle_position_y() = app_msg.vehicle_position_ylist()[node_id];
		hello->vehicle_speed() = app_msg.vehicle_speedlist()[node_id];

		Scheduler::instance().schedule(target_,p,TAVR_APP_MOBI_TX_JITTER);
}

void
TAVRAppAgent::recv(Packet* pkt, Handler*)
{
	char tmpStr[300] = {'\0'};
	hdr_cmn* cmnh = hdr_cmn::access(pkt);
	hdr_ip*  iph = hdr_ip::access(pkt);
	double	ctime = Scheduler::instance().clock();

	if (app_) {
		// If an application is attached, pass the data to the app

		if(cmnh->ptype() == PT_TAVRHELLO || cmnh->ptype() == PT_TAVRWIRED){
			if(cmnh->ptype() == PT_TAVRHELLO){
				switch(type_of_node){
				case WIRED_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  \tsip=%s, WIRED \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				case BASE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  \tsip=%s, BASE \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id + comm_id+1,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				case MOBILE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  \tsip=%s, MOBILE \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id + comm_id*2+1,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				case COMM_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  HELLO reach application layer!!  \tsip=%s, COMM \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				default:
					fprintf(stderr," AppAgent::recv HELLO reach application layer!! type ERROR");
					exit(1);
				}

//				fprintf(stdout,"%s",tmpStr);
				if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
					TAVRAppAgent::file_op.helmsg_write(tmpStr);
					TAVRAppAgent::file_op.txl_write(tmpStr);
				}
				recv_hello(type_of_node,pkt);
			}else if(cmnh->ptype() == PT_TAVRWIRED){
				switch(type_of_node){
				case WIRED_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  \tsip=%s, WIRED \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				case BASE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  \tsip=%s, BASE \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id + comm_id+1,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				case MOBILE_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  \tsip=%s, MOBILE \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id + comm_id*2+1,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				case COMM_NODE:
					sprintf(tmpStr,"\nAppAgent::recv  WIRED reach application layer!!  \tsip=%s, COMM \tcid=%d,\tdip=%s,\tstime=%0.4f\tctime=%0.4f",
						Address::instance().print_nodeaddr(iph->saddr()),
						node_id,
						Address::instance().print_nodeaddr(iph->daddr()),
						cmnh->timestamp(),
						ctime);
					break;
				default:
					fprintf(stderr," AppAgent::recv WIRED reach application layer!! type ERROR");
					exit(1);
				}

//				fprintf(stdout,"%s",tmpStr);
				if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
					TAVRAppAgent::file_op.wirmsg_write(tmpStr);
					TAVRAppAgent::file_op.txl_write(tmpStr);
				}
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
	hdr_cmn* cmnh = hdr_cmn::access(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);
	int nodeID = hello->vehicle_id();
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
		app_->recv_msg(MOBILE_NODE,hello);
		break;
	case COMM_NODE:
		vehicle_fresh_LIST_[nodeID] = 0;
		vehicle_info_updateTime_LIST_[nodeID] = cmnh->timestamp();

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
	hdr_cmn* cmnh = hdr_cmn::access(p);

	switch(type_of_node){
	case WIRED_NODE:
		for(int i=0; i < current_veh_num; i++){
			if(reply->vehicle_idlist() < 0)continue;


			vehicle_fresh_LIST_[i] = reply->recv_seqnolist()[i];
			vehicle_info_updateTime_LIST_[i] = cmnh->timestamp();
		}


		app_->recv_msg(WIRED_NODE,reply);
		break;
	case BASE_NODE:
		fprintf(stderr," AppAgent::recv_wired wired reach application layer!! a base node recv");
		exit(1);
	case MOBILE_NODE:
		app_->recv_msg(MOBILE_NODE,reply);
		break;
	case COMM_NODE:
		fprintf(stdout," AppAgent. The comm recv . ");
		break;
	default:
		fprintf(stderr," AppAgent::recv_wired wired reach application layer!! type ERROR");
		exit(1);
	}
}

void
TAVRAppAgent::packet_clean(Packet* p){
	struct hdr_wired_infos*	reply = hdr_wired_infos::access(p);
	struct hdr_veh_hello*	hello = hdr_veh_hello::access(p);

	for(int i=0; i< VEHICULAR_AMOUNT; i++){
		reply->vehicle_id_list[i] = -100;

		reply->vehicle_position_x_list[i] = -100;
		reply->vehicle_position_y_list[i] = -100;
		reply->vehicle_speed_list[i] = -100;
//	fprintf(stdout,"----%d  ",i);
	}

	hello->vehicle_id() = -100;

	hello->vehicle_position_x() = -100;
	hello->vehicle_position_y() = -100;
	hello->vehicle_speed() = -100;

}
