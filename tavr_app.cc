/*
 * tavr_app.cc
 *
 *  Created on: Mar 16, 2018
 *      Author: js
 */
#include "tavr_app.h"
#include "tavr_traffic_pkt.h"
#include "tavr_Appagent.h"
#include <stdio.h>

struct hdr_wired_infos;
struct hdr_veh_hello;

//tavrApp OTCL linkage class
static class TAVRAppClass : public TclClass {
public:
	TAVRAppClass() : TclClass("Application/TAVRApp") {}
	TclObject* create(int, const char*const*){
		return (new TAVRApp());
	}
}class_tavr_app;

void
SendtavrTimer::expire(Event* e){
	t_->stimeout();
}

void
ReplytavrTimer::expire(Event* e){
	t_->rtimeout();
}

TAVRfile TAVRApp::file_op;
bool TAVRApp::file_exist = false;

TAVRApp::TAVRApp():tsend_timer(this),treply_timer(this),running_(0),seqno_(0){
	type_of_node = 0;
	node_id = 0;

	send_interval_ = 1000;
	reply_interval_ = 1000;

	if(!TAVRApp::file_exist){
		TAVRApp::file_op.init_file("xxxx_tavr",101011);
		TAVRApp::file_op.app_write("------------this is application layer CONTROL file................\n");
		TAVRApp::file_op.helmsg_write("------------this is application layer HELLO file................\n");
		TAVRApp::file_op.wirmsg_write("------------this is application layer WIRED file................\n");
		TAVRApp::file_exist = true;
	}

	debug_file = false;

	for(int i=0; i< VEHICULAR_AMOUNT; i++){
		vehicle_id_list[i] = -100;
		vehicle_direction_list[i] = -100;
		vehicle_position_x_list[i] = -100;
		vehicle_position_y_list[i] = -100;
		vehicle_speed_list[i] = -100;
		wired_id_list[i] = -100;
		bs_id_list[i] = -100;
		junc_col_next_list[i] =-100;
		junc_row_next_list[i] = -100;
		junc_col_prev_list[i] = -100;
		junc_row_prev_list[i] = -100;
		laest_update_tstamp_list[i] = -100;
		recv_seqno_list[i] = 30;
	}
}

TAVRApp::~TAVRApp(){
	TAVRApp::file_op.close();
}

void
TAVRApp::start(){
//	running_ = 1;
}

void
TAVRApp::stop(){
	running_ = 0;
}

void
TAVRApp::stimeout(){
	char tmpStr[140];
	int netID = 0;
	switch(type_of_node){
	case MOBILE_NODE:netID = node_id + comm_id*2+1;break;
	case WIRED_NODE:netID = node_id;break;
	case COMM_NODE:netID = comm_id;break;
	}

	if(running_){
		double jitter_tmp = TAVR_JITTER;
		double timer_delay = send_interval_ + jitter_tmp;
		if(debug_file && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
//			sprintf(tmpStr,"\nnode %d type %d will retransmission at \t%0.4f + %0.4f =\t%0.4f",node_id,type_of_node, Scheduler::instance().clock(),timer_delay,(timer_delay + Scheduler::instance().clock()));
			sprintf(tmpStr,"\nnode %d type %d will retransmission at \t%0.4f + %0.4f =\t%0.4f",netID,type_of_node, Scheduler::instance().clock(),send_interval_,(Scheduler::instance().clock()+send_interval_));
			TAVRApp::file_op.app_write(tmpStr);
		}

		send_tavr_pkt();
		tsend_timer.resched(send_interval_);
	}
}

void
TAVRApp::rtimeout(){
	if(running_){
//		send_tavr_pkt();
//		send_timer.resched(send_interval_);
	}
}

void
TAVRApp::send_tavr_pkt(){
	char tmpStr[100];
	int netID = 0;
	switch(type_of_node){
	case MOBILE_NODE:netID = node_id + comm_id*2+1;break;
	case WIRED_NODE:netID = node_id;break;
	case COMM_NODE:netID = comm_id;break;
	}
	switch(type_of_node){
	case MOBILE_NODE:
//		fprintf(stdout,"\nAPP----vehicle %d send ",node_id);
		sprintf(tmpStr,"\tAPP----vehicle %d send ",netID);
		hdr_veh_hello hello_hdr;
		prepare_data_for_Agent(&hello_hdr);
		agent_->send_msg(&hello_hdr);break;
	case WIRED_NODE:
	case COMM_NODE:
		if(type_of_node == WIRED_NODE) sprintf(tmpStr,"\tAPP----wired %d send ",netID);
		else sprintf(tmpStr,"\tAPP----COMM %d send ",netID);
		hdr_wired_infos info_hdr;
		prepare_data_for_Agent(&info_hdr);
		agent_->send_msg(&info_hdr);break;
	}
	if(debug_file && (Scheduler::instance().clock() > file_wirte_Time + 0.001))
		TAVRApp::file_op.app_write(tmpStr);
}


void
TAVRApp::recv_msg(int nbytes, hdr_veh_hello *msg){
	int i = 0;
	i = msg->vehicle_id();

	int netID = i + comm_id*2+1;

	vehicle_id_list[i] = msg->vehicle_id();
	vehicle_position_x_list[i] = msg->vehicle_position_x();
	vehicle_position_y_list[i] = msg->vehicle_position_y();
	vehicle_speed_list[i] = msg->vehicle_speed();

	if(type_of_node == COMM_NODE && debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){

		char cont[200];

		sprintf(cont,"\n**recv_hello \tNO.=%d\tid=%d\tx=%d\ty=%d\tv=%d\tseq=%d\tctime=%0.2f",current_veh_num,netID,vehicle_position_x_list[i],vehicle_position_y_list[i],vehicle_speed_list[i],recv_seqno_list[i],
																Scheduler::instance().clock());
		TAVRApp::file_op.helmsg_write(cont);
//		fprintf(stdout,"** %d--%d----%d----%d---%d---%d**",current_veh_num,i,vehicle_position_x_list[i],vehicle_position_y_list[i],vehicle_speed_list[i],recv_seqno_list[i]);
	}
}

void
TAVRApp::recv_msg(int nbytes, hdr_wired_infos *msg){

	if(current_veh_num > VEHICULAR_AMOUNT){
		fprintf(stderr,"error TAVRApp::recv_msg (info) ...veh_num overflow...");
		exit(1);
	}

	int netID = 0;
	switch(type_of_node){
	case MOBILE_NODE:netID = node_id + comm_id*2+1;break;
	case WIRED_NODE:netID = node_id;break;
	case COMM_NODE:netID = comm_id;break;
	}

	char cont[200];
	int i = 0;
	int not_fresh_count = 0;
	for(; i < current_veh_num; i++){
		int vID = i + comm_id*2+1;
/*		if((msg->vehicle_idlist()[i] < 0) || (msg->infos_tstamplist()[i] < laest_update_tstamp_list[i])){
			continue;
		}*/
		if((msg->vehicle_idlist()[i] < 0)){
			if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
				sprintf(cont,"\n recv wired+-* +-* ERROR +-* +-* +-*TAVRApp::recv_msg (info) ...wired.... veh=%d LOST...",i);
				TAVRApp::file_op.wirmsg_write(cont);
			}
			not_fresh_count++;
		}else{
			vehicle_id_list[i] = msg->vehicle_idlist()[i];
			vehicle_position_x_list[i] = msg->vehicle_position_xlist()[i];
			vehicle_position_y_list[i] = msg->vehicle_position_ylist()[i];
			vehicle_speed_list[i] = msg->vehicle_speedlist()[i];
			recv_seqno_list[i] = msg->recv_seqnolist()[i];

			if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
				if(type_of_node == WIRED_NODE)sprintf(cont,"\n**wired=%d\trecv wired \tNO.=%d\tid=%d\tx=%d\ty=%d\tv=%d\tseq=%d\tctime=%0.2f",netID,current_veh_num,vID,vehicle_position_x_list[i],vehicle_position_y_list[i],vehicle_speed_list[i],recv_seqno_list[i],
																		Scheduler::instance().clock());
				else if(type_of_node == MOBILE_NODE)sprintf(cont,"\n**vehicle=%d\trecv wired \tNO.=%d\tid=%d\tx=%d\ty=%d\tv=%d\tseq=%d\tctime=%0.2f",netID,current_veh_num,vID,vehicle_position_x_list[i],vehicle_position_y_list[i],vehicle_speed_list[i],recv_seqno_list[i],
																		Scheduler::instance().clock());
				TAVRApp::file_op.wirmsg_write(cont);
			}

		}


	}

/*	if((not_fresh_count > 10)){//hard code
		fprintf(stderr,"error TAVRApp::recv_msg (info) ...wired.... %d LOST...",not_fresh_count);
		exit(1);
	}*/
}

int
TAVRApp::get_type_node(){
	return type_of_node;
}

int
TAVRApp::get_id(){
	return node_id;
}

void
TAVRApp::prepare_data_for_Agent(hdr_veh_hello *hello){
//clean the data
	hello->vehicle_id() = -100;

	hello->vehicle_position_x() = -100;
	hello->vehicle_position_y() = -100;
	hello->vehicle_speed() = -100;
	hello->send_seqno_ = -100;
}

void
TAVRApp::prepare_data_for_Agent(hdr_wired_infos *reply){
	char tmpStr[200];
	if(current_veh_num > VEHICULAR_AMOUNT){
		fprintf(stderr,"error TAVRApp::prepare_data_for_Agent (info) ...veh_num overflow...");
		exit(1);
	}

	if(type_of_node == COMM_NODE && debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		sprintf(tmpStr,"  ----COMM----info will be sent out.....%0.4f\t",Scheduler::instance().clock());
		TAVRApp::file_op.app_write(tmpStr);
		sprintf(tmpStr,"\n----COMM----info will be sent out.....%0.4f\n",Scheduler::instance().clock());
		TAVRApp::file_op.wirmsg_write(tmpStr);
		sprintf(tmpStr,"\n\n--------------COMM--------this round of hello recv finished.............%0.4f\n",Scheduler::instance().clock());
		TAVRApp::file_op.helmsg_write(tmpStr);
	}else if(type_of_node == WIRED_NODE && debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		sprintf(tmpStr,"  ----WIRED----info will be sent out.....%0.4f\t",Scheduler::instance().clock());
		TAVRApp::file_op.app_write(tmpStr);
		sprintf(tmpStr,"\n----WIRED----info will be sent out.....%0.4f\n",Scheduler::instance().clock());
		TAVRApp::file_op.wirmsg_write(tmpStr);
//		TAVRApp::file_op.helmsg_write("\n\n---WIRED---------this round of hello recv finished---------------\n");
	}



	for(int i=0; i< VEHICULAR_AMOUNT; i++){
		reply->vehicle_id_list[i] = -100;
		reply->vehicle_position_x_list[i] = -100;
		reply->vehicle_position_y_list[i] = -100;
		reply->vehicle_speed_list[i] = -100;
		reply->recv_seqno_list[i] = -100;

	}

	for(int i=0; i< current_veh_num; i++){
		int vID = i + comm_id*2+1;
		reply->vehicle_id_list[i] = vehicle_id_list[i];
		reply->vehicle_position_x_list[i] = vehicle_position_x_list[i];
		reply->vehicle_position_y_list[i] = vehicle_position_y_list[i];
		reply->vehicle_speed_list[i] = vehicle_speed_list[i];
		reply->recv_seqno_list[i] = recv_seqno_list[i];

		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
			sprintf(tmpStr,"\n**prepare data NO.\t=%d\tid=%d\tx=%d\ty=%d\tv=%d\tseq=%d\tctime=%0.4f",current_veh_num,vID,vehicle_position_x_list[i],vehicle_position_y_list[i],vehicle_speed_list[i],recv_seqno_list[i],
																				Scheduler::instance().clock());
			TAVRApp::file_op.wirmsg_write(tmpStr);
		}

	}
}

int
TAVRApp::command(int argc, const char*const* argv){
	Tcl& tcl = Tcl::instance();

	if (argc == 2) {
		if (strcmp(argv[1], "starttx") == 0) {
			tsend_timer.expire(0);
			return (TCL_OK);
		}else if (strcmp(argv[1], "startrx") == 0) {
//			treply_timer.expire(0);
			return (TCL_OK);
		}else if (strcmp(argv[1], "stoptx") == 0) {
			send_interval_ = 1000000;
			return (TCL_OK);
		}else if (strcmp(argv[1], "getSinterval") == 0) {
			fprintf(stdout,"the sendTimer is %0.2f",send_interval_);
			return (TCL_OK);
		}
	} else if (argc == 3) {
		if (strcmp(argv[1], "txinterval") == 0) {
			send_interval_ = atof(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "running") == 0) {
			running_ = atoi(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "rxinterval") == 0) {
			reply_interval_ = atof(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "node_type") == 0) {
			type_of_node = atoi(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "node_id") == 0) {
			node_id = atoi(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "comm_id") == 0) {
			comm_id = atoi(argv[2]);
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
		}else if (strcmp(argv[1], "attach-agent") == 0) {
			agent_ = (Agent*) TclObject::lookup(argv[2]);
			if(agent_ == 0){
				tcl.resultf("no such agent %s",argv[2]);
				return (TCL_ERROR);
			}


			agent_->attachApp(this);
			return (TCL_OK);
		}
	}
	return (Application::command(argc, argv));
}
