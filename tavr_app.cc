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

TAVRApp::TAVRApp():tsend_timer(this),treply_timer(this),running_(0),seqno_(0){
	type_of_node = 0;
	node_id = 0;

	send_interval_ = 1000;
	reply_interval_ = 1000;


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
	}
}

TAVRApp::~TAVRApp(){

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
	if(running_){
		double jitter_tmp = TAVR_JITTER;
		double timer_delay = send_interval_ + jitter_tmp;
//		fprintf(stdout,"\nnode %d type %d will start transmission at %0.2f + %0.2f= %0.2f",node_id,type_of_node,send_interval_ ,jitter_tmp,timer_delay);
		send_tavr_pkt();
		tsend_timer.resched(timer_delay);
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

	switch(type_of_node){
	case MOBILE_NODE:
//		fprintf(stdout,"\nAPP----vehicle %d send ",node_id);
		hdr_veh_hello hello_hdr;
		prepare_data_for_Agent(&hello_hdr);
		agent_->send_msg(&hello_hdr);break;
	case WIRED_NODE:
	case COMM_NODE:
//		if(type_of_node == WIRED_NODE) fprintf(stdout,"\nAPP----wired %d send ",node_id);
//		else fprintf(stdout,"\nAPP----COMM %d send ",node_id);
		hdr_wired_infos info_hdr;
		prepare_data_for_Agent(&info_hdr);
		agent_->send_msg(&info_hdr);break;
	}

}


void
TAVRApp::recv_msg(int nbytes, hdr_veh_hello *msg){
	int i = 0;
	i = msg->vehicle_id();
	if(msg->hello_tstamp() < laest_update_tstamp_list[i]){
		return;
	}


	vehicle_id_list[i] = msg->vehicle_id();
	vehicle_direction_list[i] = msg->vehicle_direction();
	vehicle_position_x_list[i] = msg->vehicle_position_x();
	vehicle_position_y_list[i] = msg->vehicle_position_y();
	vehicle_speed_list[i] = msg->vehicle_speed();
	wired_id_list[i] = msg->wired_id();
	bs_id_list[i] = msg->bs_id();
	junc_col_next_list[i] = msg->junc_col_next();
	junc_row_next_list[i] = msg->junc_row_next();
	junc_col_prev_list[i] = msg->junc_col_prev();
	junc_row_prev_list[i] = msg->junc_row_prev();
	laest_update_tstamp_list[i] = msg->hello_tstamp();


}

void
TAVRApp::recv_msg(int nbytes, hdr_wired_infos *msg){
	int i = 0;
	i=0;
	for(; i < VEHICULAR_AMOUNT; i++){

		if((msg->vehicle_idlist()[i] < 0) || (msg->infos_tstamplist()[i] < laest_update_tstamp_list[i])){
			continue;
		}

		vehicle_id_list[i] = msg->vehicle_idlist()[i];
		vehicle_direction_list[i] = msg->vehicle_directionlist()[i];
		vehicle_position_x_list[i] = msg->vehicle_position_xlist()[i];
		vehicle_position_y_list[i] = msg->vehicle_position_ylist()[i];
		vehicle_speed_list[i] = msg->vehicle_speedlist()[i];
		wired_id_list[i] = msg->wired_idlist()[i];
		bs_id_list[i] = msg->bs_idlist()[i];
		junc_col_next_list[i] = msg->junc_col_nextlist()[i];
		junc_row_next_list[i] = msg->junc_row_nextlist()[i];
		junc_col_prev_list[i] = msg->junc_col_prevlist()[i];
		junc_row_prev_list[i] = msg->junc_row_prevlist()[i];
		laest_update_tstamp_list[i] = msg->infos_tstamplist()[i];
	}
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

//copy the info . done in the routing layer
}

void
TAVRApp::prepare_data_for_Agent(hdr_wired_infos *reply){
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
}

int
TAVRApp::command(int argc, const char*const* argv){
	Tcl& tcl = Tcl::instance();

	if (argc == 2) {
		if (strcmp(argv[1], "starttx") == 0) {
			tsend_timer.expire(0);
			return (TCL_OK);
		}else if (strcmp(argv[1], "startrx") == 0) {
			treply_timer.expire(0);
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
