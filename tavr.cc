/*
 * tavr.cc
 *
 *  Created on: Jan 29, 2018
 *      Author: js
 */
#include "tavr.h"

#include <agent.h>
#include <mobilenode.h>
#include <trace.h>
#include <packet.h>
#include <classifier/classifier-port.h>
#include <cmu-trace.h>
#include <cmath>
#include "tavr_traffic_pkt.h"

//======define part===================
char GstrIP[34];
int  GnodeId[10];

#define LOC_JUNC_FLAG_X_PREV		1
#define LOC_JUNC_FLAG_X_NEXT		2
#define LOC_JUNC_FLAG_X_MID			3

#define LOC_JUNC_FLAG_Y_PREV		1
#define LOC_JUNC_FLAG_Y_NEXT		2
#define LOC_JUNC_FLAG_Y_MID			3


static class SIMUTAVRClass : public TclClass {
public:
	SIMUTAVRClass() : TclClass("Agent/SIMUTAVR") {}
	TclObject* create(int argc, const char*const* argv) {
		assert(argc == 5);
		return (new TAVRagent((nsaddr_t)Address::instance().str2addr(argv[4])));
	}
} class_rtSIMUTAVR;


void
TAVR_pktTimer::expire(Event* e) {
	if(Address::instance().get_firstaddr(agent_->vehicle_ip()) == 2)return;
	agent_->send_tavr_pkt();
	agent_->reset_tavr_timer();
}


TAVRagent::TAVRagent(nsaddr_t ip) : Agent(PT_SIMUTAVR), tavr_timer_(this){
	vehicle_ip_ = ip;
	vehicle_speed_ = -10;
	hello_amount_ = 0;
	firstHellorecv_ts_ = -100.2;
	inner_seqno_ = 0;

	node_ = (MobileNode*)Node::get_node_by_address(ip);

	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		vehicle_speed_LIST_[i] = -1000;
		vehicle_ip_LIST_[i] = (nsaddr_t)0;
		vehicle_position_x_LIST_[i] = -100.0;
		vehicle_position_y_LIST_[i] = -100.0;
	}

	conf_test_AXIS_ip = -100;
	conf_junc_row_bs = -100;
	conf_junc_col_bs = -100;
	debug_flag = true;
}

TAVRagent::~TAVRagent(){
	for(int i=0; i < conf_scenario_rowc; i++){
		delete[] junc_info_arrayX[i];
		delete[] junc_info_arrayY[i];
	}

	delete[] junc_info_arrayX;
	delete[] junc_info_arrayY;
}

void
TAVRagent::send_tavr_pkt() {//----------------------------------success

	Packet* p = allocpkt();
	struct hdr_cmn* 		ch = HDR_CMN(p);
	struct hdr_ip*			ih = HDR_IP(p);
	struct hdr_route_tavr*	tavrh = hdr_route_tavr::access(p);

	ch->size() = IP_HDR_LEN;
	ch->prev_hop_ = vehicle_ip();          //
	ch->next_hop() = IP_BROADCAST;
	ch->addr_type() = NS_AF_ILINK;

	ih->saddr() = vehicle_ip();
	ih->daddr() = IP_BROADCAST;
	ih->dport() = this->port();
	ih->ttl_ = 1;

	tavrh->seqno_ = inner_seqno_++;

	double time_delay = TAVR_JITTER;
//	time_delay += HELLO_INTERVAL;
/*	if(Address::instance().get_firstaddr(vehicle_ip()) == 1)
		fprintf(stdout,"\nmsg sip=%s cip=%s dip=%s will send after %0.2f domain=%d",
				Address::instance().print_nodeaddr(ch->prev_hop_),
				Address::instance().print_nodeaddr(vehicle_ip_),
				Address::instance().print_nodeaddr(ih->daddr()),
				time_delay,
				Address::instance().get_firstaddr(vehicle_ip()));*/
	Scheduler::instance().schedule(target_, p, time_delay);
}

void
TAVRagent::reset_tavr_timer() {
	tavr_timer_.resched((double)TAVR_BS_INTERVAL);
}

void
TAVRagent::update_vehicular_info(){
	node_->update_position();

	vehicle_position_x() = node_->X();
	vehicle_position_y() = node_->Y();
	vehicle_speed() = node_->speed();

	int nodeid = Address::instance().get_lastaddr(vehicle_ip());

	vehicle_position_x_LIST_[nodeid] =vehicle_position_x();
	vehicle_position_y_LIST_[nodeid] =vehicle_position_y();
	vehicle_speed_LIST_[nodeid] =vehicle_speed();
	junction_LIST_[nodeid] = true;
	bs_ip_LIST_[nodeid] = (u_int32_t)node_->base_stn();

	if(nodeid == 1){
		fprintf(stdout,"current vehinfo is x=%0.2f, y=%0.2f, speed=%0.2f",
				                     vehicle_position_x_LIST_[nodeid],
									 vehicle_position_y_LIST_[nodeid],
									 vehicle_speed_LIST_[nodeid]);

	}

	//update junction info
	int r=0;
	int c=0;
	double mini_Ystep = (conf_scenario_Height_ - 2*conf_starty_) / (conf_scenario_rowc - 1) / (conf_base_colc+1)/10;
	double mini_Xstep = (conf_scenario_Width_ - 2*conf_startx_) / (conf_scenario_colc - 1) / (conf_base_rowc+1)/10;
//	fprintf(stdout,"current mini step for locate junctions info is x=%0.2f, y=%0.2f",mini_Xstep,mini_Ystep);

	mini_Ystep = 10.002;
	mini_Xstep = 10.002;

	int posiX = 0;
	int posiY = 0;

	int juncXpre = 0;
	int juncYpre = 0;
	int juncXnxt = 0;
	int juncYnxt = 0;

//hard code for scale 1000
	posiX = (int)(vehicle_position_x_LIST_[nodeid] * 1000);
	posiY = (int)(vehicle_position_y_LIST_[nodeid] * 1000);

	junc_row_prev_list[nodeid] = -1;
	junc_row_next_list[nodeid] = -1;

	for(; r < conf_scenario_rowc; r++){

		juncYpre = (int)(junc_info_arrayY[r][0] * 1000);
		juncYnxt = (r+1 <conf_scenario_rowc)?(int)(junc_info_arrayY[r+1][0] * 1000):juncYpre;

		if(fabs(posiY - juncYpre) / 1000 < 8.0001){//near the row
//			Dist_prev = (posiY - juncYpre)/1000 * (posiY - juncYpre)/1000;
//			Dist_next = (posiY - juncYnxt)/1000 * (posiY - juncYnxt)/1000;
			junc_row_prev_list[nodeid] = r;
			junc_row_next_list[nodeid] = r;
			break;
		}else if((r+1 < conf_scenario_rowc) && (posiY > juncYpre) && ((juncYnxt - posiY) / 1000 > 7.0001)){//above the row, but below the next row
			junc_row_prev_list[nodeid] = r;
			junc_row_next_list[nodeid] = r+1;
			break;
		}
	}

	for(; c < conf_scenario_colc; c++){

		juncXpre = (int)(junc_info_arrayX[0][c] * 1000);
		juncXnxt = (c+1 <conf_scenario_colc)?(int)(junc_info_arrayX[0][c+1] * 1000):juncXpre;

		if(fabs(posiX - juncXpre) / 1000 < 8.0001){//near the row
			junc_col_prev_list[nodeid] = c;
			junc_col_next_list[nodeid] = c;
			break;
		}else if((c+1 < conf_scenario_colc) && (posiX > juncXpre) && ((juncXnxt - posiX) / 1000 > 7.0001)){//above the row, but below the next row
			junc_col_prev_list[nodeid] = c;
			junc_col_next_list[nodeid] = c+1;
			break;
		}

	}

//	fprintf(stdout,"\n TEST X_=%0.2f Y_=%0.2f r&c=%d %d ",vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],r+1,c+1);
	fprintf(stdout,"\n TEST X_=%0.2f Y_=%0.2f r&c=p%d p%d n%d n%d juncX=%0.2f juncY=%0.2f",
						vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],
						junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
						junc_info_arrayX[r][c],junc_info_arrayY[r][c]);


}
void
TAVRagent::recv(Packet *p, Handler *) {
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);

	if(ch->ptype() == PT_SIMUTAVR){
		fprintf(stdout,"---***---recv route manager  sip=%s cip=%s ",
					Address::instance().print_nodeaddr(ih->saddr()),
					Address::instance().print_nodeaddr(vehicle_ip()));
	}else if(ch->ptype() == PT_TAVRHELLO){
		fprintf(stdout,"\n---***---TAVRagent, recv hello  sip=%s cip=%s dip=%s",
					Address::instance().print_nodeaddr(ih->saddr()),
					Address::instance().print_nodeaddr(vehicle_ip()),
					Address::instance().print_nodeaddr(ih->daddr()));
	}else if(ch->ptype() == PT_TAVRWIRED){
		fprintf(stdout,"\n---***---TAVRagent, recv wired sip=%s cip=%s dip=%s",
					Address::instance().print_nodeaddr(ih->saddr()),
					Address::instance().print_nodeaddr(vehicle_ip()),
					Address::instance().print_nodeaddr(ih->daddr()));
	}else{
		if(ih->saddr() == vehicle_ip()) {
			//if there exists a loop, must drop the packet

			if (ch->num_forwards() > 0) {
				drop(p, DROP_RTR_ROUTE_LOOP);
				return;
			}
			//else if this is a packet I am originating, must add IP header
			else if (ch->num_forwards() == 0) {
				ch->size() += IP_HDR_LEN;
			}
		}
	}


	//if it is a BARV packet, must process it
	//otherwise, must forward the packet (unless TTL has reached zero)
	if (ch->ptype() == PT_SIMUTAVR) {
		recv_tavr_pkt(p);
	}else if (ch->ptype() == PT_TAVRHELLO) {
		recv_Hello(p);
	}else if (ch->ptype() == PT_TAVRWIRED) {
		recv_Wired(p);
	}else{
		tavr_forward_data(p);
	}

//	Packet::free(p);
}


void
TAVRagent::recv_tavr_pkt(Packet *p) {

	if(Address::instance().get_firstaddr(vehicle_ip()) != 2){
//		Packet::free(p);
		return;
	}

	struct hdr_ip		*ih = HDR_IP(p);


	fprintf(stdout, "update baseIP=%s of nodeIP=%s",
			Address::instance().print_nodeaddr(ih->saddr()),
			Address::instance().print_nodeaddr(vehicle_ip()));
	node_->set_base_stn(ih->saddr());
}

void
TAVRagent::recv_Wired(Packet *p) {
	fprintf(stdout,"\t** -----  ** route, recv_Wired");
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);
	struct hdr_wired_infos	*wiredh = hdr_wired_infos::access(p);


	int cnodeID = Address::instance().get_firstaddr(vehicle_ip());
	//recv
	int veh_amount = VEHICULAR_AMOUNT;
	for(int i=0; cnodeID == 2 && i < veh_amount; i++){

		if((wiredh->infos_tstamplist()[i] < vehicle_info_updateTime_list()[i]) || (wiredh->vehicle_idlist()[i] < 0)) continue;//if no info received , do NOT update the previous infos
		char ipStr[15] = {'\0'};
		sprintf(ipStr,"2.0.%d",wiredh->vehicle_idlist()[i]);

		fprintf(stdout," -recv_Wired- ipStr=%s -*- ",ipStr);

		vehicle_ip_LIST()[i] = (nsaddr_t)Address::instance().str2addr(ipStr);
		vehicle_position_x_LIST()[i] = wiredh->vehicle_position_xlist()[i];
		vehicle_position_y_LIST()[i] = wiredh->vehicle_position_ylist()[i];
		vehicle_speed_LIST()[i]      = wiredh->vehicle_speedlist()[i];
		vehicle_direction_LIST()[i]  = wiredh->vehicle_directionlist()[i];
		bs_ip_LIST()[i]              = wiredh->bs_idlist()[i];
		junc_row_prev_List()[i] = wiredh->junc_row_prevlist()[i];
		junc_col_prev_List()[i] = wiredh->junc_col_prevlist()[i];
		junc_row_next_List()[i] = wiredh->junc_row_nextlist()[i];
		junc_col_next_List()[i] = wiredh->junc_col_nextlist()[i];
		vehicle_info_updateTime_list()[i] = wiredh->infos_tstamplist()[i];
	}
	//to app
	if(cnodeID == 2){
		//to app
		Packet* toApp = allocpkt();
		struct hdr_wired_infos	*appInfo = hdr_wired_infos::access(toApp);
		struct hdr_cmn 			*appch = HDR_CMN(toApp);
		struct hdr_ip			*appih = HDR_IP(toApp);

		for(int i=0; i< VEHICULAR_AMOUNT; i++){
			appInfo->vehicle_id_list[i] = -100;
			appInfo->vehicle_direction_list[i] = -100;
			appInfo->vehicle_position_x_list[i] = -100;
			appInfo->vehicle_position_y_list[i] = -100;
			appInfo->vehicle_speed_list[i] = -100;
			appInfo->wired_id_list[i] = -100;
			appInfo->bs_id_list[i] = -100;
			appInfo->junc_col_next_list[i] =-100;
			appInfo->junc_row_next_list[i] = -100;
			appInfo->junc_col_prev_list[i] = -100;
			appInfo->junc_row_prev_list[i] = -100;
			appInfo->infos_tstamp_list[i] = -100;
		}

		appch->ptype() = ch->ptype();
		appch->direction() = hdr_cmn::UP;
		appch->next_hop() = vehicle_ip();
		appch->uid() = ch->uid();
		appch->addr_type() = NS_AF_ILINK;
		appch->size() = ch->size();

		appih->saddr() = ih->saddr();
		appih->sport() = ih->sport();
		appih->daddr() = vehicle_ip();
		appih->dport() = 2;
//		appih->dport() = 50;

		appih->ttl() = ih->ttl();
		appih->flowid() = ih->flowid();

		fprintf(stdout," -cip=%s to upper layer -*- ",Address::instance().print_nodeaddr(vehicle_ip()));
//		if(debug_flag){
//			debug_flag = false;
			dmux_->recv(toApp,(Handler*)0);
//		}
	}else{//base NODE

		ch->direction() = hdr_cmn::DOWN;
		ch->addr_type() = NS_AF_ILINK;
		ch->next_hop() = IP_BROADCAST;

		ih->daddr() = IP_BROADCAST;
		ih->ttl_ = 5;
		Scheduler::instance().schedule(target_,p,TAVR_JITTER);
	}
}

void
TAVRagent::recv_Hello(Packet *p) {
	fprintf(stdout,"** -----  ** route, recv_hello");
	struct hdr_veh_hello	*hello = hdr_veh_hello::access(p);
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);

	if(ih->saddr() == vehicle_ip()){//to links

		update_vehicular_info();

		//forward
		int cnodeID = Address::instance().get_lastaddr(vehicle_ip());
		int pnodeID = hello->vehicle_id();
		if(cnodeID != pnodeID){
			fprintf(stderr,"ERROR, a hello packet from %d will be filled up with %d's info",pnodeID,cnodeID);
			exit(1);
		}

		hello->vehicle_position_x() = vehicle_position_x_LIST()[cnodeID];
		hello->vehicle_position_y() = vehicle_position_y_LIST()[cnodeID];
		hello->vehicle_speed() = vehicle_speed_LIST()[cnodeID];
		hello->vehicle_direction() = vehicle_direction_LIST()[cnodeID];
		hello->bs_id() = bs_ip_LIST()[cnodeID];
		hello->junc_row_prev() = junc_row_prev_List()[cnodeID];
		hello->junc_col_prev() = junc_col_prev_List()[cnodeID];
		hello->junc_row_next() = junc_row_next_List()[cnodeID];
		hello->junc_col_next() = junc_col_next_List()[cnodeID];
		hello->hello_tstamp()  = vehicle_info_updateTime_list()[cnodeID];

//		ch->next_hop() = (u_int32_t)node_->base_stn();
		ch->next_hop() = IP_BROADCAST;
		ch->addr_type() = NS_AF_ILINK;
		ch->direction() = hdr_cmn::DOWN;

		Scheduler::instance().schedule(target_, p, TAVR_JITTER);
	}else if(Address::instance().get_firstaddr(vehicle_ip()) == 2){//from neighbors
		//recv
		int cnodeID = hello->vehicle_id();

		if(cnodeID < 0 || cnodeID >= VEHICULAR_AMOUNT){
			fprintf(stderr,"ERROR, a hello packet .with nodeID invalid...tavr-----recv_Hello..");
			exit(1);
		}
		if(hello->hello_tstamp()< vehicle_info_updateTime_list()[cnodeID]){
			fprintf(stderr,"ERROR, a hello packet .with info out-of-time...tavr-----recv_Hello..");
			exit(1);
		}

		char ipStr[15] = {'\0'};
		sprintf(ipStr,"2.0.%d",hello->vehicle_id());
		fprintf(stdout," -recv_Hello- ipStr=%s -*- ",ipStr);

		vehicle_ip_LIST_[cnodeID] = Address::instance().str2addr(ipStr);
		vehicle_position_x_LIST_[cnodeID] = hello->vehicle_position_x();
		vehicle_position_y_LIST_[cnodeID] = hello->vehicle_position_y();
		vehicle_speed_LIST_[cnodeID]      = hello->vehicle_speed();
		vehicle_direction_LIST_[cnodeID]  = hello->vehicle_direction();
		bs_ip_LIST_[cnodeID]              = hello->bs_id();
		junc_row_prev_list[cnodeID] = hello->junc_row_prev();
		junc_col_prev_list[cnodeID] = hello->junc_col_prev();
		junc_row_next_list[cnodeID] = hello->junc_row_next();
		junc_col_next_list[cnodeID] = hello->junc_col_next();
		vehicle_info_updateTime_LIST_[cnodeID] = hello->hello_tstamp();

		//to app
		Packet* toApp = allocpkt();
		struct hdr_veh_hello	*appHello = hdr_veh_hello::access(toApp);
		struct hdr_cmn 			*appch = HDR_CMN(toApp);
		struct hdr_ip			*appih = HDR_IP(toApp);

		appih->saddr() = ih->saddr();
		appih->sport() = ih->sport();
		appih->daddr() = vehicle_ip();
		appih->dport() = ih->sport();//may be a bug, since the dst port is changed
		appih->ttl() = ih->ttl();
		appih->flowid() = ih->flowid();

		appch->ptype() = ch->ptype();
		appch->direction() = hdr_cmn::UP;
		appch->next_hop() = vehicle_ip();
		appch->uid() = ch->uid();
		appch->size() = ch->size();

		appHello->vehicle_id() = hello->vehicle_id();
		appHello->vehicle_position_x() = hello->vehicle_position_x();
		appHello->vehicle_position_y() = hello->vehicle_position_y();
		appHello->vehicle_speed()      = hello->vehicle_speed();
		appHello->vehicle_direction()  = hello->vehicle_direction();
		appHello->bs_id()              = hello->bs_id();
		appHello->junc_row_prev() = hello->junc_row_prev();
		appHello->junc_col_prev() = hello->junc_col_prev();
		appHello->junc_row_next() = hello->junc_row_next();
		appHello->junc_col_next() = hello->junc_col_next();
		appHello->hello_tstamp() = hello->hello_tstamp();

//		Packet::free(p);//may be a bug, since the pointer argument destroy the packet recv by other nodes.
		dmux_->recv(toApp,(Handler*)0);
	}else if(Address::instance().get_firstaddr(vehicle_ip()) == 1){//base NODE recv & forward
		fprintf(stdout,"\n   ***   Alert, a hello packet to AP=%s....tavr-----recv_Hello..",Address::instance().print_nodeaddr(vehicle_ip()));

		tavr_forward_data(p);
	}else{
		fprintf(stderr,"ERROR, a hello packet ....tavr-----recv_Hello..");
		exit(1);
	}

	//forward

}

void TAVRagent::tavr_forward_data(Packet *p) {

	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);

	ih->ttl_--;
	if (ih->ttl_ == 0) {
		drop(p, DROP_RTR_TTL);
		return;
	}

	if(ih->daddr() == vehicle_ip()){
		if(ch->direction() == hdr_cmn::UP){// packets for applications
			dmux_->recv(p,0);
		}
	}else{
		Scheduler::instance().schedule(target_, p, 0.);
	}

}


int TAVRagent::command(int argc, const char*const* argv) {
	if (argc == 2) {
		if (strcasecmp(argv[1], "start") == 0) {
			tavr_timer_.resched(0.0);
			return TCL_OK;
		}else if (strcasecmp(argv[1], "print_rtable") == 0) {
			double Time_Now = CURRENT_TIME;
			if (logtarget_ != 0) {
				sprintf(logtarget_->pt_->buffer(),"P %0.2f_%d_ Routing Table",Time_Now,vehicle_ip());
					logtarget_->pt_->dump();
	//				rtable_.print(logtarget_);
			}
			else {
				fprintf(stdout, "%f _%d_ If you want to print this routing table you must create a trace file in your tcl script", Time_Now,vehicle_ip());
			}
			return TCL_OK;
		}else if (strcasecmp (argv[1], "printbs") == 0) {
			print_bs();
			return TCL_OK;
		}else if (strcasecmp (argv[1], "init_juncInfo") == 0) {
			init_juncInfo();
			return TCL_OK;
		}
	}
	else if (argc == 3) {
			//Obtains corresponding dmux to carry packets to upper layers
		TclObject *obj;
/*		if ((obj = TclObject::lookup (argv[2])) == 0)
		{
			fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
					argv[2]);
			return TCL_ERROR;
		}*/
		if (strcasecmp(argv[1], "port-dmux") == 0) {
			dmux_ = (PortClassifier*)TclObject::lookup(argv[2]);
			if (dmux_ == 0) {
				fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1], argv[2]);
				return TCL_ERROR;
			}
			return TCL_OK;
		}
		else if (strcmp(argv[1], "log-target") == 0 ||
					 strcmp(argv[1], "tracetarget") == 0) {
				logtarget_ = (Trace*)TclObject::lookup(argv[2]);
			if(logtarget_  == 0) {
				fprintf(stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1], argv[2]);
				return TCL_ERROR;
			}
			return TCL_OK;
		}
		else if (strcasecmp (argv[1], "node") == 0) {
//			 node_ = (MobileNode*) obj;
			 return TCL_OK;
		}
		else if (strcasecmp (argv[1], "vehi-num") == 0) {
			conf_scenario_vehicular_amout = atoi(argv[2]);
//			fprintf(stdout,"conf_scenario_vehicular_amout=%d",conf_scenario_vehicular_amout);
			return TCL_OK;
		}else if (strcasecmp (argv[1], "base-num") == 0) {
			conf_scenario_base_amout = atoi(argv[2]);
//			fprintf(stdout,"conf_scenario_base_amout=%d",conf_scenario_base_amout);
			return TCL_OK;
		}else if (strcasecmp (argv[1], "msgINET") == 0) {
			conf_test_INET = atoi(argv[2]);
			return TCL_OK;
		}else if (strcasecmp (argv[1], "nodeID") == 0) {
			conf_node_id = atoi(argv[2]);
			return TCL_OK;
		}else if (strcmp(argv[1], "comm_id") == 0) {
			comm_id = atoi(argv[2]);
			return (TCL_OK);
		}else if (strcasecmp (argv[1], "AXIS_ip") == 0) {
			conf_test_AXIS_ip = atoi(argv[2]);
			return TCL_OK;
		}

	}

	else if (argc == 6) {
		if (strcasecmp (argv[1], "conf-map") == 0) {
//			printf(".....I'm six...start.");
			conf_scenario_Width_ = atof(argv[2]);
			conf_scenario_Height_ = atof(argv[3]);
			conf_scenario_rowc = atoi(argv[4]);
			conf_scenario_colc = atoi(argv[5]);
			return TCL_OK;
//			printf(".....I'm six...end.w=%0.2f, h=%0.2f, r=%d, c=%d",conf_scenario_Width_,conf_scenario_Height_,conf_scenario_rowc,conf_scenario_colc);
		}else if (strcasecmp (argv[1], "conf-base") == 0) {
//			printf(".....I'm six...start.");
			conf_startx_ = atof(argv[2]);
			conf_starty_ = atof(argv[3]);
			conf_base_rowc = atoi(argv[4]);
			conf_base_colc = atoi(argv[5]);
			return TCL_OK;
//			printf(".....I'm six...end.w=%0.2f, h=%0.2f, r=%d, c=%d",conf_scenario_Width_,conf_scenario_Height_,conf_scenario_rowc,conf_scenario_colc);
		}
	}
	//Pass the command to the base class
	return Agent::command(argc,argv);
}


void
TAVRagent::print_bs(){

}

//=====================test part======================

void
TAVRagent::init_juncInfo(){
	junc_info_arrayX = new double*[conf_scenario_rowc];
	junc_info_arrayY = new double*[conf_scenario_rowc];

	for(int i=0; i < conf_scenario_rowc; i++){
		junc_info_arrayX[i] = new double[conf_scenario_colc];
		junc_info_arrayY[i] = new double[conf_scenario_colc];
	}

	double Ystep = (conf_scenario_Height_ - 2*conf_starty_) / (conf_scenario_rowc - 1);
	double Xstep = (conf_scenario_Width_ - 2*conf_startx_) / (conf_scenario_colc - 1);


	for(int r=0; r < conf_scenario_rowc; r++){
		for(int c=0; c < conf_scenario_colc; c++){
			junc_info_arrayX[r][c] = conf_startx_ + c*Xstep;
			junc_info_arrayY[r][c] = conf_starty_ + r*Ystep;
		}
	}

//	printf("Rabbit, %u am here. The command has finished", vehicle_ip());
}


