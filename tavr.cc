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




static class SIMUTAVRClass : public TclClass {
public:
	SIMUTAVRClass() : TclClass("Agent/SIMUTAVR") {}
	TclObject* create(int argc, const char*const* argv) {
		assert(argc == 5);
//		fprintf(stdout,"\n------------ip=%s",argv[4]);
		return (new TAVRagent((nsaddr_t)Address::instance().str2addr(argv[4])));
	}
} class_rtSIMUTAVR;


void
TAVR_pktTimer::expire(Event* e) {
	agent_->send_tavr_pkt();
	agent_->reset_tavr_timer();
}


TAVRagent::TAVRagent(nsaddr_t ip) : Agent(PT_SIMUTAVR), tavr_timer_(this){
	vehicle_ip_ = ip;
//	fprintf(stdout,"--***--saved as =%s",Address::instance().print_nodeaddr(vehicle_ip_));
	vehicle_speed_ = -10;
	hello_amount_ = 0;
	firstHellorecv_ts_ = -100.2;

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
TAVRagent::send_tavr_pkt() {

	Packet *p = Packet::alloc();
	struct hdr_cmn* 		ch = HDR_CMN(p);
	struct hdr_ip*			ih = HDR_IP(p);
	struct hdr_route_tavr*	tavrh = hdr_route_tavr::access(p);

	node_->update_position();
	vehicle_position_x() = node_->X();
	vehicle_position_y() = node_->Y();
	vehicle_speed() = node_->speed();

	//add self info into list
//	double times_now = CURRENT_TIME;

	ch->size() += IP_HDR_LEN;
	ch->direction() = hdr_cmn::DOWN;
	ch->iface() = -2;
	ch->error() = 0;
	ch->addr_type() = NS_AF_INET;
	ch->prev_hop_ = vehicle_ip();          //
	ch->next_hop() = IP_BROADCAST;


	ih->saddr() = vehicle_ip();
	ih->daddr() = IP_BROADCAST;
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl_ = 3;

	tavrh->seqno_ = 102;

	if(get_domain(vehicle_ip()) > 1){
		double time_delay = JITTER;
		Scheduler::instance().schedule(target_, p, time_delay);
	}

}

void
TAVRagent::reply_tavr_pkt(Packet* pp) {

	struct hdr_ip			*sih = HDR_IP(pp);

	Packet 					*p = allocpkt();
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);
	struct hdr_route_tavr	*tavrh = hdr_route_tavr::access(p);

	update_vehicular_info();

//	ih->daddr() = IP_BROADCAST;


	ch->ptype() = PT_SIMUTAVR;
	ch->size() += IP_HDR_LEN;
	ch->direction() = hdr_cmn::DOWN;
	ch->iface() = -2;
	ch->error() = 0;
	ch->addr_type() = NS_AF_INET;
	ch->next_hop() = IP_BROADCAST;
	ch->prev_hop_ = vehicle_ip();          //


	ih->saddr() = vehicle_ip();
	ih->daddr() = sih->saddr();
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl_ = NETWORK_DIAMETER;

	tavrh->seqno_ = 101;

	int slen = strlen(Address::instance().print_nodeaddr(vehicle_ip()));
	char currentIP[slen+1];

	currentIP[slen] = '\0';
	strcpy(currentIP,Address::instance().print_nodeaddr(vehicle_ip()));

	if(currentIP[0] == '1'){
		fprintf(stdout,"\n\tTAVRagent, %d replyHello sip=%s did=%s posx=%0.2f, posy=%0.2f",
				conf_node_id,
				Address::instance().print_nodeaddr(vehicle_ip()),
				Address::instance().print_nodeaddr(ih->daddr()),
				vehicle_position_x(),
				vehicle_position_y());

	}




	Scheduler::instance().schedule(target_, p, JITTER);

}


void
TAVRagent::reset_tavr_timer() {
	tavr_timer_.resched((double)TAVR_INTERVAL);
}

void
TAVRagent::update_vehicular_info(){
	node_->update_position();

	vehicle_position_x() = node_->X();
	vehicle_position_y() = node_->Y();
	vehicle_speed() = node_->speed();

	int nodeid = get_ipID(vehicle_ip());

	vehicle_position_x_LIST_[nodeid] =vehicle_position_x();
	vehicle_position_y_LIST_[nodeid] =vehicle_position_y();
	vehicle_speed_LIST_[nodeid] =vehicle_speed();
	junction_LIST_[nodeid] = true;
	bs_ip_LIST_[nodeid] = (nsaddr_t)0;

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
	fprintf(stdout,"current mini step for locate junctions info is x=%0.2f, y=%0.2f",mini_Xstep,mini_Ystep);
	for(; r < conf_scenario_rowc - 1; r++){
		if((vehicle_position_y_LIST_[nodeid] > junc_info_arrayY[r][0] - mini_Ystep)
		&& (vehicle_position_y_LIST_[nodeid] < junc_info_arrayY[r+1][0] + mini_Ystep))
			break;
	}

	for(; c < conf_scenario_rowc - 1; c++){
		if((vehicle_position_x_LIST_[nodeid] > junc_info_arrayX[0][c] - mini_Xstep)
		&& (vehicle_position_x_LIST_[nodeid] < junc_info_arrayX[0][c+1] + mini_Xstep))
			break;
	}

	if(abs(vehicle_position_y_LIST_[nodeid] - junc_info_arrayY[r][0]) < mini_Ystep){

	}else if(abs(vehicle_position_y_LIST_[nodeid] - junc_info_arrayY[r+1][0]) < mini_Ystep){

	}else if(abs(vehicle_position_x_LIST_[nodeid] - junc_info_arrayX[0][c]) < mini_Xstep){

	}else if(abs(vehicle_position_x_LIST_[nodeid] - junc_info_arrayX[0][c+1]) < mini_Xstep){

	}else{
		fprintf(stderr,"update_vehicular_info locate the street of mobile ERROR ");
		exit(1);
	}
}
void
TAVRagent::recv(Packet *p, Handler *) {
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);


	if(get_domain(vehicle_ip()) == 1)
		fprintf(stdout,"\n\nTAVRagent, recv msg  sid=%s did=%s cid=%s",
					Address::instance().print_nodeaddr(ih->saddr()),
					Address::instance().print_nodeaddr(vehicle_ip()),
					Address::instance().print_nodeaddr(ih->daddr()));
//	if(strcasecmp(cip,"2.0.1") == 0)fprintf(stdout,"\n\nTAVRagent, recv msg  sid=%s did=%s cid=%s",sip,dip,cip);
	if(ch->ptype() == PT_SIMUTAVR)
		fprintf(stdout,"\n\nTAVRagent, recv route manager  sid=%s did=%s cid=%s",
					Address::instance().print_nodeaddr(ih->saddr()),
					Address::instance().print_nodeaddr(vehicle_ip()),
					Address::instance().print_nodeaddr(ih->daddr()));
	else if(ch->ptype() == PT_TAVRHELLO)
		fprintf(stdout,"\n\nTAVRagent, recv veh_hello  sid=%s did=%s cid=%s",
					Address::instance().print_nodeaddr(ih->saddr()),
					Address::instance().print_nodeaddr(vehicle_ip()),
					Address::instance().print_nodeaddr(ih->daddr()));
	else if(ch->ptype() == PT_TAVRWIRED)
		fprintf(stdout,"\n\nTAVRagent, recv wired_infos  sid=%s did=%s cid=%s",
					Address::instance().print_nodeaddr(ih->saddr()),
					Address::instance().print_nodeaddr(vehicle_ip()),
					Address::instance().print_nodeaddr(ih->daddr()));


/*

	if(ih->saddr() == vehicle_ip()) {
		//if there exists a loop, must drop the packet

		if (ch->num_forwards() > 0) {
//			fprintf(stdout,"\n\tTAVRagent, recv msgType=%s but it will be drop s=%d d=%d c=%d",tmpStr,ih->saddr(),ih->daddr(),vehicle_ip());
			drop(p, DROP_RTR_ROUTE_LOOP);
			return;
		}
		//else if this is a packet I am originating, must add IP header
		else if (ch->num_forwards() == 0) {
			ch->size() += IP_HDR_LEN;
		}
	}

*/


	//if it is a BARV packet, must process it
	//otherwise, must forward the packet (unless TTL has reached zero)
	else {

/*
		if(get_domain(vehicle_ip()) == 2){//vehicular
			if (ch->ptype() == PT_SIMUTAVR) {
				recv_tavr_pkt(p);
			}else if (ch->ptype() == PT_TAVRHELLO) {
				recv_Hello(p);
//				dmux_->recv(p,0);
			}else if (ch->ptype() == PT_TAVRWIRED) {
				recv_Wired(p);
//				dmux_->recv(p,0);
			}
		}else if(get_domain(vehicle_ip()) == 1){//base station
			//exchange the msg through broadcast
			if (ch->ptype() == PT_SIMUTAVR) {
				Packet::free(p);
			}
		}
*/

		ih->ttl_--;
		if (ih->ttl_ == 0) {
			drop(p, DROP_RTR_TTL);
			return;
		}
		forward_data(p);
	}

//	Packet::free(p);
}


void
TAVRagent::recv_tavr_pkt(Packet *p) {
	struct hdr_route_tavr	*helloh = hdr_route_tavr::access(p);
	struct hdr_ip		*ih = HDR_IP(p);

	helloh->seqno_ = 101;

	int nodeid = Address::instance().get_lastaddr(ih->saddr());
	fprintf(stdout, "update node index=%d ip=%s",nodeid,Address::instance().print_nodeaddr(ih->saddr()));
	reply_tavr_pkt(p);
}

void
TAVRagent::recv_Wired(Packet *p) {
//	fprintf(stdout,"\n\t** -----  ** TAVRagent, recv_Wired\n cip=%s", Address::instance().print_nodeaddr(vehicle_ip()));
	struct hdr_wired_infos	*wiredh = hdr_wired_infos::access(p);
//	struct hdr_cmn 			*ch = HDR_CMN(p);
//	struct hdr_ip			*ih = HDR_IP(p);


	//recv
	int veh_amount = VEHICULAR_AMOUNT;
	for(int i=0; i < veh_amount; i++){

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
	//forward

}

void
TAVRagent::recv_Hello(Packet *p) {
//	fprintf(stdout,"\n\t** -----  ** TAVRagent, recv_Wired\n cip=%s", Address::instance().print_nodeaddr(vehicle_ip()));
	struct hdr_veh_hello	*hello = hdr_veh_hello::access(p);
//	struct hdr_cmn 			*ch = HDR_CMN(p);
//	struct hdr_ip			*ih = HDR_IP(p);


	//recv
	int veh_amount = VEHICULAR_AMOUNT;
	for(int i=0; i < veh_amount; i++){

		if((hello->hello_tstamp()< vehicle_info_updateTime_list()[i]) || (hello->vehicle_id() < 0)) continue;//if no info received , do NOT update the previous infos
		char ipStr[15] = {'\0'};
		sprintf(ipStr,"2.0.%d",hello->vehicle_id());

		fprintf(stdout," -recv_Hello- ipStr=%s -*- ",ipStr);

		vehicle_ip_LIST()[i] = Address::instance().str2addr(ipStr);
		vehicle_position_x_LIST()[i] = hello->vehicle_position_x();
		vehicle_position_y_LIST()[i] = hello->vehicle_position_y();
		vehicle_speed_LIST()[i]      = hello->vehicle_speed();
		vehicle_direction_LIST()[i]  = hello->vehicle_direction();
		bs_ip_LIST()[i]              = hello->bs_id();
		junc_row_prev_List()[i] = hello->junc_row_prev();
		junc_col_prev_List()[i] = hello->junc_col_prev();
		junc_row_next_List()[i] = hello->junc_row_next();
		junc_col_next_List()[i] = hello->junc_col_next();
		vehicle_info_updateTime_list()[i] = hello->hello_tstamp();
	}
	//forward

}

void TAVRagent::forward_data(Packet *p) {

	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);

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



int
TAVRagent::get_domain(nsaddr_t cip){
	int* pNodeShift = Address::instance().NodeShift_;
	int* pNodeMask = Address::instance().NodeMask_;
	int  IPdomain = 0;

 	IPdomain = cip >> pNodeShift[1];
 	IPdomain = IPdomain & pNodeMask[1];

	delete [] pNodeShift;
	delete [] pNodeMask;
 	return IPdomain;
}

int
TAVRagent::get_ipID(nsaddr_t cip){
	int levelsTest = Address::instance().levels_;
	int* pNodeShift = Address::instance().NodeShift_;
	int* pNodeMask = Address::instance().NodeMask_;
	int  IPid = 0;

	IPid = cip >> pNodeShift[levelsTest];
	IPid = IPid & pNodeMask[levelsTest];

	delete [] pNodeShift;
	delete [] pNodeMask;
 	return IPid;
}

int
TAVRagent::get_cluster(nsaddr_t cip){
	int* pNodeShift = Address::instance().NodeShift_;
	int* pNodeMask = Address::instance().NodeMask_;
	int  IPcluster = 0;

	IPcluster = cip >> pNodeShift[2];
	IPcluster = IPcluster & pNodeMask[2];

	delete [] pNodeShift;
	delete [] pNodeMask;
 	return IPcluster;
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
		for(int c=0; c < conf_scenario_rowc; c++){
			junc_info_arrayX[r][c] = conf_startx_ + c*Xstep;
			junc_info_arrayY[r][c] = conf_starty_ + r*Ystep;
		}
	}

//	printf("Rabbit, %u am here. The command has finished", vehicle_ip());
}


