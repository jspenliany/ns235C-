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

//======define part===================
char GstrIP[32];
int  GnodeId[10];


int hdr_tavr::offset_;
static class SIMUTAVRHeaderClass : public PacketHeaderClass {
public:
	SIMUTAVRHeaderClass() : PacketHeaderClass("PacketHeader/SIMUTAVR",
                                              sizeof(hdr_all_tavr)) {
	  bind_offset(&hdr_tavr::offset_);
	}
} class_tavr_hdr;

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
//	agent_->send_BARV_pkt();
//	agent_->reset_BARV_pkt_timer();
}

void
TAVRHello_pktTimer::expire(Event* e) {
	agent_->sendHello();
	agent_->reset_Hello_timer();
}

void
TAVRreplyHello_pktTimer::expire(Event* e) {
	agent_->replyHello();
	agent_->reset_replyHello_timer();
}

TAVRagent::TAVRagent(nsaddr_t ip) : Agent(PT_SIMUTAVR), hello_timer_(this), reply_hello_timer_(this){
	vehicle_ip_ = ip;
	vehicle_speed_ = -10;
	hello_amount_ = 0;
	firstHellorecv_ts_ = -100.2;

	node_ = (MobileNode*)Node::get_node_by_address(ip);

	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		vehicle_speed_LIST_[i] = -10;
		vehicle_ip_LIST_[i] = (nsaddr_t)0;
		vehicle_position_x_LIST_[i] = -100.0;
		vehicle_position_y_LIST_[i] = -100.0;
	}

	conf_test_AXIS_ip = -100;

}

void
TAVRagent::sendHello() {

	Packet *p = Packet::alloc();
	struct hdr_cmn* 		ch = HDR_CMN(p);
	struct hdr_ip*			ih = HDR_IP(p);
	struct hdr_veh_hello*	tavrh = HDR_VEH_HELLO(p);

	node_->update_position();
	vehicle_position_x() = node_->X();
	vehicle_position_y() = node_->Y();
	vehicle_speed() = node_->speed();

	//add self info into list
	tavrh->rvh_type_ = TAVRTYPE_HELLO;
	tavrh->vehicle_ip() 			= vehicle_ip();
	tavrh->vehicle_position_x() 	= vehicle_position_x();
	tavrh->vehicle_position_y() 	= vehicle_position_y();
	tavrh->vehicle_speed()			= vehicle_speed();
//	tavrh->vehicle_speed()			= 3.0 + vehicle_ip() * vehicle_ip();

	ch->ptype() = PT_SIMUTAVR;
	ch->size() = IP_HDR_LEN + tavrh->size();
	ch->direction() = hdr_cmn::DOWN;
	ch->iface() = -2;
	ch->error() = 0;
	ch->addr_type() = NS_AF_INET;
	ch->prev_hop_ = vehicle_ip();          //
/*

	switch(conf_test_AXIS_ip){
	case 15:
		ch->next_hop() = Address::instance().str2addr("1.0.5");break;
	case 12:
		ch->next_hop() = Address::instance().str2addr("1.0.2");break;
	case 25:
		ch->next_hop() = Address::instance().str2addr("2.0.5");break;
	case 22:
		ch->next_hop() = Address::instance().str2addr("2.0.2");break;
	default:
		ch->next_hop() = Address::instance().str2addr("0.0.2");break;
	}
*/


	ih->saddr() = vehicle_ip();
	ih->daddr() = IP_BROADCAST;
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl_ = 1;


	Scheduler::instance().schedule(target_, p, 0.0);


}

void
TAVRagent::replyHello() {
	Packet 					*p = allocpkt();
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);
	struct hdr_wired_info	*tavrh = HDR_WIRED_INFO(p);

	update_vehicular_info();

//	ih->daddr() = IP_BROADCAST;


	ch->ptype() = PT_SIMUTAVR;
	ch->size() = IP_HDR_LEN + tavrh->size();
	ch->direction() = hdr_cmn::DOWN;
	ch->iface() = -2;
	ch->error() = 0;
	ch->addr_type() = NS_AF_NONE;
	ch->next_hop() = Address::instance().str2addr("0.0.2");
	ch->prev_hop_ = vehicle_ip();          //


	ih->saddr() = vehicle_ip();
	ih->daddr() =  Address::instance().str2addr("0.0.2");
	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl_ = 2;


	tavrh->rwi_type_ = TAVRTYPE_WIRED;

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


		int i = 0;
		for(; i < VEHICULAR_AMOUNT ;i++){

			tavrh->vehicle_ip_LIST()[i] = vehicle_ip_LIST_[i];
			tavrh->vehicle_position_x_LIST()[i] = vehicle_position_x_LIST_[i];
			tavrh->vehicle_position_y_LIST()[i] = vehicle_position_y_LIST_[i];
			tavrh->vehicle_speed_LIST()[i] = vehicle_speed_LIST_[i];
			tavrh->vehicle_direction_LIST()[i] = vehicle_direction_LIST_[i];

			if(tavrh->vehicle_speed_LIST()[i] > -1){
						fprintf(stdout,"\n\t---- %d replyHello sip=%s did=%s posx=%0.2f, posy=%0.2f",
								i,
								Address::instance().print_nodeaddr(vehicle_ip()),
								Address::instance().print_nodeaddr(tavrh->vehicle_ip_LIST()[i]),
								tavrh->vehicle_position_x_LIST()[i],
								tavrh->vehicle_position_y_LIST()[i]);
						ch->next_hop() = tavrh->vehicle_ip_LIST()[i];
						ih->daddr() =  tavrh->vehicle_ip_LIST()[i];
					}
	//		fprintf(stdout,"\n\tTAVRagent, %d replyHello ptType=%u sip=%d did=%d posx=%0.2f, posy=%0.2f",i,ch->ptype(),ih->saddr(), ih->daddr(),tavrh->vehicle_position_x_LIST()[i],tavrh->vehicle_position_y_LIST()[i]);
		}

	}




	Scheduler::instance().schedule(target_, p, 0.0);

	hello_amount_ = 0;
	firstHellorecv_ts_ = -100.2;

}

void
TAVRagent::reset_Hello_timer() {
	hello_timer_.resched((double)HELLO_INTERVAL);
}

void
TAVRagent::reset_replyHello_timer() {
	reply_hello_timer_.resched((double)HELLO_INTERVAL);
}

double *TAVRagent::cvehicle_position_x_LIST() {
	double *tmp_list = new double[VEHICULAR_AMOUNT];
	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		tmp_list[i] = vehicle_position_x_LIST_[i];
	}
	return tmp_list;
}
double *TAVRagent::cvehicle_position_y_LIST() {
	double *tmp_list = new double[VEHICULAR_AMOUNT];
	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		tmp_list[i] = vehicle_position_y_LIST_[i];
	}
	return tmp_list;
}
double* TAVRagent::cvehicle_speed_LIST() {
	double *tmp_list = new double[VEHICULAR_AMOUNT];
	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		tmp_list[i] = vehicle_speed_LIST_[i];
	}
	return tmp_list;
}
u_int8_t *TAVRagent::cvehicle_direction_LIST() {
	u_int8_t *tmp_list = new u_int8_t[VEHICULAR_AMOUNT];
	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		tmp_list[i] = vehicle_direction_LIST_[i];
	}
	return tmp_list;
}
bool *TAVRagent::cjunction_LIST() {
	bool *tmp_list = new bool[VEHICULAR_AMOUNT];
	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		tmp_list[i] = junction_LIST_[i];
	}
	return tmp_list;
}
nsaddr_t *TAVRagent::cbs_ip_LIST() {
	nsaddr_t *tmp_list = new nsaddr_t[VEHICULAR_AMOUNT];
	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		tmp_list[i] = bs_ip_LIST_[i];
	}
	return tmp_list;
}
void
TAVRagent::update_vehicular_info(){
	node_->update_position();

	vehicle_position_x() = node_->X();
	vehicle_position_y() = node_->Y();
	vehicle_speed() = node_->speed();

	int nodeid = get_indexFromaddr(vehicle_ip());

	vehicle_position_x_LIST_[nodeid] =vehicle_position_x();
	vehicle_position_y_LIST_[nodeid] =vehicle_position_y();
	vehicle_speed_LIST_[nodeid] =vehicle_speed();
	junction_LIST_[nodeid] = true;
	bs_ip_LIST_[nodeid] = (nsaddr_t)0;
}
void
TAVRagent::recv(Packet *p, Handler *) {
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);
//	struct hdr_veh_hello	*tavrh = HDR_VEH_HELLO(p);

	int   slen = 0;
	int   clen = 0;
	int   dlen = 0;
	slen = strlen(Address::instance().print_nodeaddr(ih->saddr()));
	clen = strlen(Address::instance().print_nodeaddr(vehicle_ip()));
	dlen = strlen(Address::instance().print_nodeaddr(ih->daddr()));
	char sip[slen+1];
	char cip[clen+1];
	char dip[dlen+1];
	sip[slen]='\0';
	cip[clen]='\0';
	dip[dlen]='\0';
	strcpy(sip,Address::instance().print_nodeaddr(ih->saddr()));
	strcpy(cip,Address::instance().print_nodeaddr(vehicle_ip()));
	strcpy(dip,Address::instance().print_nodeaddr(ih->daddr()));


	if(ch->ptype() == PT_SIMUTAVR)fprintf(stdout,"\n\tTAVRagent, recv msgType=%u  sid=%d did=%d cid=%d",ch->ptype(),sip,dip,cip);



	if(ih->saddr() == vehicle_ip()) {
		//if there exists a loop, must drop the packet
		const char* tmpStr = (ch->ptype() == PT_SIMUTAVR) ? "tavr headers":"header info NOT known";

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



	//if it is a BARV packet, must process it
	if (ch->ptype() == PT_SIMUTAVR) {
		recv_TAVR(p);
	}
	//otherwise, must forward the packet (unless TTL has reached zero)
	else {
		ih->ttl_--;
		if (ih->ttl_ == 0) {
			drop(p, DROP_RTR_TTL);
			return;
		}
		forward_data(p);
	}
}

void TAVRagent::recv_TAVR(Packet* p) {
	struct hdr_cmn 		*ch = HDR_CMN(p);
	struct hdr_ip		*ih = HDR_IP(p);
	struct hdr_tavr 	*tavr = HDR_TAVR(p);

	int   slen = 0;
	int   clen = 0;
	int   dlen = 0;

	slen = strlen(Address::instance().print_nodeaddr(ih->saddr()));
	clen = strlen(Address::instance().print_nodeaddr(vehicle_ip()));
	dlen = strlen(Address::instance().print_nodeaddr(ih->daddr()));
	char sip[slen+1];
	char cip[clen+1];
	char dip[dlen+1];
	sip[slen]='\0';
	cip[clen]='\0';
	dip[dlen]='\0';
	strcpy(sip,Address::instance().print_nodeaddr(ih->saddr()));
	strcpy(cip,Address::instance().print_nodeaddr(vehicle_ip()));
	strcpy(dip,Address::instance().print_nodeaddr(ih->daddr()));

	fprintf(stdout, "\n---- ** --- pttype=%u s=%s d=%s c=%s", tavr->tavr_type(),sip,dip,cip);
	switch(tavr->tavr_type_){
	case TAVRTYPE_HELLO:
		if(ih->daddr() == vehicle_ip()){
			recv_Hello(p);
		}
		fprintf(stdout,"\nsip=%s, cip=%s, dip=%s TAVRagent-----recv_hello----",sip,cip,dip);
		break;
	case TAVRTYPE_WIRED:
		recv_Wired(p);
		fprintf(stdout,"\nsip=%s, cip=%s, dip=%s TAVRagent-----recv_wired----",sip,cip,dip);
		break;
	case TAVRTYPE_DATA:
		fprintf(stdout,"\nsip=%s, cip=%s, dip=%s TAVRagent-----recv_data----",sip,cip,dip);
		break;
	case TAVRTYPE_TAVR:
		fprintf(stdout,"\nsip=%s, cip=%s, dip=%s TAVRagent-----recv_tavr----",sip,cip,dip);
		break;
	}

}


void
TAVRagent::recv_Hello(Packet *p) {
	if(firstHellorecv_ts_ < 1.0)firstHellorecv_ts_ = NOW;
	hello_amount_++;
	struct hdr_veh_hello	*helloh = HDR_VEH_HELLO(p);
/*
    if(){
    	fprintf(stdout, "\n--+ + + + + + ----recv_Hello speed=%0.2f posx=%0.2f posy=%0.2f",helloh->vehicle_ip(),helloh->vehicle_speed(),helloh->vehicle_position_x(),helloh->vehicle_position_y());
    }
*/

	int nodeid = get_indexFromaddr(helloh->vehicle_ip());
	fprintf(stdout, "update node index=%d ip=%s",nodeid,Address::instance().print_nodeaddr(helloh->vehicle_ip()));
	vehicle_ip_LIST_[nodeid] = helloh->vehicle_ip();
	vehicle_position_x_LIST_[nodeid] = helloh->vehicle_position_x();
	vehicle_position_y_LIST_[nodeid] = helloh->vehicle_position_y();
	vehicle_speed_LIST_[nodeid] = helloh->vehicle_speed();
	junction_LIST_[nodeid] = true;
	bs_ip_LIST_[nodeid] = (nsaddr_t)0;

	int tmp_recv_count = VEHICULAR_INFO_PTB_RECV_COUNTER;
	if(hello_amount_ >= tmp_recv_count)replyHello();
}

void
TAVRagent::recv_Wired(Packet *p) {
//	fprintf(stdout,"\n\t** -----  ** TAVRagent, recv_Wired\n cip=%d", vehicle_ip());
	struct hdr_wired_info	*wiredh = HDR_WIRED_INFO(p);
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);

	int i = 0;
	for(; i < VEHICULAR_AMOUNT;i++){

//		fprintf(stdout,"\n\t--* * * *---type=%u-recv_Wired from %d to %d speed=%0.2f X=%0.2f Y=%0.2f",ch->ptype(),ih->saddr(), ih->daddr(),wiredh->vehicle_speed_LIST()[i], wiredh->vehicle_position_x_LIST()[i],wiredh->vehicle_position_y_LIST()[i]);
	}


//	fprintf(stdout, "\n--* * * *----recv_Wired from %d speed=%0.2f posx=%0.2f posy=%0.2f",ih->saddr(),wiredh->vehicle_ip(),wiredh->vehicle_speed(),wiredh->vehicle_position_x(),helloh->vehicle_position_y());
}

void TAVRagent::forward_data(Packet *p) {

	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);

	if(ih->daddr() == vehicle_ip()){
		if(ch->direction() == hdr_cmn::UP){// packets for applications
			port_dmux_->recv(p,0);
		}
	}else{
		Scheduler::instance().schedule(target_, p, 0.);
	}

}


int TAVRagent::command(int argc, const char*const* argv) {
	if (argc == 2) {
		if (strcasecmp(argv[1], "start") == 0) {
			hello_timer_.resched(0.0);
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
		}else if (strcasecmp (argv[1], "rabbitAll") == 0) {
			recv_Rabbitall();
			return TCL_OK;
		}else if (strcasecmp (argv[1], "printbs") == 0) {
			print_bs();
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
			port_dmux_ = (PortClassifier*)TclObject::lookup(argv[2]);
			if (port_dmux_ == 0) {
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
		else if (strcasecmp (argv[1], "conf-mapx") == 0) {
			map_X = atof(argv[2]);
//				fprintf(stdout,"max_X=%0.2f",map_X);
			return TCL_OK;
		}
		else if (strcasecmp (argv[1], "conf-mapy") == 0) {
			map_Y = atof(argv[2]);
//				fprintf(stdout,"max_Y=%0.2f",map_Y);
			return TCL_OK;
		}else if (strcasecmp (argv[1], "vehi-num") == 0) {
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
		}else if (strcasecmp (argv[1], "AXIS_ip") == 0) {
			conf_test_AXIS_ip = atoi(argv[2]);
			return TCL_OK;
		}

/*

		else if (strcasecmp (argv[1], "confmapx") == 0) {
			conf_scenario_Width_ = atof(argv[2]);
//			printf(".....I'm six...end.w=%0.2f, h=%0.2f, r=%d, c=%d",conf_scenario_Width_,conf_scenario_Height_,conf_scenario_rowc,conf_scenario_colc);
		}else if (strcasecmp (argv[1], "confmapy") == 0) {
			conf_scenario_Height_ = atof(argv[2]);
//			printf(".....I'm six...end.w=%0.2f, h=%0.2f, r=%d, c=%d",conf_scenario_Width_,conf_scenario_Height_,conf_scenario_rowc,conf_scenario_colc);
		}else if (strcasecmp (argv[1], "confmapr") == 0) {
			conf_scenario_rowc = atoi(argv[2]);
//			printf(".....I'm six...end.w=%0.2f, h=%0.2f, r=%d, c=%d",conf_scenario_Width_,conf_scenario_Height_,conf_scenario_rowc,conf_scenario_colc);
		}else if (strcasecmp (argv[1], "confmapc") == 0) {
			conf_scenario_colc = atoi(argv[2]);
			printf(".....I'm three...end.w=%0.2f, h=%0.2f, r=%d, c=%d",conf_scenario_Width_,conf_scenario_Height_,conf_scenario_rowc,conf_scenario_colc);
		}
*/

	}
	else if (argc == 4) {
		if (strcasecmp (argv[1], "conf-junc") == 0) {
			if (strcasecmp (argv[2], "index") == 0) {
				int index = atoi(argv[3]);
				map_junc_list[index].junc_id_ = index;
				return TCL_OK;
			}
		}
	}
	else if (argc == 5) {
		if (strcasecmp (argv[1], "conf-junc") == 0) {
			if (strcasecmp (argv[2], "juncx") == 0) {
				int index = atoi(argv[3]);
				map_junc_list[index].junc_x_ = atof(argv[4]);
				return TCL_OK;
			}
			else if (strcasecmp (argv[2], "juncy") == 0) {
				int index = atoi(argv[3]);
				map_junc_list[index].junc_y_ = atof(argv[4]);
				return TCL_OK;
			}
			else if (strcasecmp (argv[2], "juncz") == 0) {
				int index = atoi(argv[3]);
				map_junc_list[index].junc_z_ = atof(argv[4]);
				return TCL_OK;
			}

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
		}
	}
	//Pass the command to the base class
	return Agent::command(argc,argv);
}

void
TAVRagent::print_addr(nsaddr_t addr){
	int slen = sizeof(GstrIP)/sizeof(GstrIP[0]);
	for(int i = 0; i < slen; i++){
		GstrIP[i] = '\0';
	}
	slen = strlen(Address::instance().print_nodeaddr(addr));

	strcpy(GstrIP,Address::instance().print_nodeaddr(addr));
}

int
TAVRagent::get_indexFromaddr(nsaddr_t addr){
	print_addr(addr);
	int ipSD[2]={-1,-1};
	int ipDOTflag = 0;
	int slen = sizeof(GstrIP)/sizeof(GstrIP[0]);
	for(int i = 0; i < slen; i++){
//		fprintf(stdout,"%d %c",i,GstrIP[i]);
		if(GstrIP[i] == '.'){
			if(ipDOTflag == 1){
				ipSD[0] = i + 1;

				ipDOTflag++;
			}else if(ipDOTflag == 0){
				ipDOTflag++;
			}
		}else if(ipSD[0] > -1 && GstrIP[i] == '\0'){
			ipSD[1] = i - 1;

			i = slen + 10;
		}
	}

	char IPdata[ipSD[1] - ipSD[0] + 2];
	IPdata[ipSD[1] - ipSD[0] + 1] = '\0';
	for(int i = ipSD[0]; i < ipSD[1] + 1; i++){
		IPdata[i - ipSD[0]] = GstrIP[i];
	}
	return atoi(IPdata);
}
//=====================test part======================

void
TAVRagent::recv_Rabbitall(){
//	printf("Rabbit, %u am here. The command has finished", vehicle_ip());
}

void
TAVRagent::print_bs(){
	print_addr(vehicle_ip());
//	if(conf_debug != 0)printf("\n---** --- Cip=%u, BSip=%u is-a=%d",vehicle_ip(),node_->base_stn(),(vehicle_ip() == node_->base_stn()));
//  printf("\n---** --- Cip=%s, BSip=%s is-a=%d",GstrIP,Address::instance().print_nodeaddr(node_->base_stn()),(vehicle_ip() == node_->base_stn()));
}

