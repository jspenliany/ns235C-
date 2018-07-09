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
#include <tcl.h>
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

//========================================
#define NEW_TIMER_VEHI		2
#define	NEW_TIMER_BASE		1


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

void
TAVR_proTimer::expire(Event* e) {
	if(Address::instance().get_firstaddr(agent_->vehicle_ip()) == 1)return;
	agent_->report_loc();
	agent_->reset_tavr_protimer();
}

void
TAVR_vehiGinfoBcastTimer::expire(Event* e) {
	if(Address::instance().get_firstaddr(agent_->vehicle_ip()) == NEW_TIMER_BASE)return;
	agent_->new_update_globalInfo();
	agent_->new_reset_vehi_Global_infoTimer();
}

void
TAVR_vehiLinfoBcastTimer::expire(Event* e) {
	if(Address::instance().get_firstaddr(agent_->vehicle_ip()) == NEW_TIMER_BASE)return;
	agent_->new_update_localInfo();
	agent_->new_reset_vehi_Local_infoTimer();
}

void
TAVR_baseLinfoBcastTimer::expire(Event* e) {
	if(Address::instance().get_firstaddr(agent_->vehicle_ip()) == NEW_TIMER_VEHI)return;
	agent_->new_update_localInfo();
	agent_->new_reset_base_Local_infoTimer();
}

TAVRfile TAVRagent::file_op;
bool TAVRagent::file_exist = false;
double TAVRagent::Gscen_width = 0.0;
double TAVRagent::Gscen_length = 0.0;
int TAVRagent::Gscen_rowC = 0;
int TAVRagent::Gscen_colC = 0;
double TAVRagent::Glane_width = 0.0;
double TAVRagent::Gjunc_radius = 0.0;
int TAVRagent::Gcurrent_vnum = 0;

double	TAVRagent::GUP_length[NEWG_ROW_MAX][NEWG_COL_MAX];
double	TAVRagent::GRIGHT_length[NEWG_ROW_MAX][NEWG_COL_MAX]={};
double	TAVRagent::GUP_angle[NEWG_ROW_MAX][NEWG_COL_MAX]={};
double	TAVRagent::GRIGHT_angle[NEWG_ROW_MAX][NEWG_COL_MAX]={};
double	TAVRagent::GXbas_LOC[NEWG_ROW_MAX][NEWG_COL_MAX]={};
double	TAVRagent::GYbas_LOC[NEWG_ROW_MAX][NEWG_COL_MAX]={};

int32_t	TAVRagent::G_id_uT[NEWG_VEHI_NUM]={0};
int32_t	TAVRagent::G_speed_direction[NEWG_VEHI_NUM]={0};
int32_t	TAVRagent::G_locX[NEWG_VEHI_NUM]={0};
int32_t	TAVRagent::G_locY[NEWG_VEHI_NUM]={0};
int16_t	TAVRagent::G_loc_BID_previous[NEWG_VEHI_NUM]={0};
int16_t	TAVRagent::G_loc_BID_toward[NEWG_VEHI_NUM]={0};

int32_t	TAVRagent::LG_id_uT[NEWG_VEHI_NUM]={0};
int32_t	TAVRagent::LG_speed_direction[NEWG_VEHI_NUM]={0};
int32_t	TAVRagent::LG_locX[NEWG_VEHI_NUM]={0};
int32_t	TAVRagent::LG_locY[NEWG_VEHI_NUM]={0};
int16_t	TAVRagent::LG_loc_BID_previous[NEWG_VEHI_NUM]={0};
int16_t	TAVRagent::LG_loc_BID_toward[NEWG_VEHI_NUM]={0};

//string	TAVRagent::G_shortPATHpairs_matrix[NEWG_BASE_NUM][NEWG_BASE_NUM]={{"WHAT GOD DO"}};

int		TAVRagent::G_UP_len_indexR=0;
int		TAVRagent::G_RIGHT_len_indexR=0;
int		TAVRagent::G_UP_angle_indexR=0;
int		TAVRagent::G_RIGHT_angle_indexR=0;
int		TAVRagent::G_Xbas_LOC_indexR=0;
int		TAVRagent::G_Ybas_LOC_indexR=0;

int		TAVRagent::G_UP_len_indexC=0;
int		TAVRagent::G_RIGHT_len_indexC=0;
int		TAVRagent::G_UP_angle_indexC=0;
int		TAVRagent::G_RIGHT_angle_indexC=0;
int		TAVRagent::G_Xbas_LOC_indexC=0;
int		TAVRagent::G_Ybas_LOC_indexC=0;
char	TAVRagent::G_strTMP[700]={'C'};
//int		TAVRagent::G_SpPm_index=0;

TAVRagent::TAVRagent(nsaddr_t ip) : Agent(PT_SIMUTAVR), tavr_pkttimer_(this), tavr_protimer_(this),
		S_vehiGinfoBcastTimer(this),S_vehiLinfoBcastTimer(this),S_baseLinfoBcastTimer(this){
	vehicle_ip_ = ip;
	vehicle_speed_ = -10;
	hello_amount_ = 0;
	firstHellorecv_ts_ = -100.2;
	inner_seqno_ = 0;

	node_ = (MobileNode*)Node::get_node_by_address(ip);

	int i = 0;
	for (; i < VEHICULAR_AMOUNT; i++) {
		vehicle_speed_LIST_[i] = -100;
		vehicle_ip_LIST_[i] = (nsaddr_t)0;
		vehicle_position_x_LIST_[i] = -100;
		vehicle_position_y_LIST_[i] = -100;
		vehicle_info_updateTime_LIST_[i] = -1.0;
		recv_seqno_List_[i] = 120;
		info_update_list[i] = false;
		junc_row_prev_list[i] = -1;
		junc_row_next_list[i] = -1;
		junc_col_prev_list[i] = -1;
		junc_col_next_list[i] = -1;
		junction_LIST_[i] = false;
	}
	wire_info_recv = false;
	conf_test_AXIS_ip = -100;
	conf_junc_row_bs = -100;
	conf_junc_col_bs = -100;
	detect_junc_interval = 5;
	detect_junc_flag = false;
	tavr_route_interval = 1.0;

	debug_flag = true;
	if(!TAVRagent::file_exist){
		TAVRagent::file_op.init_file("xxxx_tavr",103011);
		TAVRagent::file_op.rtr_write("------------this is routing layer CONTROL file................\n");
		TAVRagent::file_op.helmsg_write("------------this is routing layer HELLO file................\n");
		TAVRagent::file_op.wirmsg_write("------------this is routing layer WIRED file................\n");
		TAVRagent::file_exist = true;
	}
	location_Calculate_flag = true;
	debug_file = false;
	std_NETWORK_flag = false;
	std_NETWORK_index = 0;

	junc_center_index = -100;

	lane_Width = 3.0;
	junc_Radius = 3.0;



//-----------------------------------------

	S_vehicle_ip_ = ip;

	for (int new_i=0; new_i < NEWL_VEHI_NUM; new_i++) {
		L_idTOindex[new_i] = -1;
	}
	S_timer_vlocal_RUN_flag = false;
	S_timer_vglobal_RUN_flag = false;
	S_timer_blocal_RUN_flag = false;
	S_node_RUN_flag = false;
	S_checkFirstTime = true;
}

TAVRagent::~TAVRagent(){

	file_op.close();
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

	ch->size() = IP_HDR_LEN + sizeof(hdr_route_tavr);
	ch->prev_hop_ = vehicle_ip();          //
	ch->next_hop() = IP_BROADCAST;
	ch->addr_type() = NS_AF_INET;

	ih->saddr() = vehicle_ip();
	ih->daddr() = IP_BROADCAST;

	ih->sport() = RT_PORT;
	ih->dport() = RT_PORT;
	ih->ttl() = IP_DEF_TTL;

	tavrh->seqno_ = inner_seqno_++;

	Scheduler::instance().schedule(target_, p, TAVR_RTR_BASE_JITTER);
}

void
TAVRagent::reset_tavr_timer() {
	tavr_pkttimer_.resched(tavr_route_interval);
}

void
TAVRagent::update_vehicular_info(){
	char tmpStr[400];
	int nodeid = Address::instance().get_lastaddr(vehicle_ip());

	node_->update_position();
	double dst_x = node_->destX();
	double dst_y = node_->destY();
	double delta_x = node_->dX();
	double delta_y = node_->dY();

	vehicle_position_x() = node_->X();
	vehicle_position_y() = node_->Y();
	vehicle_speed() = node_->speed();
	info_update_list[nodeid] = true;
	vehicle_position_x_LIST_[nodeid] =(int16_t)floor(vehicle_position_x());
	vehicle_position_y_LIST_[nodeid] =(int16_t)floor(vehicle_position_y());
	vehicle_speed_LIST_[nodeid] =(int16_t)floor(vehicle_speed());
	recv_seqno_List_[nodeid] = 0;
	vehicle_info_updateTime_LIST_[nodeid] = Scheduler::instance().clock();


	if(vehicle_speed_LIST_[nodeid] > 300 || vehicle_speed_LIST_[nodeid] < 0){
		fprintf(stderr,"ERROR, TAVRagent::update_vehicular_info ....the speed is OVERFLOW----***%d  %hd..",vehicle_speed_LIST_[nodeid],vehicle_speed_LIST_[nodeid]);
		exit(1);
	}
	func_updateJUNCinfo(nodeid,vehicle_position_x(),vehicle_position_y(),vehicle_speed(),delta_x,delta_y);

	int16_t	move_UpDown = 0, move_LeftRight = 0;

	bool finish = false;
	switch(((int)round(delta_x))){
	case 1:
		move_LeftRight = DRIGHT;//------------right
		vehicle_speed_LIST_[nodeid] += MOVE_RIGHT;
		finish = true;
		break;
	case -1:
		move_LeftRight = DLEFT;//------------left
		vehicle_speed_LIST_[nodeid] += MOVE_LEFT;
		finish = true;
		break;
	case 0://---------------none
		finish = false;
		break;
	default:
		fprintf(stderr,"ERROR, TAVRagent::update_vehicular_info ....direction LEFT/RIGHT-----%d..",vehicle_speed_LIST_[nodeid]);
		exit(1);
	}
	if(!finish)switch(((int)round(delta_y))){
	case 1:
		move_UpDown = DUP;//-----------up
		vehicle_speed_LIST_[nodeid] += MOVE_UP;
		break;
	case -1:
		move_UpDown = DDOWN;//-----------down
		vehicle_speed_LIST_[nodeid] += MOVE_DOWN;
		break;
	case 0:
		break;
	default:
		fprintf(stderr,"ERROR, TAVRagent::update_vehicular_info ....direction UP/DOWN-----%d..",vehicle_speed_LIST_[nodeid]);
		exit(1);
	}


	if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		bool finish = false;
		switch(((int)round(delta_x))){
		case 1:
			sprintf(tmpStr,"\n**%d.----right------.update_vehicular_info.\tx=%0.4f\ty=%0.4f\tv=%0.4f\tv=%d\tdx=%0.4f\tdy=%0.4f\t_x=%0.4f\t_y=%0.4f \n",conf_node_id,
					node_->X(),node_->Y(),node_->speed(),vehicle_speed_LIST_[nodeid],dst_x,dst_y,delta_x,delta_y);
			finish = true;
			break;
		case -1:
			sprintf(tmpStr,"\n**%d.----left------.update_vehicular_info.\tx=%0.4f\ty=%0.4f\tv=%0.4f\tv=%d\tdx=%0.4f\tdy=%0.4f\t_x=%0.4f\t_y=%0.4f \n",conf_node_id,
					node_->X(),node_->Y(),node_->speed(),vehicle_speed_LIST_[nodeid],dst_x,dst_y,delta_x,delta_y);
			finish = true;
			break;
		case 0:
			sprintf(tmpStr,"\n**%d.----NONE L&R------.update_vehicular_info.\tx=%0.4f\ty=%0.4f\tv=%0.4f\tv=%d\tdx=%0.4f\tdy=%0.4f\t_x=%0.4f\t_y=%0.4f \n",conf_node_id,
					node_->X(),node_->Y(),node_->speed(),vehicle_speed_LIST_[nodeid],dst_x,dst_y,delta_x,delta_y);
			finish = false;
			break;
		default:
			fprintf(stderr,"ERROR, TAVRagent::update_vehicular_info ....direction LEFT/RIGHT-----%d..",vehicle_speed_LIST_[nodeid]);
			exit(1);
		}
		switch(((int)round(delta_y))){
		case 1:
			if(!finish)sprintf(tmpStr,"\n**%d.----up------.update_vehicular_info.\tx=%0.4f\ty=%0.4f\tv=%0.4f\tv=%d\tdx=%0.4f\tdy=%0.4f\t_x=%0.4f\t_y=%0.4f \n",conf_node_id,
					node_->X(),node_->Y(),node_->speed(),vehicle_speed_LIST_[nodeid],dst_x,dst_y,delta_x,delta_y);
			break;
		case -1:
			if(!finish)sprintf(tmpStr,"\n**%d.-----down-----.update_vehicular_info.\tx=%0.4f\ty=%0.4f\tv=%0.4f\tv=%d\tdx=%0.4f\tdy=%0.4f\t_x=%0.4f\t_y=%0.4f \n",conf_node_id,
					node_->X(),node_->Y(),node_->speed(),vehicle_speed_LIST_[nodeid],dst_x,dst_y,delta_x,delta_y);
			break;
		case 0:
			if(!finish)sprintf(tmpStr,"\n**%d.----NONE U&D------.update_vehicular_info.\tx=%0.4f\ty=%0.4f\tv=%0.4f\tv=%d\tdx=%0.4f\tdy=%0.4f\t_x=%0.4f\t_y=%0.4f \n",conf_node_id,
					node_->X(),node_->Y(),node_->speed(),vehicle_speed_LIST_[nodeid],dst_x,dst_y,delta_x,delta_y);
			break;
		default:
			fprintf(stderr,"ERROR, TAVRagent::update_vehicular_info ....direction UP/DOWN-----%d..",vehicle_speed_LIST_[nodeid]);
			exit(1);
		}
		TAVRagent::file_op.rtr_write(tmpStr);
	}


}

bool
TAVRagent::func_updateJUNCinfo(int vid, double vx, double vy, double vs, double deltaX, double deltaY){
	double tmpMAX = (conf_scenario_Width_ + conf_scenario_Height_)*(conf_scenario_Width_ + conf_scenario_Height_);
	int baseid = -1;
	if(junc_row_prev_list[vid] < 0){

		for(int ri=0; ri < conf_scenario_rowc; ri++){
			for(int ci=0; ci < conf_scenario_colc; ci++){
				if(tmpMAX > ((vx-Xbas_LOC[ri][ci])*(vx-Xbas_LOC[ri][ci]) + (vy-Ybas_LOC[ri][ci])*(vy-Ybas_LOC[ri][ci]))){
					tmpMAX = ((vx-Xbas_LOC[ri][ci])*(vx-Xbas_LOC[ri][ci]) + (vy-Ybas_LOC[ri][ci])*(vy-Ybas_LOC[ri][ci]));
					baseid = ri*conf_scenario_colc + ci;
				}
			}
		}
	}else if(junc_row_next_list[vid] < 0){
		baseid = junc_row_prev_list[vid]*conf_scenario_colc + junc_col_prev_list[vid];
	}else{
		baseid = junc_row_next_list[vid]*conf_scenario_colc + junc_col_next_list[vid];
	}

	return func_updateJUNCinfo(vid,baseid,vx,vy,vs,deltaX,deltaY);


}
/**
 * this function will update the junction info of current vehicle, call func_injunc, func_instreet func_adjoin
 * */
bool
TAVRagent::func_updateJUNCinfo(int vid, int bid, double vx, double vy, double vs, double deltaX, double deltaY){

	bool tmp_Output_flag = true;

	double delta_x = deltaX;
	double delta_y = deltaY;
	double laneWidth = lane_Width;
	double juncRadius = junc_Radius;

	int nodeid = vid;
	int baseid = bid;
	int juncROWcur = -1;
	int juncCOLcur = -1;

	double AccelerationMINI = 0.731;
	double AccelerationMAXI = 0.691;
	double cTIME = CURRENT_TIME;
	bool	left_right_flag = false;


	int dirt[4] = {0,0,0,0};

	int tmp_DIRT_flag[4];
	tmp_DIRT_flag[0] = 0;
	tmp_DIRT_flag[1] = 0;
	tmp_DIRT_flag[2] = 0;
	tmp_DIRT_flag[3] = 0;
	string flag[2];
	flag[0] = "freee";
	flag[1] = "ATTACK";
	if((baseid%conf_scenario_colc) > 0 && func_Instreet(baseid,baseid-1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
		dirt[0] = 1;
		if(fabs(delta_x) + 0.02 < fabs(delta_y))tmp_DIRT_flag[0] = 1;
	}
	if((baseid%conf_scenario_colc)+1 < conf_scenario_colc && func_Instreet(baseid,baseid+1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
		dirt[1] = 2;
		if(fabs(delta_x) + 0.02 < fabs(delta_y))tmp_DIRT_flag[1] = 1;
	}
	if((baseid/conf_scenario_colc) > 0 && func_Instreet(baseid,baseid-conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
		dirt[2] = 4;
		if(fabs(delta_y) + 0.02 < fabs(delta_x))tmp_DIRT_flag[2] = 1;
	}
	if((baseid/conf_scenario_colc)+1 < conf_scenario_rowc && func_Instreet(baseid,baseid+conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
		dirt[3] = 8;
		if(fabs(delta_y) + 0.02 < fabs(delta_x))tmp_DIRT_flag[3] = 1;
	}

	if(tmp_Output_flag)fprintf(stdout,"\n-----------------vehicle=%d---------baseid=%d-------delta x=%0.2f y=%0.2f--flag_list--%d%d%d%d---%s %s %s %s--ctime=%0.4f",
										nodeid,baseid,delta_x,delta_y,dirt[0],dirt[1],dirt[2],dirt[3],
										flag[tmp_DIRT_flag[0]].c_str(),flag[tmp_DIRT_flag[1]].c_str(),flag[tmp_DIRT_flag[2]].c_str(),flag[tmp_DIRT_flag[3]].c_str(),cTIME);


	if(junc_row_prev_list[nodeid] < 0){//first junction info decide
		if(func_Injunc(baseid,juncRadius,vehicle_position_x(),vehicle_position_y())){//inner circle
			junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
			junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
		}else{
			if((baseid%conf_scenario_colc) > 0 && func_Instreet(baseid,baseid-1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt previous 1 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;//with drive toward that direction
				}else if(delta_x > AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to right
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)-1;//coming in this junction

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_x < -AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to left
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);//drive away from this junction
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  previous 2 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}

				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt first junction info 01-----------------(01 02 03 04 should not shown more than once)");
				left_right_flag = true;
			}else if((baseid%conf_scenario_colc) < conf_scenario_colc - 1 && func_Instreet(baseid,baseid+1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  previous 3 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;//with drive toward that direction
				}else if(delta_x > AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to right
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
				}else if(delta_x < -AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to left
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)+1;

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  previous 4 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt first junction info 02-----------------(01 02 03 04 should not shown more than once)");
				left_right_flag = true;
			}else{
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt  previous 5 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
//				exit(1);
			}

			if(baseid-conf_scenario_colc > -1 && func_Instreet(baseid,baseid-conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  previous 6 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_y) + fabs(delta_x) < 0.001){//stop
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y > AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to up
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y < -AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to down
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  previous 7 delta info shows that vehicle NOT up or down direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt first junction info 03-----------------(01 02 03 04 should not shown more than once)");
			}else if(baseid+conf_scenario_colc < conf_scenario_rowc*conf_scenario_colc && func_Instreet(baseid,baseid+conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  previous 8 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_y) + fabs(delta_x) < 0.001){//stop
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y > AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to up
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y < -AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to down
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)+1;
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  previous 9 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt first junction info 04-----------------(01 02 03 04 should not shown more than once)");
			}else{
				if(!left_right_flag){
					if(tmp_Output_flag)fprintf(stdout,"\nTAVRagent::recv_tavr_pkt  previous 10 delta info shows that vehicle NOT one of the four directions info as follows\n ,%d %0.4f %0.4f delta=%0.2f %0.2f",
									nodeid,vehicle_position_x(),vehicle_position_y(),delta_x,delta_y);
				}
			}
		}
	}else if(junc_row_next_list[nodeid] < 0){//second junction info decide
		if(func_Injunc(baseid,juncRadius,vehicle_position_x(),vehicle_position_y())){//inner circle
			junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
			junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

			//check the fresh of previous junction info
			if(fabs(junc_row_next_list[nodeid]-junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) > 2){//JUMP TO FAR AWAY?
				fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 1 error JUMP far away ,%d%d %d%d %0.4f %0.4f",
						junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],
						junc_row_next_list[nodeid],junc_col_next_list[nodeid],
						delta_x,delta_y);
				exit(1);
			}else if(fabs(junc_row_next_list[nodeid]-junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) > 1){//two hops away from previous
				junc_row_prev_list[nodeid] = junc_row_next_list[nodeid];
				junc_col_prev_list[nodeid] = junc_col_next_list[nodeid];
			}else if(fabs(junc_row_next_list[nodeid]-junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) > 0){//adjoining
				junc_row_prev_list[nodeid] = junc_row_next_list[nodeid];
				junc_col_prev_list[nodeid] = junc_col_next_list[nodeid];
			}else if(fabs(junc_row_next_list[nodeid]-junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) == 0){//just the same
				junc_row_prev_list[nodeid] = junc_row_next_list[nodeid];
				junc_col_prev_list[nodeid] = junc_col_next_list[nodeid];
			}else{
				fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 2 error for the vehicle should toward one of the four directions of inner circle, %0.4f %0.4f",delta_x,delta_y);
				exit(1);
			}
		}else{
			if((baseid%conf_scenario_colc) > 0 && func_Instreet(baseid,baseid-1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 3 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid adjoins with previous junction id?
						if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-1-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;

							junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 4 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}
					}else if(func_Adjoinjunc(baseid-1,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid-1 adjoins with previous junction id?
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)-1;

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
					}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 3){//baseid two hops away from previous base
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
					}else{
						fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 5 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
						exit(1);
					}
				}else if(delta_x > AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to right
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)-1;

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_x < -AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to left
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 6 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt second junction info 01-----------------(01 02 03 04 should not shown more than once)");
				left_right_flag = true;
			}else if((baseid%conf_scenario_colc) < conf_scenario_colc - 1 && func_Instreet(baseid,baseid+1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 7 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid adjoins with previous junction id?
						if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)+1-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;

							junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 8 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}
					}else if(func_Adjoinjunc(baseid+1,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid-1 adjoins with previous junction id, thus new street info update
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)+1;

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);//means the vehicle move to new street
					}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 3){//baseid two hops away from previous base
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
					}else{
						fprintf(stderr,"TAVRagent::recv_tavr_pkt  NEXT 9 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
						exit(1);
					}
				}else if(delta_x > AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to right
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
				}else if(delta_x < -AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to left
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)+1;

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  NEXT 10 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt second junction info 02-----------------(01 02 03 04 should not shown more than once)");
				left_right_flag = true;
			}else{
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt  NEXT 11 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
//				exit(1);
			}

			if(baseid-conf_scenario_colc > -1 && func_Instreet(baseid,baseid-conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt  NEXT 12 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid adjoins with previous junction id?
						if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
						}else if(fabs((baseid/conf_scenario_colc)-1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;

							junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt  NEXT 13 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}
					}else if(func_Adjoinjunc(baseid-conf_scenario_colc,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid-1 adjoins with previous junction id, thus new street info update
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)-1;
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);//means the vehicle move to new street
					}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 3){//baseid two hops away from previous base
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
					}else{
						fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 14 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
						exit(1);
					}
				}else if(delta_y > AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to up
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y < -AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to down
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 15 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt second junction info 03-----------------(01 02 03 04 should not shown more than once)");
			}else if(baseid+conf_scenario_colc < conf_scenario_rowc*conf_scenario_colc && func_Instreet(baseid,baseid+conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 16 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid adjoins with previous junction id?
						if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
						}else if(fabs((baseid/conf_scenario_colc)+1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
							junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
							junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

							junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
							junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 17 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}
					}else if(func_Adjoinjunc(baseid+conf_scenario_colc,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],-1)){//baseid-1 adjoins with previous junction id, thus new street info update
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)+1;
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);//means the vehicle move to new street
					}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 3){//baseid two hops away from previous base
						junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
						junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

						junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
						junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
					}else{
						fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 18 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
						exit(1);
					}
				}else if(delta_y > AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to up
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y < -AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to down
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 19 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt second junction info 04-----------------(01 02 03 04 should not shown more than once)");
			}else{
				if(!left_right_flag){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt NEXT 20 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}
//
			}
		}
	}else{//normal mode--------------------------------------------------------
		juncROWcur = (baseid/conf_scenario_colc);
		juncCOLcur = (baseid%conf_scenario_colc);


		if(func_Injunc(baseid,juncRadius,vehicle_position_x(),vehicle_position_y())){//inner circle

			//check the fresh of previous junction info
			if(fabs(juncROWcur-junc_row_prev_list[nodeid]) + fabs(juncCOLcur - junc_col_prev_list[nodeid]) > 3){//JUMP TO FAR AWAY?
				fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 1 error JUMP far away ,%d%d %d%d %0.4f %0.4f",
						junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],
						junc_row_next_list[nodeid],junc_col_next_list[nodeid],
						delta_x,delta_y);
				exit(1);
			}else if(fabs(juncROWcur-junc_row_prev_list[nodeid]) + fabs(juncCOLcur - junc_col_prev_list[nodeid]) > 2){//two hops away from next
				junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
				junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

				junc_row_prev_list[nodeid] = junc_row_next_list[nodeid];
				junc_col_prev_list[nodeid] = junc_col_next_list[nodeid];
			}else if(fabs(juncROWcur-junc_row_prev_list[nodeid]) + fabs(juncCOLcur - junc_col_prev_list[nodeid]) > 1){//two hops away from previous
				junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
				junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

				junc_row_prev_list[nodeid] = junc_row_next_list[nodeid];
				junc_col_prev_list[nodeid] = junc_col_next_list[nodeid];
			}else if(fabs(juncROWcur-junc_row_prev_list[nodeid]) + fabs(juncCOLcur - junc_col_prev_list[nodeid]) > 0){//adjoining
				junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
				junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

				junc_row_prev_list[nodeid] = junc_row_next_list[nodeid];
				junc_col_prev_list[nodeid] = junc_col_next_list[nodeid];
			}else if(fabs(juncROWcur-junc_row_prev_list[nodeid]) + fabs(juncCOLcur - junc_col_prev_list[nodeid]) == 0){//just the same
				junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
				junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

				junc_row_prev_list[nodeid] = junc_row_next_list[nodeid];
				junc_col_prev_list[nodeid] = junc_col_next_list[nodeid];
			}else{
				fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 2 error for the vehicle should toward one of the four directions of inner circle, %0.4f %0.4f",delta_x,delta_y);
				exit(1);
			}
			if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal junction info 00--------Inner circle---------(01 02 03 04 should not shown more than once)");
		}else{//--------checked up----------16:49--- vehicle position  baseid  previous junction next junction decide the following codes

			if((baseid%conf_scenario_colc) > 0 && func_Instreet(baseid,baseid-1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 3 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_next_list[nodeid],junc_col_next_list[nodeid],-1)){//baseid adjoins with previous junction id?
						if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_row_next_list[nodeid]) < 1){
							if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
								junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-1-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
								junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
								junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;

								junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
							}else{//illegal since the distance between next and previous no less than 2
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 4 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-1-junc_col_next_list[nodeid]) < 1){//baseid-1 == previous junction id?
							if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
								//no need to update
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-1-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
								junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
								junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
							}else{//illegal since the distance between next and previous no less than 2
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-1-junc_col_prev_list[nodeid]) < 2){
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)-1;
								}else{
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 5 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}
							}
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 2){//baseid-1 == previous junction id?
							if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
								//error direction should not take back
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 6 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid]-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
								junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

								junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
							}else{//illegal since the distance between next and previous no less than 2
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 7 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}
						}else{//illegal since the distance between next and baseid no less than 2
							fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 8 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}
					}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 3){//baseid is two hops away from next junction
						if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//Retrograde. error
							fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 9 the vehicle take a reentry at the same street  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}else{
							if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc-1)-junc_col_next_list[nodeid]) < 2){
								junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)-1;

								junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
							}else{
								junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

								junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
								junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
							}
						}
					}else{
						fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 10 delta info overflow of sum  ,%d %0.2f %0.2f prev=%d%d next=%d%d %d %0.2f %0.2f",
								nodeid,vehicle_position_x(),vehicle_position_y(),junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],
								junc_row_next_list[nodeid],junc_col_next_list[nodeid],baseid,delta_x,delta_y);
						exit(1);
					}//----------------------------check----06090958
					left_right_flag = true;
				}else if(delta_x > AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to right
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)-1;

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
					left_right_flag = true;
				}else if(delta_x < -AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to left
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)-1;
					left_right_flag = true;
				}else{
		//---------------to filter the condition that vehicle located at left street ,but have bigger up/down delta info
					if(fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) == 0){//from Y-axis direction movement
						if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 11 delta info shows that vehicle has Y-axis delta but located at left street NO update , %0.2f %0.2f",delta_x,delta_y);
					}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) == 0){
						if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 11 delta info shows that vehicle has Extreme X-axis delta and located at left street NO update  , %0.2f %0.2f",delta_x,delta_y);
					}else{
						fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 11 delta info shows that delat info is ERROR  , %0.2f %0.2f",delta_x,delta_y);
						exit(1);
					}

				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal junction info 01-----------------(01 02 03 04 should not shown more than once)");
			}else if((baseid%conf_scenario_colc) < conf_scenario_colc - 1 && func_Instreet(baseid,baseid+1,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 12 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_next_list[nodeid],junc_col_next_list[nodeid],-1)){//baseid adjoins with previous junction id?
							if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_row_next_list[nodeid]) < 1){
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)+1-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;

									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 13 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)+1-junc_col_next_list[nodeid]) < 1){//baseid-1 == previous junction id?
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									//no need to update
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)+1-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
									if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)+1-junc_col_prev_list[nodeid]) < 2){
										junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
										junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

										junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
										junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)+1;
									}else{
										fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 14 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
										exit(1);
									}
								}
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 2){//baseid-1 == previous junction id?
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									//error direction should not take back
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 15 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid]-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
								}else{//illegal since the distance between next and previous no less than 2
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 16 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}
							}else{//illegal since the distance between next and baseid no less than 2
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 17 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 3){//baseid is two hops away from next junction
							if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//Retrograde. error
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 18 the vehicle take a reentry at the same street  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}else{
								if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc+1)-junc_col_next_list[nodeid]) < 2){
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)+1;

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else{
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
								}
							}
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 19 delta info overflow of sum  ,%d %0.2f %0.2f prev=%d%d next=%d%d %d %0.2f %0.2f",
								nodeid,vehicle_position_x(),vehicle_position_y(),junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],
								junc_row_next_list[nodeid],junc_col_next_list[nodeid],baseid,delta_x,delta_y);
							exit(1);
						}
					left_right_flag = true;
				}else if(delta_x > AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to right
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc)+1;
					left_right_flag = true;
				}else if(delta_x < -AccelerationMINI && fabs(delta_y) < AccelerationMAXI){//to left
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc)+1;

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
					left_right_flag = true;
				}else{
		//---------------to filter the condition that vehicle located at left street ,but have bigger up/down delta info
					if(fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) == 0){//from Y-axis direction movement
						fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 20 delta info shows that vehicle has Y-axis delta but located at right street NO update , %0.2f %0.2f",delta_x,delta_y);
					}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) == 0){
						fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 20 delta info shows that vehicle has Extreme X-axis delta and located at right street NO update  , %0.2f %0.2f",delta_x,delta_y);
					}else{
						fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 20 delta info shows that delat info is ERROR  , %0.2f %0.2f",delta_x,delta_y);
						exit(1);
					}
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal junction info 02-----------------(01 02 03 04 should not shown more than once)");
			}else{//no left or right
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 21 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
			}
//------------------check-------06091018
			if(baseid-conf_scenario_colc > -1 && func_Instreet(baseid,baseid-conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 22 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_next_list[nodeid],junc_col_next_list[nodeid],-1)){//baseid adjoins with previous junction id?
							if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_row_next_list[nodeid]) < 1){//next adjoin baseid
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else if(fabs((baseid/conf_scenario_colc)-1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
									//no need to update
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 23 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}
							}else if(fabs((baseid/conf_scenario_colc)-1 - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 1){//baseid-1 == previous junction id?
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									//no need to update
								}else if(fabs((baseid/conf_scenario_colc)-1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
									if(fabs((baseid/conf_scenario_colc)-1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){
										junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
										junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

										junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)-1;
										junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
									}else{
										fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 24 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
										exit(1);
									}
								}
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 2){//baseid-1 == previous junction id?
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									//error direction should not take back
								}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid]-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 25 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}
							}else{//illegal since the distance between next and baseid no less than 2
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 26 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 3){//baseid is two hops away from next junction
							if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//Retrograde. error
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 27 the vehicle take a reentry at the same street  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}else{
								if(fabs((baseid/conf_scenario_colc)-1 - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 2){
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)-1;
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else{
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}
							}
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 28 delta info overflow of sum  ,%d %0.2f %0.2f prev=%d%d next=%d%d %d %0.2f %0.2f",
								nodeid,vehicle_position_x(),vehicle_position_y(),junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],
								junc_row_next_list[nodeid],junc_col_next_list[nodeid],baseid,delta_x,delta_y);
							exit(1);
						}
				}else if(delta_y > AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to up
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y < -AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to down
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)-1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					if(!left_right_flag){
			//---------------to filter the condition that vehicle located at left street ,but have bigger up/down delta info
						if(fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) == 0){//from Y-axis direction movement
							fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 29 delta info shows that vehicle has Y-axis delta and located at down street NO update , %0.2f %0.2f",delta_x,delta_y);
						}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) == 0){
							fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 29 delta info shows that vehicle has Extreme X-axis delta but located at down street NO update  , %0.2f %0.2f",delta_x,delta_y);
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 29 delta info shows that delat info is ERROR  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}
					}else{
						if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 29 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
					}

				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal junction info 03-----------------(01 02 03 04 should not shown more than once)");
				//-------------------------------check 1806091054
			}else if(baseid+conf_scenario_colc < conf_scenario_rowc*conf_scenario_colc && func_Instreet(baseid,baseid+conf_scenario_colc,laneWidth,vehicle_position_x(),vehicle_position_y(),juncRadius)){//locate street id
				if(fabs(delta_x) + fabs(delta_y) > 1.51){
					fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 30 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
					exit(1);
				}else if(fabs(delta_x) + fabs(delta_y) < 0.001){//stop
					if(func_Adjoinjunc(baseid,junc_row_next_list[nodeid],junc_col_next_list[nodeid],-1)){//baseid adjoins with previous junction id?
							if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_row_next_list[nodeid]) < 1){//next adjoin baseid
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else if(fabs((baseid/conf_scenario_colc)+1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
									//no need to update
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
//why two hops away between prev and next is NOT considered------1058
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 31 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}
							}else if(fabs((baseid/conf_scenario_colc)+1 - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 1){//baseid-1 == previous junction id?
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									//no need to update
								}else if(fabs((baseid/conf_scenario_colc)-1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){//baseid-1 == previous junction id?
									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
			//----------------------------------------
									if(fabs((baseid/conf_scenario_colc)+1 - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){
										junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
										junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);

										junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)+1;
										junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);
									}else{
										fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 32 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
										exit(1);
									}
								}
		//----------------------------------------
							}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 2){//baseid-1 == previous junction id?
								if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 1){
									//error direction should not take back
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 33 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) + fabs(junc_col_next_list[nodeid]-junc_col_prev_list[nodeid]) < 2){//baseid-1 == previous junction id?
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else{//illegal since the distance between next and previous no less than 2
									fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 34 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
									exit(1);
								}
							}else{//illegal since the distance between next and baseid no less than 2
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 35 delta info overflow of sum  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}
						}else if(fabs((baseid/conf_scenario_colc) - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 3){//baseid is two hops away from next junction
							if(fabs((baseid/conf_scenario_colc) - junc_row_prev_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_prev_list[nodeid]) < 2){//Retrograde. error
								fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 36 the vehicle take a reentry at the same street  , %0.2f %0.2f",delta_x,delta_y);
								exit(1);
							}else{
								if(fabs((baseid/conf_scenario_colc)+1 - junc_row_next_list[nodeid]) + fabs((baseid%conf_scenario_colc)-junc_col_next_list[nodeid]) < 2){
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)+1;
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}else{
									junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
									junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

									junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
									junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
								}
							}
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 37 delta info overflow of sum  ,%d %0.2f %0.2f prev=%d%d next=%d%d %d %0.2f %0.2f",
								nodeid,vehicle_position_x(),vehicle_position_y(),junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],
								junc_row_next_list[nodeid],junc_col_next_list[nodeid],baseid,delta_x,delta_y);
							exit(1);
						}
				}else if(delta_y > AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to up
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc)+1;
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else if(delta_y < -AccelerationMINI && fabs(delta_x) < AccelerationMAXI){//to down
					junc_row_prev_list[nodeid] = (baseid/conf_scenario_colc)+1;
					junc_col_prev_list[nodeid] = (baseid%conf_scenario_colc);

					junc_row_next_list[nodeid] = (baseid/conf_scenario_colc);
					junc_col_next_list[nodeid] = (baseid%conf_scenario_colc);
				}else{
					if(!left_right_flag){
			//---------------to filter the condition that vehicle located at left street ,but have bigger up/down delta info
						if(fabs(junc_col_next_list[nodeid] - junc_col_prev_list[nodeid]) == 0){//from Y-axis direction movement
							if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 39 delta info shows that vehicle has Y-axis delta and located at up street NO update , %0.2f %0.2f",delta_x,delta_y);
						}else if(fabs(junc_row_next_list[nodeid] - junc_row_prev_list[nodeid]) == 0){
							if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 39 delta info shows that vehicle has Extreme X-axis delta but located at up street NO update  , %0.2f %0.2f",delta_x,delta_y);
						}else{
							fprintf(stderr,"TAVRagent::recv_tavr_pkt normal 39 delta info shows that delat info is ERROR  , %0.2f %0.2f",delta_x,delta_y);
							exit(1);
						}
					}
				}
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal junction info 04-----------------(01 02 03 04 should not shown more than once)");
			}else{
				if(tmp_Output_flag)fprintf(stdout,"TAVRagent::recv_tavr_pkt normal 39 delta info shows that vehicle NOT left or right direction  , %0.2f %0.2f",delta_x,delta_y);
//				exit(1);
			}
		}
	}
}

void
TAVRagent::update_junc_info_id(int id){
	char tmpStr[400];

	info_update_list[id] = false;

	int nodeid = id;
	int vehiid = Address::instance().get_lastaddr(vehicle_ip());
	//update junction info
	int r=0;
	int c=0;
	double mini_Ystep = (conf_scenario_Height_ - 2*conf_starty_) / (conf_scenario_rowc - 1) / (conf_base_colc+1)/10;
	double mini_Xstep = (conf_scenario_Width_ - 2*conf_startx_) / (conf_scenario_colc - 1) / (conf_base_rowc+1)/10;

	mini_Ystep = 10.002;
	mini_Xstep = 10.002;

	int posiX = 0;
	int posiY = 0;



	int juncROWpre = -1;
	int juncCOLpre = -1;
	int juncROWnxt = -1;
	int juncCOLnxt = -1;

	int juncUSED = -1;
	double distMAX = conf_scenario_Height_ + conf_scenario_Width_;
	 return ;

	//	if(conf_node_id==1)fprintf(stdout,"\n------------------------------------%d\n",conf_node_id);
	if(nodeid==189)fprintf(stdout,"\n-----------------------------%d-------%d\n",conf_node_id,nodeid);

	//current vehicular base previous
	double tmpDIST =  0.0;

	if(junc_row_prev_list[vehiid] < 0 && junc_col_prev_list[vehiid] < 0 && junc_row_next_list[vehiid] < 0 && junc_col_next_list[vehiid] < 0){
		fprintf(stderr,"TAVRagent::update_junc_info_id --%d----%d--  junction info error. MAKE sure that hello msg started after tavr_routing",id,vehiid);
		exit(1);
	}else if(junc_row_next_list[vehiid] < 0 && junc_col_next_list[vehiid] < 0){
		junc_row_next_list[vehiid] = junc_row_prev_list[vehiid];
		junc_col_next_list[vehiid] = junc_col_prev_list[vehiid];
	}else{

	}



	tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[vehiid]][junc_col_prev_list[vehiid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[vehiid]][junc_col_prev_list[vehiid]])
						+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[vehiid]][junc_col_prev_list[vehiid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[vehiid]][junc_col_prev_list[vehiid]])  );

	if(distMAX > tmpDIST + 3.25){
		distMAX = tmpDIST;
		juncROWpre = junc_row_prev_list[vehiid];
		juncCOLpre = junc_col_prev_list[vehiid];
	}

	//current vehicular base next
	tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_next_list[vehiid]][junc_col_next_list[vehiid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_next_list[vehiid]][junc_col_next_list[vehiid]])
						+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_next_list[vehiid]][junc_col_next_list[vehiid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_next_list[vehiid]][junc_col_next_list[vehiid]])  );

	if(distMAX > tmpDIST + 3.25){
		distMAX = tmpDIST;
		juncROWpre = junc_row_next_list[vehiid];
		juncCOLpre = junc_col_next_list[vehiid];
	}

	//adjoining junctions

	//base previous---adjoin-----left
	if(junc_col_prev_list[vehiid]-1 > -1){
		juncUSED = junc_col_prev_list[vehiid]-1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[vehiid]][juncUSED])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[vehiid]][juncUSED])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[vehiid]][juncUSED])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[vehiid]][juncUSED])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = junc_row_prev_list[vehiid];
			juncCOLpre = juncUSED;
		}
	}
	//base previous---adjoin-----right
	if(junc_col_prev_list[vehiid]+1 < conf_scenario_colc){
		juncUSED = junc_col_prev_list[vehiid]+1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[vehiid]][juncUSED])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[vehiid]][juncUSED])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[vehiid]][juncUSED])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[vehiid]][juncUSED])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = junc_row_prev_list[vehiid];
			juncCOLpre = juncUSED;
		}
	}
	//base previous---adjoin-----up
	if(junc_row_prev_list[vehiid]+1 < conf_scenario_rowc){
		juncUSED = junc_row_prev_list[vehiid]+1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[vehiid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[vehiid]])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[vehiid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[vehiid]])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = juncUSED;
			juncCOLpre = junc_col_prev_list[vehiid];
		}
	}
	//base previous---adjoin-----down
	if(junc_row_prev_list[vehiid]-1 > -1){
		juncUSED = junc_row_prev_list[vehiid]-1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[vehiid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[vehiid]])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[vehiid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[vehiid]])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = juncUSED;
			juncCOLpre = junc_col_prev_list[vehiid];
		}
	}


	//base next---adjoin-----left
	if(junc_col_next_list[vehiid]-1 > -1){
		juncUSED = junc_col_next_list[vehiid]-1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_next_list[vehiid]][juncUSED])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_next_list[vehiid]][juncUSED])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_next_list[vehiid]][juncUSED])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_next_list[vehiid]][juncUSED])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = junc_row_next_list[vehiid];
			juncCOLpre = juncUSED;
		}
	}
	//base next---adjoin-----right
	if(junc_col_next_list[vehiid]+1 < conf_scenario_colc){
		juncUSED = junc_col_next_list[vehiid]+1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_next_list[vehiid]][juncUSED])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_next_list[vehiid]][juncUSED])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_next_list[vehiid]][juncUSED])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_next_list[vehiid]][juncUSED])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = junc_row_next_list[vehiid];
			juncCOLpre = juncUSED;
		}
	}
	//base next---adjoin-----up
	if(junc_row_next_list[vehiid]+1 < conf_scenario_rowc){
		juncUSED = junc_row_next_list[vehiid]+1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_next_list[vehiid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_next_list[vehiid]])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_next_list[vehiid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_next_list[vehiid]])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = juncUSED;
			juncCOLpre = junc_col_next_list[vehiid];
		}
	}
	//base next---adjoin-----down
	if(junc_row_next_list[vehiid]-1 > -1){
		juncUSED = junc_row_next_list[vehiid]-1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_next_list[vehiid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_next_list[vehiid]])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_next_list[vehiid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_next_list[vehiid]])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			juncROWpre = juncUSED;
			juncCOLpre = junc_col_next_list[vehiid];
		}
	}

	//located at junction area?
	if(distMAX - 9 < 0){
		junction_LIST_[nodeid] = true;
		junc_row_next_list[nodeid] = juncROWpre;
		junc_col_next_list[nodeid] = juncCOLpre;
	}else{
		junction_LIST_[nodeid] = false;
	}


//combine the adjoining four junctions to locate the street info
//	if(conf_node_id==1){
	if(nodeid < 47){
		fprintf(stdout,"\n%d %d %d %d %d  base %0.2f %0.2f dist %0.2f %0.2f",
				juncROWpre,juncCOLpre,nodeid,
				vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],
				Xbas_LOC[juncROWpre][juncCOLpre],Ybas_LOC[juncROWpre][juncCOLpre],
				tmpDIST,distMAX);
	}

	junc_row_prev_list[nodeid] = juncROWpre;
	junc_col_prev_list[nodeid] = juncCOLpre;

	distMAX = conf_scenario_Height_ + conf_scenario_Width_;
	//---adjoin-----left*street
	if(junc_col_prev_list[nodeid]-1 > -1 && !junction_LIST_[nodeid]){
		juncUSED = junc_col_prev_list[nodeid]-1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[nodeid]][juncUSED])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[nodeid]][juncUSED])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[nodeid]][juncUSED])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[nodeid]][juncUSED])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			junc_row_next_list[nodeid] = junc_row_prev_list[nodeid];
			junc_col_next_list[nodeid] = juncUSED;
		}
	}
	//---adjoin-----right*street
	if(junc_col_prev_list[nodeid]+1 < conf_scenario_colc && !junction_LIST_[nodeid]){
		juncUSED = junc_col_prev_list[nodeid]+1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[nodeid]][juncUSED])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[junc_row_prev_list[nodeid]][juncUSED])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[nodeid]][juncUSED])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[junc_row_prev_list[nodeid]][juncUSED])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			junc_row_next_list[nodeid] = junc_row_prev_list[nodeid];
			junc_col_next_list[nodeid] = juncUSED;
		}
	}
	//---adjoin-----up*street
	if(junc_row_prev_list[nodeid]+1 < conf_scenario_rowc && !junction_LIST_[nodeid]){
		juncUSED = junc_row_prev_list[nodeid]+1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[nodeid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[nodeid]])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[nodeid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[nodeid]])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			junc_row_next_list[nodeid] = juncUSED;
			junc_col_next_list[nodeid] = junc_col_prev_list[nodeid];
		}
	}
	//---adjoin-----down*street
	if(junc_row_prev_list[nodeid]-1 > -1 && !junction_LIST_[nodeid]){
		juncUSED = junc_row_prev_list[nodeid]-1;
		tmpDIST = sqrt( (vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[nodeid]])*(vehicle_position_x_LIST_[nodeid] - Xbas_LOC[juncUSED][junc_col_prev_list[nodeid]])
							+(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[nodeid]])*(vehicle_position_y_LIST_[nodeid] - Ybas_LOC[juncUSED][junc_col_prev_list[nodeid]])  );

		if(distMAX > tmpDIST + 3.25){
			distMAX = tmpDIST;
			junc_row_next_list[nodeid] = juncUSED;
			junc_col_next_list[nodeid] = junc_col_prev_list[nodeid];
		}
	}
//	if(conf_node_id==1){
	if(nodeid < 47){
		fprintf(stdout,"\n%d %d %d %d %d %d %d  base %0.2f %0.2f %0.2f %0.2f dist %0.2f %0.2f",
				junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],nodeid,junc_row_next_list[nodeid],junc_col_next_list[nodeid],
				vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],
				Xbas_LOC[junc_row_prev_list[nodeid]][junc_col_prev_list[nodeid]],Ybas_LOC[junc_row_prev_list[nodeid]][junc_col_prev_list[nodeid]],
				Xbas_LOC[junc_row_next_list[nodeid]][junc_col_next_list[nodeid]],Ybas_LOC[junc_row_next_list[nodeid]][junc_col_next_list[nodeid]],
				tmpDIST,distMAX);
	}


//	if(conf_node_id==1)fprintf(stdout,"\n-----------------------------------%d\n",conf_node_id);


	int16_t tmpSpeed = updateDirection(vehicle_speed_LIST_[nodeid],id);
	vehicle_speed_LIST_[nodeid] = tmpSpeed;

	if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		switch(vehicle_direction_LIST_[nodeid]){
		case DUP:
			sprintf(tmpStr,"\n**%d...from\tid=%d\tx=%d\ty=%d\tv=%d\t UP\tjunc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
					vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
					junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
					recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
			break;
		case DDOWN:
			sprintf(tmpStr,"\n**%d...from\tid=%d\tx=%d\ty=%d\tv=%d\t DOWN\tjunc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
					vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
					junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
					recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
			break;
		case DLEFT:
			sprintf(tmpStr,"\n**%d...from\tid=%d\tx=%d\ty=%d\tv=%d\t LEFT\tjunc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
					vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
					junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
					recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
			break;
		case DRIGHT:
			sprintf(tmpStr,"\n**%d...from\tid=%d\tx=%d\ty=%d\tv=%d\t RIGHT\tjunc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
					vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
					junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
					recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
			break;
		case -17:
		case -73:
			sprintf(tmpStr,"\n**%d...from\tid=%d\tx=%d\ty=%d\tv=%d\t STOP junc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
					vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
					junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
					recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
			break;
		default:
			sprintf(tmpStr,"\n**%d.ERROR..from\tid=%d\tx=%d\ty=%d\tv=%d\tjunc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
					vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
					junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
					recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
		}
		TAVRagent::file_op.helmsg_write(tmpStr);
	}
}



void
TAVRagent::update_junc_info(){
	return ;
	char tmpStr[400];

	for(int i=0; wire_info_recv&&i< current_veh_num; i++){
//		int nodeid = Address::instance().get_lastaddr(vehicle_ip());
		int nodeid = i;

		if(!info_update_list[i]){
			if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
				sprintf(tmpStr,"\n\t%d\talone\tid=%d.\tx=%d\ty=%d\tv=%d\tjunc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
						vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
						junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
						recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
				TAVRagent::file_op.wirmsg_write(tmpStr);
			}
			continue;
		}
		info_update_list[i] = false;


		int16_t tmpSpeed = updateDirection(vehicle_speed_LIST_[nodeid],nodeid);
		vehicle_speed_LIST_[nodeid] = tmpSpeed;

		//update junction info
		double mini_Ystep = (conf_scenario_Height_ - 2*conf_starty_) / (conf_scenario_rowc - 1) / (conf_base_colc+1)/10;
		double mini_Xstep = (conf_scenario_Width_ - 2*conf_startx_) / (conf_scenario_colc - 1) / (conf_base_rowc+1)/10;

		mini_Ystep = 10.002;
		mini_Xstep = 10.002;



//find the closest junction
		double distMAX = conf_scenario_Height_ + conf_scenario_Width_;

//find the neighboring junc


		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
			sprintf(tmpStr,"\n\t%d\tlist\tid=%d.\tx=%d\ty=%d\tv=%d\tjunc_pre=r%dc%d\tjunc_next=r%dc%d\tfresh=%d\tutime=%0.4f",(conf_node_id + comm_id*2+1),(nodeid + comm_id*2+1),
					vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid],vehicle_speed_LIST_[nodeid],
					junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
					recv_seqno_List_[nodeid],vehicle_info_updateTime_LIST_[nodeid]);
			TAVRagent::file_op.wirmsg_write(tmpStr);
		}
	}

	wire_info_recv = false;
}



void
TAVRagent::recv(Packet *p, Handler *) {
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);
	char					tmpStr[260];

	if(!std_NETWORK_flag)return;

	if(current_veh_num > VEHICULAR_AMOUNT){
		fprintf(stderr,"error TAVRagent::recv () ...veh_num overflow...");
		exit(1);
	}

	if(ch->ptype() == PT_SIMUTAVR){

		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
			sprintf(tmpStr,"\n---***---recv route manager  sip=%s cip=%s ",
						Address::instance().print_nodeaddr(ih->saddr()),
						Address::instance().print_nodeaddr(vehicle_ip()));
			TAVRagent::file_op.rtr_write(tmpStr);
		}
	}else if(ch->ptype() == PT_TAVRHELLO){

		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
			sprintf(tmpStr,"\n---***---TAVRagent, recv hello  sip=%s cip=%s dip=%s stime=%0.4f",
						Address::instance().print_nodeaddr(ih->saddr()),
						Address::instance().print_nodeaddr(vehicle_ip()),
						Address::instance().print_nodeaddr(ih->daddr()),
						ch->timestamp());
			TAVRagent::file_op.rtr_write(tmpStr);
		}
	}else if(ch->ptype() == PT_TAVRWIRED){


		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
			sprintf(tmpStr,"\n---***---TAVRagent, recv wired sip=%s cip=%s dip=%s stime=%0.4f",
						Address::instance().print_nodeaddr(ih->saddr()),
						Address::instance().print_nodeaddr(vehicle_ip()),
						Address::instance().print_nodeaddr(ih->daddr()),
						ch->timestamp());
			TAVRagent::file_op.rtr_write(tmpStr);
		}

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


/**
 * whether vehicle located at base ID?
 * */
bool
TAVRagent::func_Injunc(int startBASE, double juncRadius, double vehiX, double vehiY){

	int sBr = startBASE/conf_scenario_colc;
	int sBc = startBASE%conf_scenario_colc;

	if((vehiX - Xbas_LOC[sBr][sBc])*(vehiX - Xbas_LOC[sBr][sBc]) +  (vehiY - Ybas_LOC[sBr][sBc])*(vehiY - Ybas_LOC[sBr][sBc]) < juncRadius*juncRadius){//1.414
		return true;
	}else{
		return false;
	}
}

/**
 * startBASE must located at left & down of endBASE
 * */
bool
TAVRagent::func_Instreet(int startBASE, int endBASE, double laneWidth, double vehiX, double vehiY,double junctionRadius){
	double  tmp_Slope_prevBasenextBase = 0.0;//k
	double	tmp_Margin_prevBasenextBase = 0.0;//b   y=kx+b
	double  tmp_Jitter_laneEdgeLine	= 0.0;//always positive

	bool direction_flag = false;
	bool directionUP_flag = false;
	bool directionDOWN_flag = false;
	bool directionLEFT_flag = false;
	bool directionRIGHT_flag = false;

	int sBr = startBASE/conf_scenario_colc;
	int sBc = startBASE%conf_scenario_colc;
	int eBr = endBASE/conf_scenario_colc;
	int eBc = endBASE%conf_scenario_colc;

//	double 	innerSquarewidth = junctionRadius / 1.413;

	if(startBASE == endBASE){//in this case, all four neighbor junctions will be checked for locating the info of vehicle
		fprintf(stderr,"TAVRagent::func_Instreet NONE two juncs should be the one  , %d<>%d",startBASE,endBASE);
		exit(1);
	}else if((fabs(Xbas_LOC[eBr][eBc] - Xbas_LOC[sBr][sBc]) < 10.0) && (fabs(Ybas_LOC[eBr][eBc] - Ybas_LOC[sBr][sBc]) < 10.0)){//regular grid topology
		fprintf(stderr,"TAVRagent::func_Instreet the Distance between two junctions is too small  , %d %d",startBASE,endBASE);
		exit(1);
	}else{
		if(fabs(sBr-eBr) + fabs(sBc - eBc) > 1){//JUMP TO farther junction?
			fprintf(stderr,"TAVRagent::func_Instreet JUMP to further junction  , %d %d",startBASE,endBASE);
			exit(1);
		}else{

			if(fabs(Xbas_LOC[eBr][eBc] - Xbas_LOC[sBr][sBc]) < 0.01){//same column, of regular grid

			}else if(fabs(Ybas_LOC[eBr][eBc] - Ybas_LOC[sBr][sBc]) < 0.01){//same row, of regular grid

			}else {

				//check the relative positions between vehicle and baseid
				if((vehiX - Xbas_LOC[sBr][sBc]) > 0.0001){//NO left, since vehicle located right from baseid
					if(endBASE == startBASE-1)return false;
				}else if((vehiX - Xbas_LOC[sBr][sBc]) < -0.0001){//NO right, since vehicle located left from baseid
					if(endBASE == startBASE+1)return false;
				}else{
				}

				if(vehiY - Ybas_LOC[sBr][sBc] > 0.0001){//NO up, since vehicle located down from baseid
					if(endBASE == startBASE-conf_scenario_colc) return false;
				}else  if(vehiY - Ybas_LOC[sBr][sBc] < -0.0001){//NO down, since vehicle located up from baseid
					if(endBASE == startBASE+conf_scenario_colc) return false;
				}else{
				}


				if(eBc-sBc){//left || right directions
					if(endBASE < startBASE){
						tmp_Slope_prevBasenextBase = (Ybas_LOC[sBr][sBc] - Ybas_LOC[eBr][eBc])/(Xbas_LOC[sBr][sBc] - Xbas_LOC[eBr][eBc]);
						tmp_Margin_prevBasenextBase = Ybas_LOC[eBr][eBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[eBr][eBc];
					}else{
						tmp_Slope_prevBasenextBase = (Ybas_LOC[eBr][eBc] - Ybas_LOC[sBr][sBc])/(Xbas_LOC[eBr][eBc] - Xbas_LOC[sBr][sBc]);
						tmp_Margin_prevBasenextBase = Ybas_LOC[sBr][sBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[sBr][sBc];
					}
//					tmp_Slope_prevBasenextBase = (Ybas_LOC[eBr][eBc] - Ybas_LOC[sBr][sBc])/(Xbas_LOC[eBr][eBc] - Xbas_LOC[sBr][sBc]);
//					tmp_Margin_prevBasenextBase = Ybas_LOC[sBr][sBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[sBr][sBc];
					tmp_Jitter_laneEdgeLine = laneWidth/sqrt(1/(tmp_Slope_prevBasenextBase*tmp_Slope_prevBasenextBase + 1));
					fprintf(stdout,"\n k=%0.2f b=%0.2f vx=%0.2f vy=%0.2f maxy=%0.2f miny=%0.2f",
							tmp_Slope_prevBasenextBase,tmp_Margin_prevBasenextBase,vehiX, vehiY,
							tmp_Slope_prevBasenextBase*vehiX + tmp_Margin_prevBasenextBase  + tmp_Jitter_laneEdgeLine,
							tmp_Slope_prevBasenextBase*vehiX + tmp_Margin_prevBasenextBase  - tmp_Jitter_laneEdgeLine);

					if(vehiY < tmp_Slope_prevBasenextBase*vehiX + tmp_Margin_prevBasenextBase  + tmp_Jitter_laneEdgeLine){//below the maximum value
						if(vehiY > tmp_Slope_prevBasenextBase*vehiX + tmp_Margin_prevBasenextBase  - tmp_Jitter_laneEdgeLine){//beyond the minimum value
							directionLEFT_flag = true;
						}else{
							directionLEFT_flag = false;
						}
					}else{
						directionLEFT_flag = false;
					}
				}

				if(eBr-sBr){//up || down directions
					if(endBASE<startBASE){
						tmp_Slope_prevBasenextBase = (Ybas_LOC[sBr][sBc] - Ybas_LOC[eBr][eBc])/(Xbas_LOC[sBr][sBc] - Xbas_LOC[eBr][eBc]);
						tmp_Margin_prevBasenextBase = Ybas_LOC[eBr][eBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[eBr][eBc];
					}else{
						tmp_Slope_prevBasenextBase = (Ybas_LOC[eBr][eBc] - Ybas_LOC[sBr][sBc])/(Xbas_LOC[eBr][eBc] - Xbas_LOC[sBr][sBc]);
						tmp_Margin_prevBasenextBase = Ybas_LOC[sBr][sBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[sBr][sBc];
					}
//					tmp_Slope_prevBasenextBase = (Ybas_LOC[eBr][eBc] - Ybas_LOC[sBr][sBc])/(Xbas_LOC[eBr][eBc] - Xbas_LOC[sBr][sBc]);
//					tmp_Margin_prevBasenextBase = Ybas_LOC[sBr][sBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[sBr][sBc];
					tmp_Jitter_laneEdgeLine = laneWidth*sqrt(1+1/(tmp_Slope_prevBasenextBase*tmp_Slope_prevBasenextBase));
					fprintf(stdout,"\n k=%0.2f b=%0.2f vx=%0.2f vy=%0.2f maxx=%0.2f minx=%0.2f",
							tmp_Slope_prevBasenextBase,tmp_Margin_prevBasenextBase,vehiX, vehiY,
							(vehiY - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  + tmp_Jitter_laneEdgeLine,
							(vehiY - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  - tmp_Jitter_laneEdgeLine);
					if(vehiX < (vehiY - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  + tmp_Jitter_laneEdgeLine){//below the maximum value
						if(vehiX > (vehiY - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  - tmp_Jitter_laneEdgeLine){//beyond the minimum value
							directionUP_flag = true;
						}else{
							directionUP_flag = false;
						}
					}else{
						directionUP_flag = false;
					}
				}

				if(directionUP_flag && directionLEFT_flag){
					fprintf(stderr,"TAVRagent::func_Instreet both direction info can be induced  , %d %d %0.2f %0.2f",startBASE,endBASE, vehiX, vehiY);
					exit(1);
				}else if(directionLEFT_flag){
					if(endBASE == startBASE-1) return true;
					else if(endBASE == startBASE+1) return true;
					else return false;
				}else if(directionUP_flag){
					if(endBASE == startBASE-conf_scenario_colc) return true;
					else if(endBASE == startBASE+conf_scenario_colc) return true;
					else return false;
				}else{
					return false;//current decision is NOT  ....
				}
			}
		}
	}
	return direction_flag;
}
/**
 * whether the two base id are adjoin? if prev=next, return true;
 * */
bool
TAVRagent::func_Adjoinjunc(int curID, int curCOL, int preROW=-1, int preCOL=-1){
	if(preROW < 0 && preCOL < 0){//only ids provided
		preROW = curCOL/conf_scenario_colc;
		preCOL = curCOL%conf_scenario_colc;

		curCOL = curID%conf_scenario_colc;
		curID = curID/conf_scenario_colc;

		if(fabs(curID - preROW) + fabs(curCOL-preCOL) > 1){
			return false;
		}else{
			return true;
		}
	}else if(preCOL < 0){//previous id has been transformed into row & col
		preCOL = preROW;
		preROW = curCOL;
		curCOL = curID%conf_scenario_colc;
		curID = curID/conf_scenario_colc;

		if(fabs(curID - preROW) + fabs(curCOL-preCOL) > 1){
			return false;
		}else{
			return true;
		}
	}else{//both have been be rows & cols
		if(fabs(curID - preROW) + fabs(curCOL-preCOL) > 1){
			return false;
		}else{
			return true;
		}
	}
}

/**
 * decide the junction info of vehicular
*/
void
TAVRagent::recv_tavr_pkt(Packet *p) {

	if(Address::instance().get_firstaddr(vehicle_ip()) != 2){
//		Packet::free(p);
		return;
	}

	struct hdr_ip		*ih = HDR_IP(p);

	int nodeid = Address::instance().get_lastaddr(vehicle_ip());
	bs_ip_LIST_[nodeid] = ih->saddr();

	int baseid = Address::instance().get_lastaddr(ih->saddr());
	bool	tmp_Output_flag = true;


	int juncRadius = junc_Radius;//thus, the square included in the circle with radius ?/1.414  should NOT smaller than the width of lanes
	int laneWidth = lane_Width;//thus, the square included in the circle with radius ?/1.414  should NOT smaller than the width of lanes

	node_->update_position();
	double dst_x = node_->destX();
	double dst_y = node_->destY();
	double delta_x = node_->dX();
	double delta_y = node_->dY();

	vehicle_position_x() = node_->X();
	vehicle_position_y() = node_->Y();
	vehicle_speed() = node_->speed();

	func_updateJUNCinfo(nodeid,baseid,vehicle_position_x(),vehicle_position_y(),vehicle_speed(),delta_x,delta_y);

	info_update_list[nodeid] = true;
	vehicle_position_x_LIST_[nodeid] =(int16_t)floor(vehicle_position_x());
	vehicle_position_y_LIST_[nodeid] =(int16_t)floor(vehicle_position_y());
	vehicle_speed_LIST_[nodeid] =(int16_t)floor(vehicle_speed());
	recv_seqno_List_[nodeid] = 0;
	vehicle_info_updateTime_LIST_[nodeid] = Scheduler::instance().clock();// no direction info added to speed
	if(nodeid < 48){
		if(tmp_Output_flag)fprintf(stdout,"\nid=%d BASE=%d**%d %d %d %d %0.4f %0.4f\n",nodeid,baseid,junc_row_prev_list[nodeid],junc_col_prev_list[nodeid],junc_row_next_list[nodeid],junc_col_next_list[nodeid],
				vehicle_position_x_LIST_[nodeid],vehicle_position_y_LIST_[nodeid]);
	}

}

void
TAVRagent::recv_Wired(Packet *p) {
//	fprintf(stdout,"\t** -----  ** route, recv_Wired");
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);
	struct hdr_wired_infos	*wiredh = hdr_wired_infos::access(p);
	char tmpStr[200];


	int cnodeID = Address::instance().get_firstaddr(vehicle_ip());
	//recv
	int veh_amount = VEHICULAR_AMOUNT;

	if(cnodeID == 2 && debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		sprintf(tmpStr,"\nrecv_Wired -id-----x----y---v--recvSEQ-list ip=%s stime=%0.4f\trtime=%0.4f\n",Address::instance().print_nodeaddr(vehicle_ip()),ch->timestamp(),Scheduler::instance().clock());
		TAVRagent::file_op.wirmsg_write(tmpStr);
	}
	for(int i=0; cnodeID == 2 && i < current_veh_num; i++){

//		if((wiredh->infos_tstamplist()[i] < vehicle_info_updateTime_list()[i]) || (wiredh->vehicle_idlist()[i] < 0)) continue;//if no info received , do NOT update the previous infos
		if(wiredh->vehicle_idlist()[i] < 0) continue;
//		if((recv_seqno_List_[i] == 0)) continue;// own the fresh data
		if(updateValid(ch->timestamp(),wiredh->recv_seqnolist()[i],vehicle_info_updateTime_LIST_[i],recv_seqno_List_[i])){
			sprintf(tmpStr,"\n** %d\t%d\t%0.4f\t%0.4f\t%d\t%d**",current_veh_num,(i + comm_id*2+1),ch->timestamp(),vehicle_info_updateTime_LIST_[i],wiredh->recv_seqnolist()[i],recv_seqno_List_[i]);
			TAVRagent::file_op.rtr_write(tmpStr);
			info_update_list[i] = false;
			continue;
		}
		char ipStr[15] = {'\0'};
		sprintf(ipStr,"2.0.%d",wiredh->vehicle_idlist()[i]);
		info_update_list[i] = true;
//		fprintf(stdout," -recv_Wired- ipStr=%s -*- ",ipStr);

		vehicle_ip_LIST_[i] = (nsaddr_t)Address::instance().str2addr(ipStr);
		vehicle_position_x_LIST_[i] = wiredh->vehicle_position_xlist()[i];
		vehicle_position_y_LIST_[i] = wiredh->vehicle_position_ylist()[i];
		vehicle_speed_LIST_[i]      = wiredh->vehicle_speedlist()[i];
		recv_seqno_List_[i] 			 = wiredh->recv_seqnolist()[i];
		vehicle_info_updateTime_LIST_[i] = ch->timestamp();

		if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
			sprintf(tmpStr,"\n** %d\t%d\t%d\t%d\t%d\t%d**",current_veh_num,(i + comm_id*2+1),vehicle_position_x_LIST_[i],vehicle_position_y_LIST_[i],vehicle_speed_LIST_[i],recv_seqno_List_[i]);
			TAVRagent::file_op.wirmsg_write(tmpStr);
		}
	}
	//to app
	if(cnodeID == 2){

		wire_info_recv = true;
		update_junc_info();

		ch->next_hop() = vehicle_ip();
		ih->dport() = port_TAVR_App;
		ih->daddr() = vehicle_ip();
		dmux_->recv(p,(Handler*)0);

	}else{//base NODE

		ch->direction() = hdr_cmn::DOWN;
		ch->addr_type() = NS_AF_ILINK;
		ch->next_hop() = IP_BROADCAST;

		ih->daddr() = IP_BROADCAST;
		ih->ttl_ = 5;
		Scheduler::instance().schedule(target_,p,TAVR_RTR_WIRED_RET_JITTER);
	}
}

void
TAVRagent::recv_Hello(Packet *p) {
//	fprintf(stdout,"** -----  ** route, recv_hello");
	struct hdr_veh_hello	*hello = hdr_veh_hello::access(p);
	struct hdr_cmn 			*ch = HDR_CMN(p);
	struct hdr_ip			*ih = HDR_IP(p);

	if(ih->saddr() == vehicle_ip()){//to links

		update_vehicular_info();

		//forward
		int cnodeID = Address::instance().get_lastaddr(vehicle_ip());
		int pnodeID = hello->vehicle_id();
		info_update_list[cnodeID] = true;
		hello->vehicle_position_x() = vehicle_position_x_LIST_[cnodeID];
		hello->vehicle_position_y() = vehicle_position_y_LIST_[cnodeID];
		hello->vehicle_speed() = vehicle_speed_LIST_[cnodeID];
		hello->vehicle_id() = cnodeID;
		vehicle_info_updateTime_LIST_[cnodeID] = ch->timestamp();
		update_junc_info_id(cnodeID);


//		ch->next_hop() = (u_int32_t)node_->base_stn();
		ch->next_hop() = IP_BROADCAST;
		ch->addr_type() = NS_AF_ILINK;
		ch->direction() = hdr_cmn::DOWN;

		Scheduler::instance().schedule(target_, p, TAVR_RTR_HELLO_RET_JITTER);
	}else if(Address::instance().get_firstaddr(vehicle_ip()) == 2){//from neighbors
		//recv
		int cnodeID = hello->vehicle_id();

		if(cnodeID < 0 || cnodeID >= current_veh_num){
			fprintf(stderr,"ERROR, a hello packet .with nodeID invalid...tavr-----recv_Hello..");
			exit(1);
		}
		info_update_list[cnodeID] = true;

		char ipStr[15] = {'\0'};
		sprintf(ipStr,"2.0.%d",hello->vehicle_id());
//		fprintf(stdout," -recv_Hello- ipStr=%s -*- ",ipStr);

		vehicle_ip_LIST_[cnodeID] = Address::instance().str2addr(ipStr);
		vehicle_position_x_LIST_[cnodeID] = hello->vehicle_position_x();
		vehicle_position_y_LIST_[cnodeID] = hello->vehicle_position_y();
		vehicle_speed_LIST_[cnodeID]      = hello->vehicle_speed();
		recv_seqno_List_[cnodeID]		  = 0;//the latest info
		vehicle_info_updateTime_LIST_[cnodeID] = ch->timestamp();
		update_junc_info_id(cnodeID);



		ch->direction() = hdr_cmn::UP;
		ch->next_hop() = vehicle_ip();

		ih->daddr() = vehicle_ip();
		ih->dport() = port_TAVR_App;

		dmux_->recv(p,(Handler*)0);
	}else if(Address::instance().get_firstaddr(vehicle_ip()) == 1){//base NODE recv & forward
		fprintf(stdout,"\n   ***   Alert, a hello packet to AP=%s....tavr-----recv_Hello..",Address::instance().print_nodeaddr(vehicle_ip()));

		tavr_forward_data(p);
	}else{
		fprintf(stderr,"ERROR, a hello packet ....tavr-----recv_Hello..");
		exit(1);
	}
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

void
TAVRagent::report_loc(){
	char tmpStr[400];
	int nodeid = Address::instance().get_lastaddr(vehicle_ip());

	node_->update_position();
	double cx = node_->X();
	double cy = node_->Y();
	double dst_x = node_->destX();
	double dst_y = node_->destY();
	double delta_x = node_->dX();
	double delta_y = node_->dY();
	double speed = node_->speed();

	double MiniStep = 2.50001;
	bool   loc_at_juncx = true;
	bool   loc_at_juncy = true;
	int    junc_rowNo=-1,junc_colNo=-1;


	if(debug_file  && (Scheduler::instance().clock() > file_wirte_Time + 0.001)){
		if(loc_at_juncx && loc_at_juncy){
			sprintf(tmpStr,"\nloc all report %d\t%d\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f\t%0.4f**",
					current_veh_num,(nodeid + comm_id*2+1),cx,cy,dst_x,dst_y,delta_x,delta_y,speed,Scheduler::instance().clock());
			TAVRagent::file_op.reachjunc_write(tmpStr);
		}

	}
}

void
TAVRagent::reset_tavr_protimer(){
	if(std_NETWORK_index < 80)tavr_protimer_.resched(detect_junc_interval);
}

int TAVRagent::command(int argc, const char*const* argv) {
	if (argc == 2) {
		if (strcasecmp(argv[1], "start") == 0) {
//			tavr_pkttimer_.resched(0.0);
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
		}else if (strcasecmp (argv[1], "send_Rmsg") == 0) {
//			tavr_protimer_.resched(0.0);
			return TCL_OK;
		}else if (strcasecmp (argv[1], "test_BASE") == 0) {
			print_bs();
			return TCL_OK;
		}


		else if(strcasecmp(argv[1], "vehiLocal_Bcast") == 0){
			S_vehiLinfoBcastTimer.resched(0.0);
			return (TCL_OK);
		}else if(strcasecmp(argv[1], "vehiGlobal_Bcast") == 0){
			S_vehiGinfoBcastTimer.resched(0.0);
			return (TCL_OK);
		}else if(strcasecmp(argv[1], "baseLocal_Bcast") == 0){
			S_baseLinfoBcastTimer.resched(0.0);
			return (TCL_OK);
		}
	}
	else if (argc == 3) {
			//Obtains corresponding dmux to carry packets to upper layers
		TclObject *obj;
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
		}else if (strcasecmp (argv[1], "vehicle_gone") == 0) {

			if (strcmp(argv[2], "true") == 0){
				std_NETWORK_flag = true;
				std_NETWORK_index = 10;
			}else{
				std_NETWORK_flag = false;
				std_NETWORK_index = 100;
			}
			return TCL_OK;
		}else if (strcasecmp (argv[1], "vehi-num") == 0) {
			conf_scenario_vehicular_amout = atoi(argv[2]);
			current_veh_num = conf_scenario_vehicular_amout;
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
		}else if (strcasecmp (argv[1], "app_port") == 0) {
			port_TAVR_App = (int32_t)atoi(argv[2]);
			return TCL_OK;
		}else if (strcmp(argv[1], "tavr_debug") == 0) {
			if (strcmp(argv[2], "true") == 0) debug_file = true;
			else debug_file = false;
			return (TCL_OK);
		}else if (strcmp(argv[1], "tavr_fileTime") == 0) {
			file_wirte_Time = atof(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "wired_interval") == 0) {
			wired_send_interval = atof(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "hello_interval") == 0) {
			hello_send_interval = atof(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "prepare_probability") == 0) {
			if (strcmp(argv[2], "true") == 0) detect_junc_flag = true;
			else detect_junc_flag = false;
			return (TCL_OK);
		}else if (strcmp(argv[1], "uplength") == 0) {
			strcpy(strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Ulength");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					UP_length[UP_len_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			UP_length[UP_len_indexR][cindex++] = atof(floatTMP);

			UP_len_indexC = cindex;
			UP_len_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "upangle") == 0) {
			strcpy(strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Uangle");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					UP_angle[UP_angle_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			UP_angle[UP_angle_indexR][cindex++] = atof(floatTMP);

			UP_angle_indexC = cindex;
			UP_angle_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "rightlength") == 0) {
			strcpy(strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Rlength");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					RIGHT_length[RIGHT_len_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			RIGHT_length[RIGHT_len_indexR][cindex++] = atof(floatTMP);

			RIGHT_len_indexC = cindex;
			RIGHT_len_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "rightangle") == 0) {
			strcpy(strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Rangle");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					RIGHT_angle[RIGHT_angle_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			RIGHT_angle[RIGHT_angle_indexR][cindex++] = atof(floatTMP);

			RIGHT_angle_indexC = cindex;
			RIGHT_angle_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "Xlist") == 0) {
			strcpy(strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for JUNC xList");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					Xbas_LOC[Xbas_LOC_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			Xbas_LOC[Xbas_LOC_indexR][cindex++] = atof(floatTMP);

			Xbas_LOC_indexC = cindex;
			Xbas_LOC_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "Ylist") == 0) {
			strcpy(strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for JUNC yList");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					Ybas_LOC[Ybas_LOC_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			Ybas_LOC[Ybas_LOC_indexR][cindex++] = atof(floatTMP);

			Ybas_LOC_indexC = cindex;
			Ybas_LOC_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "send_interval") == 0) {
			detect_junc_interval = atof(argv[2]);
			return (TCL_OK);
		}

//------------------------------------------------------------------------

		else if(strcmp(argv[1], "gscenWidth") == 0) {
			Gscen_width = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "gscenLength") == 0) {
			Gscen_length = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "gscenRowc") == 0) {
			Gscen_rowC = atoi(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "gscenColc") == 0) {
			Gscen_colC = atoi(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "glaneWidth") == 0) {
			Glane_width = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "gjuncRadius") == 0) {
			Gjunc_radius = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "gscenVehiNum") == 0) {
			Gcurrent_vnum = atoi(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerVlocalperiod") == 0) {
			S_timer_vlocal_Binterval = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerVglobalperiod") == 0) {
			S_timer_vglobal_Binterval = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerBlocalperiod") == 0) {
			S_timer_blocal_Binterval = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerVlocalflag") == 0) {
			S_timer_vlocal_RUN_flag = (strcmp(argv[2],"true"))?true:false;
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerVglobalflag") == 0) {
			S_timer_vglobal_RUN_flag = (strcmp(argv[2],"true"))?true:false;
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerBlocalflag") == 0) {
			S_timer_blocal_RUN_flag = (strcmp(argv[2],"true"))?true:false;
			return (TCL_OK);
		}else if(strcmp(argv[1], "sNode_runflag") == 0) {
			S_node_RUN_flag = (strcmp(argv[2],"true"))?true:false;
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerVlocalstart") == 0) {
			S_timer_vlocal_Bstart = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerVglobalstart") == 0) {
			S_timer_vglobal_Bstart = atof(argv[2]);
			return (TCL_OK);
		}else if(strcmp(argv[1], "stimerBlocalstart") == 0) {
			S_timer_blocal_Bstart = atof(argv[2]);
			return (TCL_OK);
		}else if (strcmp(argv[1], "guplength") == 0) {
			strcpy(G_strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(G_strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Ulength");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					GUP_length[G_UP_len_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = G_strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			GUP_length[G_UP_len_indexR][cindex++] = atof(floatTMP);

			G_UP_len_indexC = cindex;
			G_UP_len_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "gupangle") == 0) {
			strcpy(G_strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(G_strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Uangle");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					GUP_angle[G_UP_angle_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = G_strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			GUP_angle[G_UP_angle_indexR][cindex++] = atof(floatTMP);

			G_UP_angle_indexC = cindex;
			G_UP_angle_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "grightlength") == 0) {
			strcpy(strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(G_strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Rlength");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					GRIGHT_length[G_RIGHT_len_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = G_strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			GRIGHT_length[G_RIGHT_len_indexR][cindex++] = atof(floatTMP);

			G_RIGHT_len_indexC = cindex;
			G_RIGHT_len_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "grightangle") == 0) {
			strcpy(G_strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(G_strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for street Rangle");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					GRIGHT_angle[G_RIGHT_angle_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = G_strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			GRIGHT_angle[G_RIGHT_angle_indexR][cindex++] = atof(floatTMP);

			G_RIGHT_angle_indexC = cindex;
			G_RIGHT_angle_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "gXlist") == 0) {
			strcpy(G_strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(G_strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for JUNC xList");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					GXbas_LOC[G_Xbas_LOC_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = G_strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			GXbas_LOC[G_Xbas_LOC_indexR][cindex++] = atof(floatTMP);

			G_Xbas_LOC_indexC = cindex;
			G_Xbas_LOC_indexR++;
			return (TCL_OK);
		}else if (strcmp(argv[1], "gYlist") == 0) {
			strcpy(G_strTMP,argv[2]);
			int sindex = 0;
			int	cindex = 0;
			char floatTMP[20];
			for(int i=0; i< strlen(argv[2]); i++){
				if(G_strTMP[i] == ','){
					if(sindex > 17){
						fprintf(stderr,"error in type cast for JUNC yList");
						return TCL_ERROR;
					}
					floatTMP[sindex++] = '\0';
					GYbas_LOC[G_Ybas_LOC_indexR][cindex++] = atof(floatTMP);
					sindex = 0;
				}else{
					floatTMP[sindex++] = G_strTMP[i];
				}
			}

			floatTMP[sindex++] = '\0';
			GYbas_LOC[G_Ybas_LOC_indexR][cindex++] = atof(floatTMP);

			G_Ybas_LOC_indexC = cindex;
			G_Ybas_LOC_indexR++;
			return (TCL_OK);
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
	for(int r=0; r < conf_scenario_rowc; r++){
		if(conf_node_id==1)fprintf(stdout,"\n");
		for(int c=0; c < conf_scenario_colc; c++){
			if(conf_node_id==1)fprintf(stdout," base %0.2f %0.2f",Xbas_LOC[r][c],Ybas_LOC[r][c]);

		}

	}
}

//=====================test part======================

void
TAVRagent::init_juncInfo(){
/*
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
*/

//	printf("Rabbit, %u am here. The command has finished", vehicle_ip());
}

bool
TAVRagent::speedDeltaEqual_compare(double pd,double listd){
	double epsilon = 0.0001;

	if(fabs(pd - listd) < epsilon) return true;

	return false;
}

bool
TAVRagent::timeValid_compare(double pd,double listd){
	double epsilon = 0.0001;

	if(fabs(pd - listd) < epsilon) return false;

	return (pd < listd);
}

bool
TAVRagent::updateValid(double pd,int pno,double listd,int lno){
	double lastInfoTimer = pd - pno * hello_send_interval;
	double freshInfoTimer = listd - lno * hello_send_interval;

	return timeValid_compare(lastInfoTimer,freshInfoTimer);
}

int16_t
TAVRagent::updateDirection(int16_t speed, int id){
	int16_t tmpSpeed = 0;
	if(speed < MOVE_UP){//00
		vehicle_direction_LIST_[id] = -17;
		tmpSpeed = speed;
	}else if(speed < MOVE_LEFT){//00
		vehicle_direction_LIST_[id] = DUP;
		tmpSpeed = speed - MOVE_UP;
	}else if(speed < MOVE_DOWN){//01
		vehicle_direction_LIST_[id] = DLEFT;
		tmpSpeed = speed - MOVE_LEFT;
	}else if(speed < MOVE_RIGHT){//10
		vehicle_direction_LIST_[id] = DDOWN;
		tmpSpeed = speed - MOVE_DOWN;
	}else if(speed < MOVE_NULL){//11
		vehicle_direction_LIST_[id] = DRIGHT;
		tmpSpeed = speed - MOVE_RIGHT;
	}else{//11
		vehicle_direction_LIST_[id] = -73;
		tmpSpeed = speed;
	}
	return tmpSpeed;
}

bool
TAVRagent::getDetect_junc(){
	return detect_junc_flag;
}







//==============-----------------------=======----------------------------
void
TAVRagent::new_SvehiUpdate_localinfo(){
	new_Supdate_selfInfo();
}

void
TAVRagent::new_SbaseUpdate_localinfo(){
/*	 Tcl& tcl = Tcl::instance();
	 tcl.evalc("set shortpath10TO1");
	 string tmp=tcl.result();
	 printf("the sequence 10-1 is %s",tmp.c_str());*/
}

void
TAVRagent::new_update_globalInfo(){
/*	 Tcl& tcl = Tcl::instance();
	 tcl.evalc("set shortpath0TO1");
	 sprintf();
	 string tmp=tcl.result();
	 printf("the sequence 0-1 is %s",tmp.c_str());*/
}

void
TAVRagent::new_update_localInfo(){
	new_SvehiUpdate_localinfo();
	new_SbaseUpdate_localinfo();
}

void
TAVRagent::new_reset_vehi_Global_infoTimer(){
	G_id_uT[0] = 12;
	S_vehiGinfoBcastTimer.resched(S_timer_vglobal_Binterval);
}

void
TAVRagent::new_reset_vehi_Local_infoTimer(){
	S_vehiLinfoBcastTimer.resched(S_timer_vlocal_Binterval);
}

void
TAVRagent::new_reset_base_Local_infoTimer(){
	S_baseLinfoBcastTimer.resched(S_timer_blocal_Binterval);
}

void
TAVRagent::new_Supdate_selfInfo(){
	char tmpStr[400];
	int nodeid = Address::instance().get_lastaddr(vehicle_ip());

	node_->update_position();
	double dst_x = node_->destX();
	double dst_y = node_->destY();
	double delta_x = node_->dX();
	double delta_y = node_->dY();
	new_Supdate_JuncInfo(node_->X_,node_->Y_);//bid info
	S_vehicle_direction_= new_Supdate_DirectionInfo(node_->X_,node_->Y_,delta_x,delta_y);//orient info
	S_vehicle_position_x_ = node_->X_; //current position for instant hello msg
	S_vehicle_position_y_= node_->Y_;//
	S_vehicle_speed_= node_->speed_; //
}
/**
 * find driving direction of self
 * */
u_int8_t
TAVRagent::new_Supdate_DirectionInfo(double cvx, double cvy, double deltaX, double deltaY){
	char tmpStr[400];

	return 12;
}
/**
 * update self JUNCTION info--------------find the closest junction id? check in junction area?   which street?
 * */
void
TAVRagent::new_Supdate_JuncInfo(double cvx, double cvy){

	double 	tmp_Distance = 29999999999.0;
	int		tmp_BID = -1;
	int16_t	tmp_SaveBID = S_currentBID;
	int16_t	tmp_MaxBID = Gscen_rowC*Gscen_colC;
	int nodeid = Address::instance().get_lastaddr(vehicle_ip());

	if(S_checkFirstTime){
		for(int Bri=0; Bri<Gscen_rowC; Bri++){
			for(int Bci=0; Bci<Gscen_colC; Bci++){
				if(tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
					tmp_BID = Bri*Gscen_colC + Bci;
					tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
				}
			}
		}
		if(tmp_BID>-1){
			S_currentBID = tmp_BID;
			if(tmp_Distance < Gjunc_radius*Gjunc_radius){
				S_junction_ = true;
			}
		}
		S_checkFirstTime = false;
	}else{
		new_Spotential_Junc(S_currentBID,S_loc_BID_previous,S_loc_BID_toward);//S_potentialBaseIDlist save the result
		for(int pBindex=0; pBindex<39; pBindex++){
			if(S_potentialBaseIDlist[pBindex]<0){
				fprintf(stdout,"TAVRagent::new_Supdate_JuncInfo vehi-%d bid=%d %d %d has %d potential junction",nodeid,S_currentBID,S_loc_BID_previous,S_loc_BID_toward,pBindex);
				break;
			}
			int16_t		Bri = S_potentialBaseIDlist[pBindex]/Gscen_colC;
			int16_t		Bci = S_potentialBaseIDlist[pBindex]%Gscen_colC;
			if(tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
				tmp_BID = Bri*Gscen_colC + Bci;
				tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
			}
		}



/*

		int16_t		Bri = S_currentBID/Gscen_colC;
		int16_t		Bci = S_currentBID%Gscen_colC;
		//----------current BID-----check------------------
		if(tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---UP--check------------------
		Bri = S_currentBID/Gscen_colC + 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri<Gscen_rowC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---DOWN--check------------------
		Bri = S_currentBID/Gscen_colC - 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---LEFT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC - 1;
		if(Bci<Gscen_colC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---RIGHT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC + 1;
		if(Bci>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}

		//------------------------center with adjoin junction ID---------------------check---------------
			//-------------------------UP-center ID--------------------------
		S_currentBID = tmp_SaveBID + Gscen_colC;
		//----------current BID---UP--check------------------
		Bri = S_currentBID/Gscen_colC + 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri<Gscen_rowC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---DOWN--check------------------
		Bri = S_currentBID/Gscen_colC - 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---LEFT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC - 1;
		if(Bci<Gscen_colC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---RIGHT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC + 1;
		if(Bci>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
			//-------------------------DOWN-center ID--------------------------
		S_currentBID = tmp_SaveBID - Gscen_colC;
		//----------current BID---UP--check------------------
		Bri = S_currentBID/Gscen_colC + 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri<Gscen_rowC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---DOWN--check------------------
		Bri = S_currentBID/Gscen_colC - 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---LEFT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC - 1;
		if(Bci<Gscen_colC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---RIGHT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC + 1;
		if(Bci>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
			//-------------------------LEFT-center ID--------------------------
		S_currentBID = tmp_SaveBID - 1;
		//----------current BID---UP--check------------------
		Bri = S_currentBID/Gscen_colC + 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri<Gscen_rowC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---DOWN--check------------------
		Bri = S_currentBID/Gscen_colC - 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---LEFT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC - 1;
		if(Bci<Gscen_colC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---RIGHT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC + 1;
		if(Bci>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
			//-------------------------RIGHT-center ID--------------------------
		S_currentBID = tmp_SaveBID + 1;
		//----------current BID---UP--check------------------
		Bri = S_currentBID/Gscen_colC + 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri<Gscen_rowC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---DOWN--check------------------
		Bri = S_currentBID/Gscen_colC - 1;
		Bci = S_currentBID%Gscen_colC;
		if(Bri>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---LEFT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC - 1;
		if(Bci<Gscen_colC && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
		//----------current BID---RIGHT--check------------------
		Bri = S_currentBID/Gscen_colC;
		Bci = S_currentBID%Gscen_colC + 1;
		if(Bci>-1 && tmp_Distance > (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy)){
			tmp_BID = Bri*Gscen_colC + Bci;
			tmp_Distance = (GXbas_LOC[Bri][Bci]-cvx)*(GXbas_LOC[Bri][Bci]-cvx) + (GYbas_LOC[Bri][Bci]-cvy)*(GYbas_LOC[Bri][Bci]-cvy);
		}
*/

		if(tmp_BID>-1){
			S_currentBID = tmp_BID;
			if(tmp_Distance < Gjunc_radius*Gjunc_radius){
				S_junction_ = true;
			}else{
				//may be jump to far from current junction,  e.g.  record-BID=13(r=13/11 c=13%11)   but location indicates that vehicle at BID=16
			}
		}
	}

	bool tmp_UP = new_Scheck_street(cvx,cvy,NEW_ORIENT_UP);
	bool tmp_DOWN = new_Scheck_street(cvx,cvy,NEW_ORIENT_DOWN);
	bool tmp_LEFT = new_Scheck_street(cvx,cvy,NEW_ORIENT_LEFT);
	bool tmp_RIGHT = new_Scheck_street(cvx,cvy,NEW_ORIENT_RIGHT);






	if(tmp_UP && tmp_LEFT){//enter into overlap area, no action takes
		fprintf(stdout,"\nTAVRagent::new_Supdate_JuncInfo vehi-%d bid=%d %d %d has LU potential junctions",nodeid,S_currentBID,S_loc_BID_previous,S_loc_BID_toward);
	}else if(tmp_UP && tmp_RIGHT){//enter into overlap area, no action takes
		fprintf(stdout,"\nTAVRagent::new_Supdate_JuncInfo vehi-%d bid=%d %d %d has RU potential junctions",nodeid,S_currentBID,S_loc_BID_previous,S_loc_BID_toward);
	}else if(tmp_DOWN && tmp_LEFT){//enter into overlap area, no action takes
		fprintf(stdout,"\nTAVRagent::new_Supdate_JuncInfo vehi-%d bid=%d %d %d has LD potential junctions",nodeid,S_currentBID,S_loc_BID_previous,S_loc_BID_toward);
	}else if(tmp_DOWN && tmp_RIGHT){//enter into overlap area, no action takes
		fprintf(stdout,"\nTAVRagent::new_Supdate_JuncInfo vehi-%d bid=%d %d %d has RD potential junctions",nodeid,S_currentBID,S_loc_BID_previous,S_loc_BID_toward);
	}else if(tmp_UP){
		S_loc_BID_previous = S_currentBID;
		S_loc_BID_toward = S_loc_BID_previous + Gscen_colC;
	}else if(tmp_DOWN){
		S_loc_BID_previous = S_currentBID;
		S_loc_BID_toward = S_loc_BID_previous - Gscen_colC;
	}else if(tmp_LEFT){
		S_loc_BID_previous = S_currentBID;
		S_loc_BID_toward = S_loc_BID_previous - 1;
	}else if(tmp_RIGHT){
		S_loc_BID_previous = S_currentBID;
		S_loc_BID_toward = S_loc_BID_previous + 1;
	}else{//loop itself
/*		S_currentBID = tmp_SaveBID%Gscen_colC + 1 + tmp_SaveBID - tmp_SaveBID%Gscen_colC;
		if(S_currentBID < tmp_MaxBID) new_Supdate_JuncInfo(cvx,cvy);
		S_currentBID = tmp_SaveBID%Gscen_colC - 1 + tmp_SaveBID - tmp_SaveBID%Gscen_colC;
		if(S_currentBID > -1) new_Supdate_JuncInfo(cvx,cvy);
		S_currentBID = (tmp_SaveBID/Gscen_colC + 1)*Gscen_colC + tmp_SaveBID%Gscen_colC;
		if(S_currentBID < tmp_MaxBID) new_Supdate_JuncInfo(cvx,cvy);
		S_currentBID = (tmp_SaveBID/Gscen_colC - 1)*Gscen_colC + tmp_SaveBID%Gscen_colC;
		if(S_currentBID > -1) new_Supdate_JuncInfo(cvx,cvy);*/
		fprintf(stderr,"TAVRagent::new_Supdate_JuncInfo  street info calculate beyond design");
		exit(1);
	}
}

bool
TAVRagent::new_Scheck_street(double cvx, double cvy, int Orient_type){
	if(Orient_type == NEW_ORIENT_UP && S_currentBID + Gscen_colC > (Gscen_rowC-1)*Gscen_colC + Gscen_colC){
		return false;
	}else if(Orient_type == NEW_ORIENT_DOWN && S_currentBID - Gscen_colC < 0){
		return false;
	}
	if(Orient_type == NEW_ORIENT_RIGHT && S_currentBID + 1 > (Gscen_rowC-1)*Gscen_colC + Gscen_colC){
		return false;
	}else if(Orient_type == NEW_ORIENT_LEFT && S_currentBID - 1 < 0){
		return false;
	}

	double  tmp_Slope_prevBasenextBase = 0.0;//k
	double	tmp_Margin_prevBasenextBase = 0.0;//b   y=kx+b
	double  tmp_Jitter_laneEdgeLine	= 0.0;//always positive

	bool direction_flag = false;
	bool directionUP_flag = false;
	bool directionDOWN_flag = false;
	bool directionLEFT_flag = false;
	bool directionRIGHT_flag = false;

	int sBr = S_currentBID/conf_scenario_colc;
	int sBc = S_currentBID%conf_scenario_colc;
	int eBr = -1;
	int eBc = -1;

	bool	tmp_Check_valid = true;

	double 	innerSquarewidth = 3.2 / 1.413;

	if(Orient_type == NEW_ORIENT_UP){
		eBr = S_currentBID/conf_scenario_colc+1;
		eBc = sBc;
	}else if(Orient_type == NEW_ORIENT_DOWN){
		eBr = S_currentBID/conf_scenario_colc-1;
		eBc = sBc;
	}else if(Orient_type == NEW_ORIENT_RIGHT){
		eBr = sBr;
		eBc = S_currentBID%conf_scenario_colc+1;
	}else if(Orient_type == NEW_ORIENT_LEFT){
		eBr = sBr;
		eBc = S_currentBID%conf_scenario_colc-1;
	}else{
		fprintf(stderr,"TAVRagent::new_Scheck_street  potential street info error. No one of the four");
		exit(1);
	}

	if(fabs(GXbas_LOC[eBr][eBc] - GXbas_LOC[sBr][sBc]) < 2.0){//same column, of regular grid

	}else if(fabs(GYbas_LOC[eBr][eBc] - GYbas_LOC[sBr][sBc]) < 2.0){//same row, of regular grid

	}else{
		if(eBc-sBc){//must same row
/*			if(endBASE < startBASE){
				tmp_Slope_prevBasenextBase = (Ybas_LOC[sBr][sBc] - Ybas_LOC[eBr][eBc])/(Xbas_LOC[sBr][sBc] - Xbas_LOC[eBr][eBc]);
				tmp_Margin_prevBasenextBase = Ybas_LOC[eBr][eBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[eBr][eBc];
			}else{
				tmp_Slope_prevBasenextBase = (Ybas_LOC[eBr][eBc] - Ybas_LOC[sBr][sBc])/(Xbas_LOC[eBr][eBc] - Xbas_LOC[sBr][sBc]);
				tmp_Margin_prevBasenextBase = Ybas_LOC[sBr][sBc] - tmp_Slope_prevBasenextBase*Xbas_LOC[sBr][sBc];
			}*/
			tmp_Slope_prevBasenextBase = (GYbas_LOC[eBr][eBc] - GYbas_LOC[sBr][sBc])/(GXbas_LOC[eBr][eBc] - GXbas_LOC[sBr][sBc]);
			tmp_Margin_prevBasenextBase = GYbas_LOC[sBr][sBc] - tmp_Slope_prevBasenextBase*GXbas_LOC[sBr][sBc];
			tmp_Jitter_laneEdgeLine = Glane_width/sqrt(1/(tmp_Slope_prevBasenextBase*tmp_Slope_prevBasenextBase + 1));
			fprintf(stdout,"\n k=%0.2f b=%0.2f vx=%0.2f vy=%0.2f maxy=%0.2f miny=%0.2f",
					tmp_Slope_prevBasenextBase,tmp_Margin_prevBasenextBase,cvx, cvy,
					tmp_Slope_prevBasenextBase*cvx + tmp_Margin_prevBasenextBase  + tmp_Jitter_laneEdgeLine,
					tmp_Slope_prevBasenextBase*cvx + tmp_Margin_prevBasenextBase  - tmp_Jitter_laneEdgeLine);

			if(cvy < tmp_Slope_prevBasenextBase*cvx + tmp_Margin_prevBasenextBase  + tmp_Jitter_laneEdgeLine){//below the maximum value
				if(cvy > tmp_Slope_prevBasenextBase*cvx + tmp_Margin_prevBasenextBase  - tmp_Jitter_laneEdgeLine){//beyond the minimum value
					directionLEFT_flag = true;
				}else{
					directionLEFT_flag = false;
				}
			}else{
				directionLEFT_flag = false;
			}
			tmp_Check_valid = !tmp_Check_valid;
		}
		if(eBr-sBr){//must same column
			tmp_Slope_prevBasenextBase = (GYbas_LOC[eBr][eBc] - GYbas_LOC[sBr][sBc])/(GXbas_LOC[eBr][eBc] - GXbas_LOC[sBr][sBc]);
			tmp_Margin_prevBasenextBase = GYbas_LOC[sBr][sBc] - tmp_Slope_prevBasenextBase*GXbas_LOC[sBr][sBc];
			tmp_Jitter_laneEdgeLine = Glane_width*sqrt(1+1/(tmp_Slope_prevBasenextBase*tmp_Slope_prevBasenextBase));
			fprintf(stdout,"\n k=%0.2f b=%0.2f vx=%0.2f vy=%0.2f maxx=%0.2f minx=%0.2f",
					tmp_Slope_prevBasenextBase,tmp_Margin_prevBasenextBase,cvx, cvy,
					(cvy - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  + tmp_Jitter_laneEdgeLine,
					(cvy - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  - tmp_Jitter_laneEdgeLine);
			if(cvx < (cvy - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  + tmp_Jitter_laneEdgeLine){//below the maximum value
				if(cvx > (cvy - tmp_Margin_prevBasenextBase)/tmp_Slope_prevBasenextBase  - tmp_Jitter_laneEdgeLine){//beyond the minimum value
					directionUP_flag = true;
				}else{
					directionUP_flag = false;
				}
			}else{
				directionUP_flag = false;
			}
			tmp_Check_valid = !tmp_Check_valid;
		}

		if(tmp_Check_valid){
			fprintf(stderr,"TAVRagent::new_Scheck_street  the pair of %d-%d or %d-%d is error, since only one pair is zero",eBc,sBc,eBr,sBr);
			exit(1);
		}else{
			int startBASE = sBr*Gscen_colC + sBc;
			int endBASE = sBr*Gscen_colC + sBc;
			if(directionUP_flag && directionLEFT_flag){
				fprintf(stderr,"TAVRagent::new_Scheck_street both direction info can be induced  , %d %d %0.2f %0.2f",startBASE,endBASE, cvx, cvy);
				exit(1);
			}else if(directionLEFT_flag){
				if(endBASE == startBASE-1) return true;
				else if(endBASE == startBASE+1) return true;
				else return false;
			}else if(directionUP_flag){
				if(endBASE == startBASE-conf_scenario_colc) return true;
				else if(endBASE == startBASE+conf_scenario_colc) return true;
				else return false;
			}else{
				return false;//current decision is NOT  ....
			}
		}
	}

	return true;
}


void
TAVRagent::new_Spotential_Junc(int cBID,int pBID,int tBID){
	int 	tmp_BaseNum = Gscen_rowC*Gscen_colC;
	int		potentialCheckBase[tmp_BaseNum];
	bool	tmp_check_valid = true;
	for(int i=0; i<tmp_BaseNum; i++){
		potentialCheckBase[i] = 0;
		S_potentialBaseIDlist[i%39]=-1;
	}


	if(cBID <0){
		fprintf(stderr,"TAVRagent::new_Spotential_Junc S_currentBID should have positive value ");
		exit(1);
	}else{
		potentialCheckBase[cBID]++;

		if(cBID+1<tmp_BaseNum) potentialCheckBase[cBID+1]++;
		if(cBID-1>-1) potentialCheckBase[cBID-1]++;
		if(cBID+Gscen_colC<tmp_BaseNum) potentialCheckBase[cBID+Gscen_colC]++;
		if(cBID-Gscen_colC>-1) potentialCheckBase[cBID-Gscen_colC]++;

		if(cBID+1+1<tmp_BaseNum) 			potentialCheckBase[cBID+1+1]++;
		if(cBID+1-1>-1) 					potentialCheckBase[cBID+1-1]++;
		if(cBID+1+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[cBID+1+Gscen_colC]++;
		if(cBID+1-Gscen_colC>-1) 			potentialCheckBase[cBID+1-Gscen_colC]++;

		if(cBID-1+1<tmp_BaseNum) 			potentialCheckBase[cBID-1+1]++;
		if(cBID-1-1>-1) 					potentialCheckBase[cBID-1-1]++;
		if(cBID-1+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[cBID-1+Gscen_colC]++;
		if(cBID-1-Gscen_colC>-1) 			potentialCheckBase[cBID-1-Gscen_colC]++;

		if(cBID+Gscen_colC+1<tmp_BaseNum) 			potentialCheckBase[cBID+Gscen_colC+1]++;
		if(cBID+Gscen_colC-1>-1) 					potentialCheckBase[cBID+Gscen_colC-1]++;
		if(cBID+Gscen_colC+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[cBID+Gscen_colC+Gscen_colC]++;
		if(cBID+Gscen_colC-Gscen_colC>-1) 			potentialCheckBase[cBID+Gscen_colC-Gscen_colC]++;

		if(cBID-Gscen_colC+1<tmp_BaseNum) 			potentialCheckBase[cBID-Gscen_colC+1]++;
		if(cBID-Gscen_colC-1>-1) 					potentialCheckBase[cBID-Gscen_colC-1]++;
		if(cBID-Gscen_colC+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[cBID-Gscen_colC+Gscen_colC]++;
		if(cBID-Gscen_colC-Gscen_colC>-1) 			potentialCheckBase[cBID-Gscen_colC-Gscen_colC]++;
	}
	if(pBID<0){
		tmp_check_valid=false;
	}else{
		potentialCheckBase[pBID]++;

		if(pBID+1<tmp_BaseNum) potentialCheckBase[pBID+1]++;
		if(pBID-1>-1) potentialCheckBase[pBID-1]++;
		if(pBID+Gscen_colC<tmp_BaseNum) potentialCheckBase[pBID+Gscen_colC]++;
		if(pBID-Gscen_colC>-1) potentialCheckBase[pBID-Gscen_colC]++;

		if(pBID+1+1<tmp_BaseNum) 			potentialCheckBase[pBID+1+1]++;
		if(pBID+1-1>-1) 					potentialCheckBase[pBID+1-1]++;
		if(pBID+1+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[pBID+1+Gscen_colC]++;
		if(pBID+1-Gscen_colC>-1) 			potentialCheckBase[pBID+1-Gscen_colC]++;

		if(pBID-1+1<tmp_BaseNum) 			potentialCheckBase[pBID-1+1]++;
		if(pBID-1-1>-1) 					potentialCheckBase[pBID-1-1]++;
		if(pBID-1+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[pBID-1+Gscen_colC]++;
		if(pBID-1-Gscen_colC>-1) 			potentialCheckBase[pBID-1-Gscen_colC]++;

		if(pBID+Gscen_colC+1<tmp_BaseNum) 			potentialCheckBase[pBID+Gscen_colC+1]++;
		if(pBID+Gscen_colC-1>-1) 					potentialCheckBase[pBID+Gscen_colC-1]++;
		if(pBID+Gscen_colC+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[pBID+Gscen_colC+Gscen_colC]++;
		if(pBID+Gscen_colC-Gscen_colC>-1) 			potentialCheckBase[pBID+Gscen_colC-Gscen_colC]++;

		if(pBID-Gscen_colC+1<tmp_BaseNum) 			potentialCheckBase[pBID-Gscen_colC+1]++;
		if(pBID-Gscen_colC-1>-1) 					potentialCheckBase[pBID-Gscen_colC-1]++;
		if(pBID-Gscen_colC+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[pBID-Gscen_colC+Gscen_colC]++;
		if(pBID-Gscen_colC-Gscen_colC>-1) 			potentialCheckBase[pBID-Gscen_colC-Gscen_colC]++;
	}
	if(tBID<0){

	}else{
		if(!tmp_check_valid){
			fprintf(stderr,"TAVRagent::new_Spotential_Junc pBID has Negative value but tBID has positive value ");
			exit(1);
		}
		potentialCheckBase[tBID]++;

		if(tBID+1<tmp_BaseNum) potentialCheckBase[tBID+1]++;
		if(tBID-1>-1) potentialCheckBase[tBID-1]++;
		if(tBID+Gscen_colC<tmp_BaseNum) potentialCheckBase[tBID+Gscen_colC]++;
		if(tBID-Gscen_colC>-1) potentialCheckBase[tBID-Gscen_colC]++;

		if(tBID+1+1<tmp_BaseNum) 			potentialCheckBase[tBID+1+1]++;
		if(tBID+1-1>-1) 					potentialCheckBase[tBID+1-1]++;
		if(tBID+1+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[tBID+1+Gscen_colC]++;
		if(tBID+1-Gscen_colC>-1) 			potentialCheckBase[tBID+1-Gscen_colC]++;

		if(tBID-1+1<tmp_BaseNum) 			potentialCheckBase[tBID-1+1]++;
		if(tBID-1-1>-1) 					potentialCheckBase[tBID-1-1]++;
		if(tBID-1+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[tBID-1+Gscen_colC]++;
		if(tBID-1-Gscen_colC>-1) 			potentialCheckBase[tBID-1-Gscen_colC]++;

		if(tBID+Gscen_colC+1<tmp_BaseNum) 			potentialCheckBase[tBID+Gscen_colC+1]++;
		if(tBID+Gscen_colC-1>-1) 					potentialCheckBase[tBID+Gscen_colC-1]++;
		if(tBID+Gscen_colC+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[tBID+Gscen_colC+Gscen_colC]++;
		if(tBID+Gscen_colC-Gscen_colC>-1) 			potentialCheckBase[tBID+Gscen_colC-Gscen_colC]++;

		if(tBID-Gscen_colC+1<tmp_BaseNum) 			potentialCheckBase[tBID-Gscen_colC+1]++;
		if(tBID-Gscen_colC-1>-1) 					potentialCheckBase[tBID-Gscen_colC-1]++;
		if(tBID-Gscen_colC+Gscen_colC<tmp_BaseNum) 	potentialCheckBase[tBID-Gscen_colC+Gscen_colC]++;
		if(tBID-Gscen_colC-Gscen_colC>-1) 			potentialCheckBase[tBID-Gscen_colC-Gscen_colC]++;
	}
	int pIndex=0;
	for(int i=0; i<tmp_BaseNum; i++){
		if(potentialCheckBase[i])//  >0????
			S_potentialBaseIDlist[pIndex++]=i;
	}
}
