/*
 * tavr.h
 *
 *  Created on: Jan 21, 2018
 *      Author: js
 */

#ifndef TAVR_H_
#define TAVR_H_

#include "tavr_traffic_pkt.h"
#include "tavr_file.h"

#include <agent.h>
#include <mobilenode.h>
#include <trace.h>
#include <classifier.h>
#include <string>
#include <classifier-port.h>
#include <list>

using namespace std;


#define CURRENT_TIME		Scheduler::instance().clock();


class TAVRagent;

class TAVR_pktTimer  : public TimerHandler {
public:
	TAVR_pktTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

class TAVR_proTimer  : public TimerHandler {
public:
	TAVR_proTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

//----------------------------------------------------------------------
class TAVR_vehiSinfoBcastTimer  : public TimerHandler {
public:
	TAVR_vehiSinfoBcastTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

class TAVR_vehiLinfoBcastTimer  : public TimerHandler {
public:
	TAVR_vehiLinfoBcastTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

class TAVR_baseLinfoBcastTimer  : public TimerHandler {
public:
	TAVR_baseLinfoBcastTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

class TAVR_vehiGinfoBcastTimer  : public TimerHandler {
public:
	TAVR_vehiGinfoBcastTimer(TAVRagent* agent){
		agent_ = agent;
	}
protected:
	TAVRagent* 	agent_;
	virtual	void expire(Event* e);
};

//=================================


class TAVRagent : public Agent{

	friend class TAVR_pktTimer;
	friend class TAVR_proTimer;

	friend class TAVR_SinfoBcastTimer;
	friend class TAVR_GinfoBcastTimer;
	friend class TAVR_LinfoBcastTimer;
	friend class TAVR_baseLinfoBcastTimer;

private:
	//self info
	nsaddr_t 	vehicle_ip_; //device ID----vehicle
	nsaddr_t 	all_ip_; //device ID----vehicle
	double 		vehicle_position_x_; //current position for instant hello msg
	double 		vehicle_position_y_; //
	double 		vehicle_speed_; //
	u_int8_t 	vehicle_direction_; //
	bool 		junction_;
	nsaddr_t 	bs_ip_;

	//hello msgs encounter
	u_int8_t	hello_amount_;
	double		firstHellorecv_ts_;

	//msg timer
	TAVR_pktTimer 	tavr_pkttimer_;
	TAVR_proTimer 	tavr_protimer_;



	u_int32_t	inner_seqno_;

	static TAVRfile file_op;
	static bool	file_exist;
	bool	debug_file;
	double	file_wirte_Time;

	double  wired_send_interval;
	double  hello_send_interval;
	double	detect_junc_interval;
	double	tavr_route_interval;
	bool	detect_junc_flag;
	bool	location_Calculate_flag;

	//vehicular info

	nsaddr_t 	vehicle_ip_LIST_[VEHICULAR_AMOUNT]; //device ID----vehicle
	int16_t 	vehicle_position_x_LIST_[VEHICULAR_AMOUNT]; //
	int16_t 	vehicle_position_y_LIST_[VEHICULAR_AMOUNT]; //
	int16_t 	vehicle_speed_LIST_[VEHICULAR_AMOUNT]; //
	int16_t 	recv_seqno_List_[VEHICULAR_AMOUNT]; //
	double 		vehicle_info_updateTime_LIST_[VEHICULAR_AMOUNT]; //
	int16_t 	vehicle_direction_LIST_[VEHICULAR_AMOUNT]; //
	bool 		junction_LIST_[VEHICULAR_AMOUNT];
	nsaddr_t 	bs_ip_LIST_[VEHICULAR_AMOUNT];

	int16_t 	junc_row_prev_list[VEHICULAR_AMOUNT]; //
	int16_t 	junc_col_prev_list[VEHICULAR_AMOUNT]; //
	int16_t 	junc_row_next_list[VEHICULAR_AMOUNT]; //
	int16_t 	junc_col_next_list[VEHICULAR_AMOUNT]; //

	int16_t		junc_center_index;//-1 first time recv, 0 center area, 1 out of center area

	bool		info_update_list[VEHICULAR_AMOUNT];
	bool		wire_info_recv;

	bool		std_NETWORK_flag;//for organizing the activity period of nodes in network during times;
	bool		std_NETWORK_index;//for organizing the activity period of nodes in network during times;

	//
	MobileNode *node_;
	Trace *logtarget_;
	PortClassifier *dmux_;

	//otcl command methods for configuration parameter
	bool		conf_vehicle_;
	double		conf_scenario_Width_;
	double		conf_scenario_Height_;
	int			conf_scenario_rowc;
	int			conf_scenario_colc;
	int			conf_scenario_vehicular_amout;
	int			conf_scenario_base_amout;
	int			conf_node_id;
	int			conf_junc_row_bs;
	int			conf_junc_col_bs;

	double		conf_startx_;
	double		conf_starty_;
	int			conf_base_rowc;
	int			conf_base_colc;
	double		**junc_info_arrayX;
	double		**junc_info_arrayY;
	int			comm_id;


	double		UP_length[NET_ROW_MAX][NET_COL_MAX];
	double		RIGHT_length[NET_ROW_MAX][NET_COL_MAX];
	double		UP_angle[NET_ROW_MAX][NET_COL_MAX];
	double		RIGHT_angle[NET_ROW_MAX][NET_COL_MAX];
	double		Xbas_LOC[NET_ROW_MAX][NET_COL_MAX];
	double		Ybas_LOC[NET_ROW_MAX][NET_COL_MAX];

	int			UP_len_indexR;
	int			RIGHT_len_indexR;
	int			UP_angle_indexR;
	int			RIGHT_angle_indexR;
	int			Xbas_LOC_indexR;
	int			Ybas_LOC_indexR;

	int			UP_len_indexC;
	int			RIGHT_len_indexC;
	int			UP_angle_indexC;
	int			RIGHT_angle_indexC;
	int			Xbas_LOC_indexC;
	int			Ybas_LOC_indexC;
	char		strTMP[400];

	int			conf_test_INET;//bits VALID: six bits, first FLAG, second BROADCAST, third SUBNET, last INTERNET

	int         conf_test_AXIS_ip;

	int32_t		port_TAVR_App;


	bool		debug_flag;
	int			current_veh_num;


	double		lane_Width;
	double		junc_Radius;


//====================THE ABOVE ARE DISCARDED====0702

//self-info
	nsaddr_t 	S_vehicle_ip_; //device ID----vehicle
	nsaddr_t 	S_all_ip_; //device ID----vehicle
	double 		S_vehicle_position_x_; //current position for instant hello msg
	double 		S_vehicle_position_y_; //
	double 		S_vehicle_speed_; //
	u_int8_t 	S_vehicle_direction_; //
	bool 		S_junction_;
	nsaddr_t 	S_bs_ip_;
	int16_t		S_loc_BID_previous;//
	int16_t		S_loc_BID_toward;//
	bool		S_checkFirstTime;//for initiation of S_currentBID
	int16_t		S_currentBID;

	double		S_timer_slocal_Binterval;//update global-infos with period of ....
	double		S_timer_vlocal_Binterval;//update local-infos with period of ....
	double		S_timer_vglobal_Binterval;//update global-infos with period of ....
	double		S_timer_blocal_Binterval;//update global-infos with period of ....

	TAVR_vehiSinfoBcastTimer	S_vehiSinfoBcastTimer;
	TAVR_vehiGinfoBcastTimer	S_vehiGinfoBcastTimer;
	TAVR_vehiLinfoBcastTimer	S_vehiLinfoBcastTimer;
	TAVR_baseLinfoBcastTimer	S_baseLinfoBcastTimer;

	double		S_timer_vlocal_Bstart;//update local-infos with period of ....
	double		S_timer_vglobal_Bstart;//update global-infos with period of ....
	double		S_timer_blocal_Bstart;//update global-infos with period of ....

	bool		S_timer_slocal_RUN_flag;
	bool		S_timer_vlocal_RUN_flag;
	bool		S_timer_vglobal_RUN_flag;
	bool		S_timer_blocal_RUN_flag;
	bool		S_node_RUN_flag;

//local-infos
	int			L_nextIndex;	//next available index-----means the number of local neighbors
	int32_t		L_idTOindex[NEWL_VEHI_NUM];	//builds reflection from Global to Local(global id TO local index)
	int32_t		L_id_uT[NEWL_VEHI_NUM];	//id====12bits,  updateTime==19bits--include two point-right part
	int32_t		L_speed_direction[NEWL_VEHI_NUM];//speed=15bits,  direction=16bits----H&V
	int32_t		L_locX[NEWL_VEHI_NUM];//float data into integer
	int32_t		L_locY[NEWL_VEHI_NUM];
	int16_t		L_loc_BID_previous[NEWL_VEHI_NUM];//
	int16_t		L_loc_BID_toward[NEWL_VEHI_NUM];//

	int			Stmp_potentialBaseIDlist[3+3*12];
	int16_t		Stmp_loc_BID_previous;
	int16_t		Stmp_loc_BID_toward;



//-----------methods--------------------------------
	void new_SvehiUpdate_globalinfo();
	void new_SvehiUpdate_localinfo();
	void new_SbaseUpdate_localinfo();
	void new_SvehiUpdate_selfinfo();
//	void new_Supdate_selfInfo();
	u_int8_t new_SupdateSI_DirectionInfo(double,double);
	void new_SupdateSI_JuncInfo(double,double);
	bool new_ScheckSI_street(double,double,int);
	void new_SpotentialSI_Junc(int,int,int);

	void new_ScurrentVLI_neighbors();
//protected:
public:

	inline nsaddr_t& vehicle_ip() {
		return vehicle_ip_;
	}
	inline double& vehicle_position_x() {
		return vehicle_position_x_;
	}
	inline double& vehicle_position_y() {
		return vehicle_position_y_;
	}
	inline double& vehicle_speed() {
		return vehicle_speed_;
	}
	inline u_int8_t& vehicle_direction() {
		return vehicle_direction_;
	}
	inline bool& junction() {
		return junction_;
	}
	inline nsaddr_t bs_ip() {
		return bs_ip_;
	}

	inline int16_t* vehicle_speed_LIST() {
		return vehicle_speed_LIST_;
	}
	inline int16_t* vehicle_position_x_LIST() {
		return vehicle_position_x_LIST_;
	}
	inline int16_t* vehicle_position_y_LIST() {
		return vehicle_position_y_LIST_;
	}
	inline int16_t* recv_seqno_List() {
		return recv_seqno_List_;
	}


	inline double* vehicle_info_updateTime_list() {
		return vehicle_info_updateTime_LIST_;
	}
	inline nsaddr_t* vehicle_ip_LIST() {
		return vehicle_ip_LIST_;
	}

	inline int16_t* vehicle_direction_LIST() {
		return vehicle_direction_LIST_;
	}
	inline bool* junction_LIST() {
		return junction_LIST_;
	}
	inline nsaddr_t* bs_ip_LIST() {
		return bs_ip_LIST_;
	}
	inline u_int8_t& hello_amount() {
		return hello_amount_;
	}
	inline int16_t* junc_row_prev_List() {
		return junc_row_prev_list;
	}
	inline int16_t* junc_col_prev_List() {
		return junc_col_prev_list;
	}
	inline int16_t* junc_row_next_List() {
		return junc_row_next_list;
	}
	inline int16_t* junc_col_next_List() {
		return junc_col_next_list;
	}


	inline double& firstHellorecv_ts() {
		return firstHellorecv_ts_;
	}
	inline bool	triggerReplyHello(double delay){
		double tmp = CURRENT_TIME;
		return ((firstHellorecv_ts_ + delay) < tmp);
	}





	void update_vehicular_info();
	void update_junc_info();
	void update_junc_info_id(int);
	bool timeValid_compare(double,double);
	bool speedDeltaEqual_compare(double,double);
	bool updateValid(double,int,double,int);
	int16_t updateDirection(int16_t,int);
	void report_loc();
	void reset_tavr_protimer();


	bool	func_Instreet(int,int,double,double,double,double);
	bool	func_Injunc(int,double,double,double);
	bool	func_Adjoinjunc(int,int,int,int);
	bool	func_updateJUNCinfo(int, double, double, double, double, double);
	bool	func_updateJUNCinfo(int, int, double, double, double, double, double);



//-------------------------------------------------------------------------------

	inline nsaddr_t&	new_S_vehicle_ip(){
		return S_vehicle_ip_;
	}
	inline nsaddr_t&	new_S_all_ip(){
		return S_all_ip_;
	}
	inline double&	new_S_vehicle_position_x(){
		return S_vehicle_position_x_;
	}
	inline double&	new_S_vehicle_position_y(){
		return S_vehicle_position_y_;
	}
	inline double&	new_S_vehicle_speed(){
		return S_vehicle_speed_;
	}
	inline u_int8_t&	new_S_vehicle_direction(){
		return S_vehicle_direction_;
	}
	inline bool&	new_S_junction(){
		return S_junction_;
	}
	inline nsaddr_t&	new_S_bs_ip(){
		return S_bs_ip_;
	}
	inline double&	new_S_timer_local_Binterval(){
		return S_timer_vlocal_Binterval;
	}
	inline double&	new_S_timer_global_Binterval(){
		return S_timer_vglobal_Binterval;
	}
	inline int&	new_L_nextIndex(){
		return L_nextIndex;
	}
	inline int32_t*	new_L_idTOindex(){
		return L_idTOindex;
	}
	inline int32_t*	new_L_id_uT(){
		return L_id_uT;
	}
	inline int32_t*	new_L_speed_direction(){
		return L_speed_direction;
	}
	inline int16_t*	new_L_loc_BID_previous(){
		return L_loc_BID_previous;
	}
	inline int16_t*	new_L_loc_BID_toward(){
		return L_loc_BID_toward;
	}

	inline bool new_S_node_active(){
		return S_node_RUN_flag;
	}
	inline bool new_S_selfUpdate_active(){
		return S_timer_slocal_RUN_flag;
	}
	inline bool new_S_localUpdate_active(){
		return S_timer_vlocal_RUN_flag;
	}
	inline bool new_S_globalUpdae_active(){
		return S_timer_vglobal_RUN_flag;
	}
	inline bool new_S_baseUpdate_active(){
		return S_timer_blocal_RUN_flag;
	}

	void new_update_selfInfo();
	void new_update_globalInfo();
	void new_update_localInfo();
	void new_update_BaselocalInfo();
	void new_reset_vehi_Global_infoTimer();
	void new_reset_vehi_Local_infoTimer();
	void new_reset_vehi_self_infoTimer();
	void new_reset_base_Local_infoTimer();



public:
	TAVRagent(nsaddr_t);
	~TAVRagent();
	int command(int argc, const char*const* argv);
	void recv(Packet*, Handler* callback = 0);
	void send_tavr_pkt();
	void reset_tavr_timer();
	void tavr_forward_data(Packet*);
	void recv_tavr_pkt(Packet*);
	void recv_Wired(Packet*);
	void recv_Hello(Packet*);
	void init_juncInfo();
	void print_bs();

	void base_sendBeacon();

	bool getDetect_junc();



	//global---infos
	static	double		Gscen_width;
	static 	double		Gscen_length;
	static	int			Gscen_rowC;
	static 	int			Gscen_colC;
	static  double		Glane_width;
	static 	double		Gjunc_radius;
	static 	double		Gneighbor_radius;
	static 	int			Gcurrent_vnum;
//	static	double		Gbase_loc[2][NEWG_BASE_NUM];

	static	double		GUP_length[NEWG_ROW_MAX][NEWG_COL_MAX];
	static	double		GRIGHT_length[NEWG_ROW_MAX][NEWG_COL_MAX];
	static	double		GUP_angle[NEWG_ROW_MAX][NEWG_COL_MAX];
	static	double		GRIGHT_angle[NEWG_ROW_MAX][NEWG_COL_MAX];
	static	double		GXbas_LOC[NEWG_ROW_MAX][NEWG_COL_MAX];
	static	double		GYbas_LOC[NEWG_ROW_MAX][NEWG_COL_MAX];
	static	int			G_UP_len_indexR;
	static	int			G_RIGHT_len_indexR;
	static	int			G_UP_angle_indexR;
	static	int			G_RIGHT_angle_indexR;
	static	int			G_Xbas_LOC_indexR;
	static	int			G_Ybas_LOC_indexR;

	static	int			G_UP_len_indexC;
	static	int			G_RIGHT_len_indexC;
	static	int			G_UP_angle_indexC;
	static	int			G_RIGHT_angle_indexC;
	static	int			G_Xbas_LOC_indexC;
	static	int			G_Ybas_LOC_indexC;
	static	char		G_strTMP[700];

	static 	int32_t		G_id_uT[NEWG_VEHI_NUM];//id====12bits,  updateTime==19bits--include two point-right part
	static 	int32_t		G_speed_direction[NEWG_VEHI_NUM];//speed=15bits,  direction=16bits----H&V
	static 	int32_t		G_locX[NEWG_VEHI_NUM];//float data into integer
	static 	int32_t		G_locY[NEWG_VEHI_NUM];

	static	int16_t		G_loc_BID_previous[NEWG_VEHI_NUM];//
	static	int16_t		G_loc_BID_toward[NEWG_VEHI_NUM];//

	static 	int32_t		LG_id_uT[NEWG_VEHI_NUM];//id====12bits,  updateTime==19bits--include two point-right part
	static 	int32_t		LG_speed_direction[NEWG_VEHI_NUM];//speed=15bits,  direction=16bits----H&V
	static 	int32_t		LG_locX[NEWG_VEHI_NUM];//float data into integer
	static 	int32_t		LG_locY[NEWG_VEHI_NUM];

	static	int16_t		LG_loc_BID_previous[NEWG_VEHI_NUM];//
	static	int16_t		LG_loc_BID_toward[NEWG_VEHI_NUM];//

	static 	int32_t		LG_NEW_id_uT[NEW_LG_LINKS_MAX][NEWG_VEHI_NUM];//id====12bits,  updateTime==19bits--include two point-right part
	static 	int32_t		LG_NEW_speed_direction[NEW_LG_LINKS_MAX][NEWG_VEHI_NUM];//speed=15bits,  direction=16bits----H&V
	static 	int32_t		LG_NEW_locX[NEW_LG_LINKS_MAX][NEWG_VEHI_NUM];//float data into integer
	static 	int32_t		LG_NEW_locY[NEW_LG_LINKS_MAX][NEWG_VEHI_NUM];

	static	int16_t		LG_NEW_loc_BID_previous[NEW_LG_LINKS_MAX][NEWG_VEHI_NUM];//
	static	int16_t		LG_NEW_loc_BID_toward[NEW_LG_LINKS_MAX][NEWG_VEHI_NUM];//


	//shortest path save
//	static 	string		G_shortPATHpairs_matrix[NEWG_BASE_NUM][NEWG_BASE_NUM];
//	static 	int			G_SpPm_index;

//---------------test part ----------------------


};



#endif /* TAVR_H_ */
