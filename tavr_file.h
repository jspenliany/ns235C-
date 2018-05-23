/*
 * tavr_app.h
 *
 *  Created on: Mar 16, 2018
 *      Author: js
 */

#ifndef AVANET_TAVR_FILE_H_
#define AVANET_TAVR_FILE_H_
#include <random.h>
#include "tavr_traffic_pkt.h"
#include <stdio.h>


class TAVRfile{
public:
	TAVRfile();

	void init_file(char*,int);
	void close();


	void app_write(char*, int id=-1);
	void txl_write(char*, int id=-1);
	void rtr_write(char*, int id=-1);
	void helmsg_write(char*, int id=-1);
	void wirmsg_write(char*, int id=-1);
	void reachjunc_write(char*, int id=-1);

protected:

private:
	void clean_file(char*);
	void create_app();
	void create_txl();
	void create_rtr();

	void create_hello();
	void create_wired();

	void create_junc();

	int  from_layer;
	int	 from_hello;
	int  from_wired;
	int  from_id;


	char appLayer_info[30];
	char txlLayer_info[30];
	char rtrLayer_info[30];

	char helloMSG_info[30];
	char wiredMSG_info[30];

	char reachJunc_info[30];

	FILE *appFile;
	FILE *txlFile;
	FILE *rtrFile;

	FILE *helloFile;
	FILE *wiredFile;

	FILE *juncFile;

};


#endif /* AVANET_TAVR_APP_H_ */
