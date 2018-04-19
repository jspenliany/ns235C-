/*
 * tavr_app.cc
 *
 *  Created on: Mar 16, 2018
 *      Author: js
 */
#include "tavr_file.h"


TAVRfile::TAVRfile(){
}

/*
void
TAVRfile::next_line(){
	if(nextLine_flag == NLINE_LAYER){
		switch(from_layer){
		case APP_FROM:
			fprintf(appFile,"\n");break;
		case RTR_FROM:
			fprintf(rtrFile,"\n");break;
		case TXL_FROM:
			fprintf(txlFile,"\n");break;
		default:
			fprintf(stderr,"error TAVRfile::next_line ..from_layer  ERROR...");
			exit(1);
		}
	}else if(nextLine_flag == NLINE_MSG){
		switch(from_hello*10+from_wired){
		case 0:
			break;
		case WIRED_FROM:
			fprintf(wiredFile,"\n");break;
		case 10:
			fprintf(helloFile,"\n");break;
		case 11:
			fprintf(wiredFile,"\n");
			fprintf(helloFile,"\n");
			break;
		default:
			fprintf(stderr,"error TAVRfile::next_line ..  ERROR...");
			exit(1);
		}
	}
}
*/

void
TAVRfile::app_write(char *str, int id){
	if(appFile == NULL){
		fprintf(stderr,"error TAVRfile::app_write ..appFile.txt create ERROR...");
		exit(1);
	}

	if(id < 0)fprintf(appFile,"%s",str);
	else fprintf(appFile,"%s-----------------%d",str,id);
}

void
TAVRfile::txl_write(char *str, int id){
	if(txlFile == NULL){
		fprintf(stderr,"error TAVRfile::txl_write ..txlFile.txt create ERROR...");
		exit(1);
	}

	if(id < 0)fprintf(txlFile,"%s",str);
	else fprintf(txlFile,"%s-----------------%d",str,id);
}

void
TAVRfile::rtr_write(char *str, int id){
	if(rtrFile == NULL){
		fprintf(stderr,"error TAVRfile::rtr_write ..rtrFile.txt create ERROR...");
		exit(1);
	}

	if(id < 0)fprintf(rtrFile,"%s",str);
	else fprintf(rtrFile,"%s-----------------%d",str,id);
}

void
TAVRfile::helmsg_write(char *str, int id){
	if(helloFile == NULL){
		fprintf(stderr,"error TAVRfile::helmsg_write ..helloFile.txt create ERROR...");
		exit(1);
	}

	if(id < 0)fprintf(helloFile,"%s",str);
	else fprintf(helloFile,"%s-----------------%d",str,id);

}

void
TAVRfile::wirmsg_write(char *str, int id){
	if(wiredFile == NULL){
		fprintf(stderr,"error TAVRfile::wirmsg_write ..wiredFile.txt create ERROR...");
		exit(1);
	}

	if(id < 0)fprintf(wiredFile,"%s",str);
	else fprintf(wiredFile,"%s-----------------%d",str,id);
}


void
TAVRfile::init_file(char *file_pattern, int ft){
//	from_layer = ft;
	from_layer = ((ft%10000)/1000);
	from_hello = ((ft%100)/10);
	from_wired = ((ft%100)%10);

		clean_file(file_pattern);

		switch(from_layer){
		case APP_FROM:
			create_app();
			if(appFile == NULL){
				fprintf(stderr,"error TAVRfile::init_file ..%s create ERROR...",appFile);
				exit(1);
			}
			break;

		case RTR_FROM:
			create_rtr();
			if(rtrFile == NULL){
				fprintf(stderr,"error TAVRfile::init_file ..%s create ERROR...",rtrFile);
				exit(1);
			}
			break;

		case TXL_FROM:
			create_txl();
			if(txlFile == NULL){
				fprintf(stderr,"error TAVRfile::init_file ..%s create ERROR...",txlFile);
				exit(1);
			}
			break;
		default:
			fprintf(stderr,"error TAVRfile::init_file ..  ERROR...");
			exit(1);
		}

		switch(from_hello*10+from_wired){
		case 0:
			break;
		case WIRED_FROM:
			create_wired();
			if(wiredFile == NULL){
				fprintf(stderr,"error TAVRfile::init_file ..miss_wired.txt create ERROR...");
				exit(1);
			}
			break;
		case 10:
			create_hello();
			if(helloFile == NULL){
				fprintf(stderr,"error TAVRfile::init_file ..miss_hello.txt create ERROR...");
				exit(1);
			}
			break;
		case 11:
			create_hello();
			create_wired();
			if(helloFile == NULL){
				fprintf(stderr,"error TAVRfile::init_file ..miss_hello.txt create ERROR...");
				exit(1);
			}
			if(wiredFile == NULL){
				fprintf(stderr,"error TAVRfile::init_file ..miss_wired.txt create ERROR...");
				exit(1);
			}
			break;
		default:
			fprintf(stderr,"error TAVRfile::init_file ..  ERROR...");
			exit(1);
		}

		switch(from_hello*10+from_wired){
		case 0:
			break;
		case WIRED_FROM:
			break;
		case 10:
			break;
		case 11:
			break;
		default:
			fprintf(stderr,"error TAVRfile::init_file ..  ERROR...");
			exit(1);
		}

}


void
TAVRfile::clean_file(char *file_name){
	int i=0;

	if(((int)(sizeof(appLayer_info)/sizeof(char))) < 11){
		fprintf(stderr,"error TAVRfile::clean_file ..  ERROR.the length of file name ..");
		exit(1);
	}

	for(; i< ((int)(sizeof(appLayer_info)/sizeof(char)))-6; i++){
		if(file_name[i] == '\0') break;

		if(i < 4){
			if(from_layer == APP_FROM){
				appLayer_info[0] = 'a';
				appLayer_info[1] = 'p';
				appLayer_info[2] = 'p';
				appLayer_info[3] = 'L';
			}else if(from_layer == RTR_FROM){
				rtrLayer_info[0] = 'r';
				rtrLayer_info[1] = 't';
				rtrLayer_info[2] = 'r';
				rtrLayer_info[3] = 'L';
			}else if(from_layer == TXL_FROM){
				txlLayer_info[0] = 't';
				txlLayer_info[1] = 'x';
				txlLayer_info[2] = 'l';
				txlLayer_info[3] = 'L';
			}

			switch(from_hello*10+from_wired){
			case 0:
				break;
			case WIRED_FROM:
				if(from_layer == APP_FROM){
					wiredMSG_info[0] = 'a';
					wiredMSG_info[1] = 'p';
					wiredMSG_info[2] = 'p';
					wiredMSG_info[3] = 'W';
				}else if(from_layer == RTR_FROM){
					wiredMSG_info[0] = 'r';
					wiredMSG_info[1] = 't';
					wiredMSG_info[2] = 'r';
					wiredMSG_info[3] = 'W';
				}else if(from_layer == TXL_FROM){
					wiredMSG_info[0] = 't';
					wiredMSG_info[1] = 'x';
					wiredMSG_info[2] = 'l';
					wiredMSG_info[3] = 'W';
				}
				break;
			case 10:
				if(from_layer == APP_FROM){
					helloMSG_info[0] = 'a';
					helloMSG_info[1] = 'p';
					helloMSG_info[2] = 'p';
					helloMSG_info[3] = 'H';
				}else if(from_layer == RTR_FROM){
					helloMSG_info[0] = 'r';
					helloMSG_info[1] = 't';
					helloMSG_info[2] = 'r';
					helloMSG_info[3] = 'H';
				}else if(from_layer == TXL_FROM){
					helloMSG_info[0] = 't';
					helloMSG_info[1] = 'x';
					helloMSG_info[2] = 'l';
					helloMSG_info[3] = 'H';
				}
				break;
			case 11:
				if(from_layer == APP_FROM){
					helloMSG_info[0] = 'a';
					helloMSG_info[1] = 'p';
					helloMSG_info[2] = 'p';
					helloMSG_info[3] = 'H';
				}else if(from_layer == RTR_FROM){
					helloMSG_info[0] = 'r';
					helloMSG_info[1] = 't';
					helloMSG_info[2] = 'r';
					helloMSG_info[3] = 'H';
				}else if(from_layer == TXL_FROM){
					helloMSG_info[0] = 't';
					helloMSG_info[1] = 'x';
					helloMSG_info[2] = 'l';
					helloMSG_info[3] = 'H';
				}
				if(from_layer == APP_FROM){
					wiredMSG_info[0] = 'a';
					wiredMSG_info[1] = 'p';
					wiredMSG_info[2] = 'p';
					wiredMSG_info[3] = 'W';
				}else if(from_layer == RTR_FROM){
					wiredMSG_info[0] = 'r';
					wiredMSG_info[1] = 't';
					wiredMSG_info[2] = 'r';
					wiredMSG_info[3] = 'W';
				}else if(from_layer == TXL_FROM){
					wiredMSG_info[0] = 't';
					wiredMSG_info[1] = 'x';
					wiredMSG_info[2] = 'l';
					wiredMSG_info[3] = 'W';
				}
				break;
			default:
				fprintf(stderr,"error TAVRfile::init_file ..  ERROR...");
				exit(1);
			}
		}else{
			if(from_layer == APP_FROM)appLayer_info[i] = file_name[i];
			else if(from_layer == RTR_FROM)rtrLayer_info[i] = file_name[i];
			else if(from_layer == TXL_FROM)txlLayer_info[i] = file_name[i];

			if(from_hello == HELLO_FROM)helloMSG_info[i] = file_name[i];
			if(from_wired == WIRED_FROM)wiredMSG_info[i] = file_name[i];
		}
	}
	switch(from_layer){
	case APP_FROM:
		appLayer_info[i] = '.';
		appLayer_info[i+1] = 't';
		appLayer_info[i+2] = 'x';
		appLayer_info[i+3] = 't';
		appLayer_info[i+4] = '\0';
		appLayer_info[i+5] = '\0';break;
	case RTR_FROM:
		rtrLayer_info[i] = '.';
		rtrLayer_info[i+1] = 't';
		rtrLayer_info[i+2] = 'x';
		rtrLayer_info[i+3] = 't';
		rtrLayer_info[i+4] = '\0';
		rtrLayer_info[i+5] = '\0';break;
	case TXL_FROM:
		txlLayer_info[i] = '.';
		txlLayer_info[i+1] = 't';
		txlLayer_info[i+2] = 'x';
		txlLayer_info[i+3] = 't';
		txlLayer_info[i+4] = '\0';
		txlLayer_info[i+5] = '\0';break;
	default:
		fprintf(stderr,"error TAVRfile::clean_file ..invalid file source  ERROR...");
		exit(1);
	}

	switch(from_hello*10+from_wired){
	case 0:
		break;
	case WIRED_FROM:
		wiredMSG_info[i] = '.';
		wiredMSG_info[i+1] = 't';
		wiredMSG_info[i+2] = 'x';
		wiredMSG_info[i+3] = 't';
		wiredMSG_info[i+4] = '\0';
		wiredMSG_info[i+5] = '\0';
		break;
	case 10:
		helloMSG_info[i] = '.';
		helloMSG_info[i+1] = 't';
		helloMSG_info[i+2] = 'x';
		helloMSG_info[i+3] = 't';
		helloMSG_info[i+4] = '\0';
		helloMSG_info[i+5] = '\0';
		break;
	case 11:
		helloMSG_info[i] = '.';
		helloMSG_info[i+1] = 't';
		helloMSG_info[i+2] = 'x';
		helloMSG_info[i+3] = 't';
		helloMSG_info[i+4] = '\0';
		helloMSG_info[i+5] = '\0';

		wiredMSG_info[i] = '.';
		wiredMSG_info[i+1] = 't';
		wiredMSG_info[i+2] = 'x';
		wiredMSG_info[i+3] = 't';
		wiredMSG_info[i+4] = '\0';
		wiredMSG_info[i+5] = '\0';
		break;
	default:
		fprintf(stderr,"error TAVRfile::clean_file ....invalid file source   ERROR...");
		exit(1);
	}


	fprintf(stdout,"\n===========\nfile list %s %s %s %s %s\n",appLayer_info,txlLayer_info,rtrLayer_info,helloMSG_info,wiredMSG_info);

	switch(from_layer){
	case APP_FROM:
		if(fopen(appLayer_info,"r") != NULL) remove(appLayer_info);break;
	case RTR_FROM:
		if(fopen(rtrLayer_info,"r") != NULL) remove(rtrLayer_info);break;
	case TXL_FROM:
		if(fopen(txlLayer_info,"r") != NULL) remove(txlLayer_info);break;
	default:
		fprintf(stderr,"error TAVRfile::clean_file ..invalid file source  ERROR...");
		exit(1);
	}

	switch(from_hello*10+from_wired){
	case 0:
		break;
	case WIRED_FROM:
		if(fopen(wiredMSG_info,"r") != NULL) remove(wiredMSG_info);
		break;
	case 10:
		if(fopen(helloMSG_info,"r") != NULL) remove(helloMSG_info);break;
	case 11:
		if(fopen(helloMSG_info,"r") != NULL) remove(helloMSG_info);
		if(fopen(wiredMSG_info,"r") != NULL) remove(wiredMSG_info);
		break;
	default:
		fprintf(stderr,"error TAVRfile::clean_file ....invalid file source   ERROR...");
		exit(1);
	}



}

void
TAVRfile::close(){

	switch(from_layer){
	case APP_FROM:
		if(appFile == NULL){
			fprintf(stderr,"error TAVRfile::close ..app_layer.txt create ERROR...");
			exit(1);
		}
		fclose(appFile);break;

	case RTR_FROM:
		if(txlFile == NULL){
			fprintf(stderr,"error TAVRfile::close ..txl_layer.txt create ERROR...");
			exit(1);
		}
		fclose(txlFile);break;

	case TXL_FROM:
		if(rtrFile == NULL){
			fprintf(stderr,"error TAVRfile::close ..rtr_layer.txt create ERROR...");
			exit(1);
		}
		fclose(rtrFile);break;
	default:
		fprintf(stderr,"error TAVRfile::close ..  ERROR...");
		exit(1);
	}

	switch(from_hello*10+from_wired){
	case 0:
		break;
	case WIRED_FROM:
		if(wiredFile == NULL){
			fprintf(stderr,"error TAVRfile::close ..wired_info.txt create ERROR...");
			exit(1);
		}
		fclose(wiredFile);break;
	case 10:
		if(helloFile == NULL){
			fprintf(stderr,"error TAVRfile::close ..hello_info.txt create ERROR...");
			exit(1);
		}
		fclose(helloFile);break;
	case 11:
		if(helloFile == NULL){
			fprintf(stderr,"error TAVRfile::close ..hello_info.txt create ERROR...");
			exit(1);
		}
		fclose(helloFile);
		if(wiredFile == NULL){
			fprintf(stderr,"error TAVRfile::close ..wired_info.txt create ERROR...");
			exit(1);
		}
		fclose(wiredFile);break;
	default:
		fprintf(stderr,"error TAVRfile::init_file ..  ERROR...");
		exit(1);
	}


}

void
TAVRfile::create_app(){
	appFile = fopen(appLayer_info,"a+");
}

void
TAVRfile::create_txl(){
	txlFile = fopen(txlLayer_info,"a+");
}

void
TAVRfile::create_rtr(){
	rtrFile = fopen(rtrLayer_info,"a+");
}

void
TAVRfile::create_hello(){
	helloFile = fopen(helloMSG_info,"a+");
}

void
TAVRfile::create_wired(){
	wiredFile = fopen(wiredMSG_info,"a+");
}
