/*
 * tavr_traffic_pkt.cc
 *
 *  Created on: Mar 16, 2018
 *      Author: js
 */

#include <packet.h>
#include "tavr_traffic_pkt.h"

int hdr_route_tavr::offset_;
int hdr_app_tavr::offset_;
int hdr_veh_hello::offset_;
int hdr_wired_infos::offset_;

//OTcl linkage for packet headers
static class TAVRHELLOHeaderClass : public PacketHeaderClass {
public:
	TAVRHELLOHeaderClass() : PacketHeaderClass("PacketHeader/TAVRHELLO",
                                              sizeof(hdr_veh_hello)) {
	  bind_offset(&hdr_veh_hello::offset_);
	}
} class_tavr_hello_hdr;

static class TAVRWIREDHeaderClass : public PacketHeaderClass {
public:
	TAVRWIREDHeaderClass() : PacketHeaderClass("PacketHeader/TAVRWIRED",
                                              sizeof(hdr_wired_infos)) {
	  bind_offset(&hdr_wired_infos::offset_);
	}
} class_tavr_wired_hdr;


static class TAVRAPPHeaderClass : public PacketHeaderClass {
public:
	TAVRAPPHeaderClass() : PacketHeaderClass("PacketHeader/TAVRAPP",
                                              sizeof(hdr_app_tavr)) {
	  bind_offset(&hdr_app_tavr::offset_);
	}
} class_tavr_app_hdr;

static class SIMUTAVRHeaderClass : public PacketHeaderClass {
public:
	SIMUTAVRHeaderClass() : PacketHeaderClass("PacketHeader/SIMUTAVR",
                                              sizeof(hdr_route_tavr)) {
	  bind_offset(&hdr_route_tavr::offset_);
	}
} class_tavr_route_hdr;


