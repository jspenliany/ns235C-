/*
 * tavr_rtable.h
 *
 *  Created on: Jan 21, 2018
 *      Author: js
 */

#ifndef TAVR_RTABLE_H_
#define TAVR_RTABLE_H_

//========================
//include part
//========================
#include <trace.h>
#include <map>

//=========================
//define part
//=========================
typedef std::map<nsaddr_t, nsaddr_t> rtable_t;

//========================
//class declaration part
//========================
class TAVR_rtable {
private:
	rtable_t	rt_;
public:
	TAVR_rtable();
	void		print(Trace*);
	void		clear();
	void 		rm_entry(nsaddr_t);
	void 		add_entry(nsaddr_t, nsaddr_t);
	nsaddr_t	lookup(nsaddr_t);
	u_int32_t	size();
};



#endif /* TAVR_RTABLE_H_ */
