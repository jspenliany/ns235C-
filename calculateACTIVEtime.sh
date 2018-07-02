

rm "NodeActive.tcl"
rm "NodeActive_info.txt"


awk '
BEGIN{

pre_define_time=100;


flag=0;
cID=-1;


Time_Asymptotic_flag=-100;

tcl_first="$ns_ at ";
tcl_second=" \"$app_r(";
tcl_third=") starttx\"";
tcl_final="";


tcl_rtrFirst="$ns_ at ";
tcl_rtrSecond=" \"$ragent_(";
tcl_rtrThird=") vehicle_gone true\"";
tcl_rtrFinal="";

tcl_locFirst="$ns_ at ";
tcl_locSecond=" \"$ragent_(";
tcl_locThird=") send_Rmsg\"";
tcl_locFinal="";

}

{

if(flag==1)
	{
		flag=0;		
		print "cid=",cID," is start at ",$3 >> "NodeActive_info.txt";
		START_time=$3*0.97;
		print pre_define_time""START_time;

		if(START_time < $3 - 3 && $3 - 3 > pre_define_time){
			Time_Asymptotic_flag=100;
		}

		if(Time_Asymptotic_flag < 0){
			START_time=pre_define_time;
		}else{
			START_time=$3-3;
		}


		tcl_final=tcl_first""START_time""tcl_second""cID""tcl_third;
		print tcl_final  >>"NodeActive.tcl";
 
		START_time-=20;
		tcl_rtrFinal=tcl_rtrFirst""START_time""tcl_rtrSecond""cID""tcl_rtrThird;
		print tcl_rtrFinal  >>"NodeActive.tcl";

		START_time-=2.2;
		tcl_locFinal=tcl_locFirst""START_time""tcl_locSecond""cID""tcl_locThird;
		print tcl_locFinal  >>"NodeActive.tcl";

	}

i=index($3,"Z_");
if(i>0)
	{		
		print "=======";
		flag=1;
		print $0;
#		print length($1);
		print substr($1,8, length($1)-8);
		cID=0+substr($1,8, length($1)-8); 
	}
}

END{}' ns2mobility.tcl
