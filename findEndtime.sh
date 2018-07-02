
rm "NodeEndTime.tcl"
rm "NodeEndTime_info.txt"

awk '
BEGIN{
pre_define_time=100;

tcl_first="$ns_ at ";
tcl_second=" \"$app_r(";
tcl_third=") stoptx\"";
tcl_final="";


tcl_rtrFirst="$ns_ at ";
tcl_rtrsecond=" \"$ragent_(";
tcl_rtrThird=") vehicle_gone false\"";
tcl_rtrFinal="";

}

{
i=index($4,"node_");
if(i>0)
	{		
#		print $4;
		id_node=substr($4,9, length($4)-9);
		Rtime[id_node]=$3;
#		print substr($4,9, length($4)-9) >> "NodeEndTime_info.txt";
	}
}

END{

print Rtime[0];

for(value_time in Rtime)
{
	print "id=",value_time," value=",Rtime[value_time]+2.33 >> "NodeEndTime_info.txt";
	tcl_final=tcl_first""Rtime[value_time]-6.33""tcl_second""value_time""tcl_third;
	print tcl_final  >>"NodeEndTime.tcl";

	tcl_rtrFinal=tcl_rtrFirst""Rtime[value_time]+2.33""tcl_rtrsecond""value_time""tcl_rtrThird;
	print tcl_rtrFinal  >>"NodeEndTime.tcl";
}

}' ns2mobility.tcl
