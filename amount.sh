
awk '
BEGIN{}

{
i=index($0,"node_");
if(i>0)
	{		
		print "=======";
		print $4;
		print length($4);
		print substr($4,9, length($4)-9) >> "numStr.txt";
	}
}

END{}' ns2mobility.tcl
