
awk '
BEGIN{}

{
i=index($0,"vehicle=");
if(i>0)
	{		

		print  >> "Routinecheck.txt";
	}
}

END{}' dataTMPP01.txt
