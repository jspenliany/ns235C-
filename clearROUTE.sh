rm "clearROUTE.txt"


awk '
BEGIN{}

{
i=index($2,"route");
if(i==0)
	{		
		print $0 >> "clearROUTE.txt";
	}
}

END{}' rtrL_tavr.txt
