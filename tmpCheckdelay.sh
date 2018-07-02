
awk '
BEGIN{}

{
i=index($17,"vehicle=");
j=index($15,"black");
if(i<1 && j<1)
	{		

		print  >> "Newtest.nam";
	}
}

END{}' fine.nam
