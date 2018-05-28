filename=$1

awk '
BEGIN{i=0}

{

if($0 > i)
	{		
		print "======="+$0;
		i=$0;
	}
}

END{print i+1}' numStr.txt
