#include <iostream>
#include <fstream>

int main()
{
	std::ofstream output("a.pgm");
	output<<"P2"<<std::endl;
	output<<"#"<<std::endl;
	output<<200<<" "<<200<<std::endl;
	output<<255<<std::endl;
	int i=200;
	int j=200;
	while(j--){
	while(i--){
		output<<254<<" ";
	}
	output<<std::endl;
	i = 200;
	}
	output.close();
	return 0;
}
