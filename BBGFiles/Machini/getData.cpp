#include "getData.h"


std::vector<double> offset_correction()
{
	accx = 19682; accy = 6644; accz = -20448; gyrox = -204; gyroy = -58 108; gyroz = -3.48523;
	    aux[0] = accx - 12196;
		aux[1] = accy - 7100;
		aux[2] = accz + 20594;
		aux[3] = (gyrox - 79)/131;
		aux[4] = (gyroy - 1466)/131;
		aux[5] = (gyroz - 7868)/131;
		
		return aux;
		
}