#include "eqep.h"

int main (int argc, char** argv)
{


    float vf = 0;
    float vi = 0;    
    int i = 0; 

    // Allocate an instane of 
    eQEP eqep1("/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep", eQEP::eQEP_Mode_Absolute);
    
    // Set the unit time period to 100,000,000 ns, or 0.1 seconds
    eqep1.set_period(100000000L);
    
    // Query back the period
    std::cout << "[eQEP - ] Period = " << eqep1.get_period() << " ns" << std::endl;
    
    // Read position indefintely
    vi = eqep1.get_position();

    while(1)
    {
	int i = 0;
	scanf("%d", &i);
	if(i == 0)
	{
        vf = eqep1.get_position();
	std::cout << "[eQEP] Position = " << eqep1.get_position() 
		  << " [o] = " << ((vf-vi)*360/4000)
		  << std::endl;
	}
    }
    
    // Return success
    return 0;
}

