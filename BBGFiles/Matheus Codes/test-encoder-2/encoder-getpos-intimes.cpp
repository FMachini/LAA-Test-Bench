#include "eqep.h"
#include <math.h>

#define PI 3.14159265
#define L 0.620 //m
#define DT 0.02 //s

int main (int argc, char** argv)
{

/*
    float vfV = 0, vfH = 0;
    float viV = 0, viH = 0;    
    int i = 0; 
*/

    double eVertical_p0, eVertical_p1, eVertical_Ang, eHorizontal_p0, eHorizontal_p1, eHorizontal_Ang;
    double x, y, z;
    //para calculo de velocidade
    double x_old = 0, y_old = 0, z_old = 0;
    double vX, vY, vZ;

/*	
    // Allocate an instane of 
    eQEP eqep1("/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep", eQEP::eQEP_Mode_Absolute);
*/

    eQEP eqepVertical(eQEP2, eQEP::eQEP_Mode_Absolute);
    eQEP eqepHorizontal(eQEP1, eQEP::eQEP_Mode_Absolute);
    
    // Set the unit time period to 100,000,000 ns, or 0.1 seconds
    eqepVertical.set_period(100000000L);
    eqepHorizontal.set_period(100000000L);
    
    // Query back the period
    std::cout << "[eQEP vertical - ] Period = " << eqepVertical.get_period() << " ns" << std::endl;
    std::cout << "[eQEP horizontal - ] Period = " << eqepHorizontal.get_period() << " ns" << std::endl;

    /*
    // Read position indefintely
    viV = eqepVertical.get_position();
    viH	= eqepHorizontal.get_position();
    */

     //obter posicao ao iniciar sistema
     eVertical_p0 = eqepVertical.get_position();
     eHorizontal_p0 = eqepHorizontal.get_position();
     
	std::cout << "[eQEP vertical - ] Period = " << eqepVertical.get_period() << " ns" << std::endl;
    std::cout << "[eQEP horizontal - ] Period = " << eqepHorizontal.get_period() << " ns" << std::endl;


    while(1)
    {
	int i = 0;
	scanf("%d", &i);
	if(i == 0)
	{
/*
        vfV = eqepVertical.get_position();
        vfH = eqepHorizontal.get_position();
*/

            eVertical_p1 = eqepVertical.get_position();
            eVertical_Ang = ((eVertical_p1-eVertical_p0)*360/4000);
            eVertical_Ang *= -1;	   
	    eHorizontal_p1 = eqepHorizontal.get_position();
            eHorizontal_Ang = ((eHorizontal_p1-eHorizontal_p0)*360/4000);
	    eHorizontal_Ang *= -1;
/*

	std::cout << "[eQEP Vertical] Position = " << eqepVertical.get_position() 
		  << " [o] = " << ((vfV-viV)*360/4000)
		  << std::endl;
	std::cout << "[eQEP Horizontal] Position = " << eqepHorizontal.get_position() 
		  << " [o] = " << ((vfH-viH)*360/4000)
		  << std::endl;
*/

            x = -L * cos(eHorizontal_Ang*(PI/180));
            y = L * sin(eHorizontal_Ang*(PI/180));
            z = L * sin(eVertical_Ang*(PI/180));

            vX = (x - x_old) / DT;
            vY = (y - y_old) / DT;
            vZ = (z - z_old) / DT;

            //reatribuicao de valor/atualizacao
            x_old = x;
            y_old = y;
            z_old = z;

	std::cout << "Encoder vertical:" << std::endl;
	std::cout << "angulo: " << eVertical_Ang << std::endl;
	std::cout << "altura: " << z << std::endl;
	std::cout << "velocidade z: " << vZ << std::endl;
	std::cout << "-------------------------------------" << std::endl;
	std::cout << "Encoder horizontal:" << std::endl;
	std::cout << "angulo: " << eHorizontal_Ang << std::endl;
	std::cout << "x: " << x << std::endl;
	std::cout << "y: " << y << std::endl;
	std::cout << "velocidade x: " << vX << std::endl;
	std::cout << "velocidade y: " << vY << std::endl;
	std::cout << "-------------------------------------" << std::endl;


	}



    }
    
    // Return success
    return 0;
}

