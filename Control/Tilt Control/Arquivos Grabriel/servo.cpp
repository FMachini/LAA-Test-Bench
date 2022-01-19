#include "Servo.h"


Servo::Servo(string pwmchip, int exp){
    
    vector<int> value;
    string pwm;


    value.push_back(20000000); // period
    value.push_back(2350000); // initial required duty_cycle to activate ESC
    value.push_back(1); // enable
    
    if (exp==1){
        setValue(exp, root + pwmchip + "export");
        pwm = "pwm1/";
    }
    else if (exp==0)
    {
        setValue(exp, root + pwmchip + "export");
        pwm = "pwm0/";
    }
    

    paths.push_back(period);
    paths.push_back(duty);
    paths.push_back(enable);
    
    for (int i{0}; i<paths.size(); ++i){
       paths.at(i) = root + pwmchip + pwm + paths.at(i);
       setValue(value[i], paths[i]);

    }   
  
    cout<< pwmchip + "engaged" <<endl;
 }

void Servo::setValue(int val1, string path){
   

    ofstream fs;
    fs.open(path.c_str());
    
    if (!fs)
    {
        std::cerr<< "Servo path is not open"<<std::endl;
    }
    fs << val1;
    fs.close();

}

void Servo::setDutyCycle(int duty){
    
     
    this->paths = paths;

    setValue(duty,paths[2]); 
}


void Servo::deactivateESC()
{
    
    this->paths = paths;
    
    vector<int> val;
    val.push_back(0); // enable
    val.push_back(20000000); // period
    val.push_back(0); // initial required duty_cycle to activate ESC
    
    for (int i{0}; i<paths.size(); ++i){
       setValue(val[i],paths[i]); 
    };
    cout<<"ESC deactivated"<<endl;
 }
