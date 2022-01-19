#include "motor.h"


Motor::Motor(string pwmchip, int exp){
    
    vector<int> value;
    string pwm;
    value.push_back(1); // enable
    value.push_back(3000000); // period
    value.push_back(1000000); // initial required duty_cycle to activate ESC
    
    if (exp==1){
        setValue(exp, root + pwmchip + "export");
        pwm = "pwm1/";
    }
    else if (exp==0)
    {
        setValue(exp, root + pwmchip + "export");
        pwm = "pwm0/";
    }
    
    paths.push_back(enable);
    paths.push_back(period);
    paths.push_back(duty);
    
    for (int i{0}; i<paths.size(); ++i){
       paths.at(i) = root + pwmchip + pwm + paths.at(i);
       setValue(value[i], paths[i]);
       //cout<<paths.at(i)<<" " <<value[i]<<endl;
    }   
  
    cout<< pwmchip + "engaged" <<endl;
 }

void Motor::setValue(int val1, string path){
   

    ofstream fs;
    fs.open(path.c_str());
    
    if (!fs)
    {
        std::cerr<< "Motor path is not open"<<std::endl;
    }
    fs << val1;
    fs.close();

}

void Motor::setDutyCycle(int duty){
    
     
    this->paths = paths;

    setValue(duty,paths[2]); 
}


void Motor::deactivateESC()
{
    
    this->paths = paths;
    
    vector<int> val;
    val.push_back(0); // enable
    val.push_back(3000000); // period
    val.push_back(0); // initial required duty_cycle to activate ESC
    
    for (int i{0}; i<paths.size(); ++i){
       setValue(val[i],paths[i]); 
    };
    cout<<"ESC deactivated"<<endl;
 }
