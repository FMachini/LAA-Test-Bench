Ts=0.02;
qts_med=3000;
ch= 1;
IP_BeagleBone = '10.0.33.236';
IP_Local = '10.0.35.97';
PORT = 8080;
DATA_TYPE = 'double';
NUMBER_OF_DATA_2_RCV = 7;

t_obj = tcpclient(IP_BeagleBone, PORT);

tempo_exec=zeros(qts_med);
state_=zeros(qts_med,NUMBER_OF_DATA_2_RCV);
write(t_obj, Ts);
write(t_obj, qts_med);


for i=1:qts_med
t_inicial=tic;
ch= 1;
while ch~=42   
ch=read(t_obj, 1,DATA_TYPE);
end
if ch ==0
    break;
end;
        
state=read(t_obj, NUMBER_OF_DATA_2_RCV, DATA_TYPE);
state_(i,:)= state;    

t_final=toc(t_inicial);
tempo_exec(i) = t_final; 
pause(Ts - t_final);

end
clear t_obj
