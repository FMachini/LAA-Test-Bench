function dado = readDataTilt(filename)
global Ts;
global numValues;

txtfile = dlmread(filename);
data = iddata([txtfile(:,3) txtfile(:,4)],[txtfile(:,1) txtfile(:,2)],Ts,'ExperimentName','Identificacao do tilt');
data.OutputName = {'CAD','Angle'};
data.OutputUnit = {'0-1023','degrees'};
data.InputName = {'Time','PWM'};
data.InputUnit = {'seconds','milliseconds'};


col1=[];
col2=[];
col3=[];
col4=[];
col5=[];
col6=[];
col7=[];
col8=[];
col9=[];
col10=[];
col11=[];
col12=[];
col13=[];
col14=[];
col15=[];


for i = 1:length(data.u)
    if(data.u(i,2)==1650)
        col1=[col1;data.y(i,2)];
        
    elseif(data.u(i,2)==1700)
        col2=[col2;data.y(i,2)];
        
    elseif(data.u(i,2)==1750)
        col3=[col3;data.y(i,2)]; 
        
    elseif(data.u(i,2)==1800)
        col4=[col4;data.y(i,2)];
        
    elseif(data.u(i,2)==1850)
        col5=[col5;data.y(i,2)];
        
    elseif(data.u(i,2)==1900)
        col6=[col6;data.y(i,2)];
        
    elseif(data.u(i,2)==1950)
        col7=[col7;data.y(i,2)];
        
    elseif(data.u(i,2)==2000)
        col8=[col8;data.y(i,2)];
        
    elseif(data.u(i,2)==2050)
        col9=[col9;data.y(i,2)];
        
    elseif(data.u(i,2)==2100)
        col10=[col10;data.y(i,2)];
        
    elseif(data.u(i,2)==2150)
        col11=[col11;data.y(i,2)];
        
    elseif(data.u(i,2)==2200)
        col12=[col12;data.y(i,2)];
        
    elseif(data.u(i,2)==2250)
        col13=[col13;data.y(i,2)];
        
    elseif(data.u(i,2)==2300)
        col14=[col14;data.y(i,2)];
        
    elseif(data.u(i,2)==2350)
        col15=[col15;data.y(i,2)];

    end
end


dado = [col1(1:numValues) col2(1:numValues) col3(1:numValues) col4(1:numValues) col5(1:numValues) col6(1:numValues) col7(1:numValues) col8(1:numValues) col9(1:numValues) col10(1:numValues) col11(1:numValues) col12(1:numValues) col13(1:numValues) col14(1:numValues) col15(1:numValues)];

end
