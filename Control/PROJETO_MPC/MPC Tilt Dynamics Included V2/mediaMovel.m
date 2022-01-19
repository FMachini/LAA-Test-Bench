function filteredState = mediaMovel(stateVec,nit,k)

sum1=0; sum2=0; sum3=0;

    if nit>k
        for  i = 0:k-1
            sum1 = sum1+stateVec(2,nit-i);
            sum2 = sum2+stateVec(4,nit-i);
            sum3 = sum3+stateVec(6,nit-i);
        end
    end
    
    filteredState = [sum1 sum2 sum3]/k;

end

