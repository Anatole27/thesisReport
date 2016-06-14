% System Markov parameters

Y{1} = Y_bar1{1};

for k = 2:l
    temp = 0;
    for i = 2:k
        temp = temp + Y_bar2{i}*Y{k-i+1};
    end
    Y{k} = Y_bar1{k} - temp;
end