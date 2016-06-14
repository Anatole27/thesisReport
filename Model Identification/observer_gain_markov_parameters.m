% Observer gain markov parameters

Yo{1} = Y_bar2{1}; % = 0;

for k = 2:l
    temp = 0;
    for i = 2:k-1
        temp = temp + Y_bar2{i}*Yo{k-i+1};
    end
    Yo{k} = Y_bar2{k} - temp;
end