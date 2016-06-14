function w3db = minus3db(sys)

[sig,w] = sigma(sys,logspace(0,1,1000));
i = find(20*log10(sig) > -3,1);
w3db = w(i);
