function list = placementSelection(s1,sigma)

list = [];

[n,p] = size(sigma);
for i = 1:n
    sig_i = sigma(i,:);
    [~,index] = sort(sig_i);
    list = [list index(p-s1+1:p)];
end

list = sort(unique(list));