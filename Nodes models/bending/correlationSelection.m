function list = correlationSelection(eps,list,corr,sigma)

p = length(list);
sigma = max(sigma);
I = ones(p,1);

for i = 1:p
    for j = i+1:p
        if corr(list(i),list(j)) > 1-eps
            if sigma(list(i)) < sigma(list(j))
                I(i) = 0;
            else
                I(j) = 0;
            end
        end
    end
end

new_list = [];
for i = 1:p
    if I(i) == 1
        new_list = [new_list, list(i)];
    end
end
list = new_list;