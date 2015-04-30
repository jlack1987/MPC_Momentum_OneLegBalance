function [A_out,b_out] = add_constraint(A_in,b_in,A_con,b_con)


A_out = zeros(size(A_in,1) + size(A_con,1),size(A_con,2));
b_out = zeros(length(b_in) + length(b_con),1);

A_out = [A_in;A_con];
b_out = [b_in;b_con];

end