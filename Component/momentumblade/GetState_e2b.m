function y = GetState_e2b(Input)
Trans = TranMatrix([0 0 0], rad2deg(Input(1:3)));
n = size(Input,1);
for i = 2:floor(n/3)
    y((i-2)*3+1 : (i-1)*3) = Trans.DCMe2b * Input((i-1)*3+1 : i*3);
end

