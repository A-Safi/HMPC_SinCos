function [mode,x]=SinCos(ModelStruct_d,x,u)
x_1=0;
if x(1)>=x_1
    mode=1;
    x=ModelStruct_d(1).A*x+ModelStruct_d(1).B*u+ModelStruct_d(1).d;
elseif x(1)<x_1
    mode=2;
    x=ModelStruct_d(2).A*x+ModelStruct_d(2).B*u+ModelStruct_d(2).d;
end