yalmip('clear')
clear;clear classes;
% Model data
initialization
% MPC data
Q = [1,0;0,1];
R = 1;
N = 3;
des=[0;0];
u = sdpvar(repmat(1,1,N),repmat(1,1,N));
x = sdpvar(repmat(2,1,N+1),repmat(1,1,N+1));
constraints = [];
objective   = 0;

LastSample=50;

for k = 1:N
    objective = objective + norm(Q*(x{k}-des),1) + norm(R*u{k},1);
    Model1 = [x{k+1} == Problem.ModelStruct_d(1).A*x{k} + Problem.ModelStruct_d(1).B*u{k}, x{k}(1) >= 0];
    Model2 = [x{k+1} == Problem.ModelStruct_d(2).A*x{k} + Problem.ModelStruct_d(2).B*u{k}, x{k}(1) <= 0];
    constraints = [constraints, Model1 | Model2, -1 <= u{k}<= 1, -2<=x{k}<=2];
end
objective = objective + norm(Q*(x{N+1}-des),1);
controller = optimizer(constraints, objective , sdpsettings('Solver','cbc','verbose',0),x{1},[u{1};x{2}]);
%%
Out = [0;-1;1];

for k=1:LastSample
    clc
    fprintf('%2.0d%%',k/LastSample*100)
    %% adding Noise
    Out(:,k+1)=controller{Out(2:3,k)+0.05*randn(2,1)};
    
    %% Simulation
    [mode,Out(2:3,k+1)]=SinCos(Problem.ModelStruct_d,Out(2:3,k),Out(1,k+1));
end
%%
figure(1)
plot(Out(2,1:end-1)','--k','linewidth',1);hold on
stairs([1 k],[des(1) des(1)],':k','linewidth',.8)
legend('CHC-$x_1$-Unweighted','CHC-$x_1$-Weighted','MPC-$x_1$','Interpreter','latex','location','southeast')
% saveas(gcf,'x1_Sim','epsc')

figure(2)
plot(Out(3,1:end-1)','--k','linewidth',1);hold on
stairs([1 k],[des(2) des(2)],':k','linewidth',.8)
legend('CHC-$x_2$-Unweighted','CHC-$x_2$-Weighted','MPC-$x_2$','Interpreter','latex','location','southeast')
% saveas(gcf,'x2_Sim','epsc')

figure(3)
stairs(Out(1,:),'--k','linewidth',1); axis tight;hold on
legend('CHC-Uneighted','CHC-Weighted','MPC','Interpreter','latex','location','southeast')
% saveas(gcf,'u_Sim','epsc')

figure(4)
plot(Out(2,1:end-1)',Out(3,1:end-1)','--k', 'LineWidth' , 1.5);axis equal;hold on
xlim([-2 2]);ylim([-2 2]);
plot(0,0,'*g')
legend('Operating Node','Mode 1','Mode 2','CHC-Uneighted','CHC-Weighted','MPC','Destination Node','Interpreter','latex','location','southeast')
% saveas(gcf,'x1x2_SinCos','epsc')