% 2DOF Kinematic Manipulability Ellipse


%=== CODE ===%

%Clean and clear
clc
close all
clear

%--- Style Plot ---%
figure(1);
set(gca,'FontSize',16)
axis equal
hold all
grid on
box on
xlabel('x [m]')
ylabel('y [m]')
leg={};

%--- 2DOF Data 100 kg ---%
m0=100;
mi=10;
[robot,TEE_Ln,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi);

%--- Joint States ---%

%Base position
R0=eye(3);
r0=[0;0;0];

%--- Contour ---%

%Joint limits

n=200;
i=1;
for q1=deg2rad(linspace(-90,90,n));
    
    %Show progress
    clc
    fprintf('q1: %d \n',rad2deg(q1))
    
    j=1;
    for q2=deg2rad(linspace(-135,135,n));
        
        %Manipulator joint variables
        qm=[q1;q2];
        
        %--- Kinematics ---%
        %kinematics
        [RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
        %End-Effector
        TEE=[RL(1:3,1:3,end),rL(1:3,end);zeros(1,3),1]*TEE_Ln;
        xEE(i,j)=TEE(1,4);
        yEE(i,j)=TEE(2,4);
        
        
        %--- Kinematic Manipulability ---%
        [~,km_fixed(i,j),~,km_floating(i,j)]=Kinematic_Manipulability(R0,r0,m0,mi,qm);
        
        %Counter
        j=j+1;
        
    end
    %Counter
    i=i+1;
end

%--- Plot ---%
[cv,ch]=contourf(xEE,yEE,km_fixed,50);
set(ch,'edgecolor','none');
colormap('gray');
[cmin,cmax] = caxis;
caxis([cmin,cmax*1.1]);
colorbar
qm=deg2rad([45;-90]);
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Plot
Man_Plot(R0,r0,base_contour,man_contour,man_contour_end,RL,rL,robot.n_links_joints);


