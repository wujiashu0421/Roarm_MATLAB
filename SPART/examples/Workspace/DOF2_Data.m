function [robot,TEE_Ln,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi)
% Function that resturns the 2DOF manipulator data and base/manipulator
% contours.

%=== CODE ===%

%--- Base Spacecraft ---%
%Base-spacececraft side
base_side = 0.5;

%Base-spacecraft inertia matrix
data.base.mass=m0;
data.base.I=diag(ones(3,1)*data.base.mass/6*(base_side^2));


%Firts joint location with respect to base
data.base.T_L0_J1=[eye(3),[base_side/2;0;0];zeros(1,3),1];

%--- Contours ---%
%Manipulator contour
man_side = 0.5;
man_width = 0.05;
semi_circle = [man_width/2*cos(linspace(pi/2,3*pi/2,100))',man_width/2*sin(linspace(pi/2,3*pi/2,100))'];
man_contour = [semi_circle+repmat([-man_side/2,0],100,1);
               -semi_circle+repmat([-man_side/2,0],100,1);
               -man_side/2,man_width/2;
               man_side/2,man_width/2
               NaN,NaN;                
               -man_side/2,-man_width/2;
               man_side/2,-man_width/2];
           
man_contour_end = [man_contour;
                   man_side/2,man_width/2];
    

%Base-spacecraft contour
base_contour = [base_side/2,base_side/2;
                -base_side/2,base_side/2;
                -base_side/2,-base_side/2;
                base_side/2,-base_side/2;
                base_side/2,-man_width/2;
                flipud(semi_circle)+repmat([+base_side/2,0],100,1);
                 base_side/2,man_width/2;
                 base_side/2,base_side/2];



%--- Manipulator Data ---%               
%Number of joints/links
data.n=2;

%First joint
data.man(1).type=1;
data.man(1).DH.d = 0;
data.man(1).DH.alpha = 0;
data.man(1).DH.a = man_side;
data.man(1).DH.theta=0;
data.man(1).b = [data.man(1).DH.a/2;0;0];
data.man(1).mass=mi;
data.man(1).I=diag([0,0,data.man(1).mass/12*(man_side^2+man_width^2)]);

%Second joint
data.man(2).type=1;
data.man(2).DH.d = 0;
data.man(2).DH.alpha = 0;
data.man(2).DH.a = man_side;
data.man(2).DH.theta=0;
data.man(2).b = [data.man(2).DH.a/2;0;0];
data.man(2).mass=mi;
data.man(2).I=diag([0,0,data.man(2).mass/12*(man_side^2+man_width^2)]);

%End-Effector 
data.EE.theta=0; %Rotation around z-axis
data.EE.d=0; %Translation along z-axis

%--- Create robot structure ---%
[robot,TEE_Ln] = DH_Serial2robot(data);