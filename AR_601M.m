   %%
   %definitions of our links' lenghts in metres
   L11 =0.088;
   L12 = 0;
   L2 = 0;
   L3 = 0;
   L4 = 0.280;
   L5 = 0.280;
   L6 = 0;
   L7 = 0.1055;
   %%
   %Using DH parameters
   %L = Link ([Th d a alpha])
   %Th - the angle about the prev z to aligh its x to the new x
   %d - the depth along the previous joint's z axis to the common normal
   %a - is the distance along the rotated x axis (radius of rotation about
   %previous z axis)
   %alpha - angle of rotation about the new x axis to put previous z in its
   %desired orientation
   %if z and z next are parallel then a = 0. d is various
   clear L
   fix = pi/2;
   R_base = rotz(0);
   trplot(R_base,'color','black');
   %%
   %Kinematic scheme of the robot, using DH convention
   q = [0 0 -pi/2 0 0 0 0]; %angles theta_initial
   q0 = [q(1) q(2) q(3)+pi/2 q(4)+pi/4 q(5)-pi/4 q(6) q(7)]; %angles theta
   alpha = [pi -pi/2 -pi/2 0 0 pi/2 0]; %angles alpha(fixed)
   d = [-L12 L2 0 0 0 0 0];
   a = [L11 0 L3 L4 L5 L6 L7];
   DH1 = [q0(1) d(1) a(1) alpha(1)];% DH parameters
   DH2 = [q0(2)	d(2) a(2) alpha(2)];
   DH3 = [q0(3)	d(3) a(3) alpha(3)];
   DH4 = [q0(4)	d(4) a(4) alpha(4)];
   DH5 = [q0(5)	d(5) a(5) alpha(5)];
   DH6 = [q0(6)	d(6) a(6) alpha(6)];
   DH7 = [q0(7)	d(7) a(7) alpha(7)];
   %Creating links via above expressions
   L(1) = Link(DH1);%fixed
   L(2) = Link('revolute',DH2);
   L(3) = Link('revolute',DH3);
   L(4) = Link('revolute',DH4);
   L(5) = Link('revolute',DH5);
   L(6) = Link('revolute',DH6);
   L(7) = Link('revolute',DH7);
   %Create a robot via existing links, which we can see above and show it
   R = SerialLink(L, 'name', 'AR-601M');
   R.plot(q0)
   %%
   
   %Method of solfing FK
   q1 = q0*(180/pi); % converting theta angles from radian to degrees to calculate FK via 
   %DH convention
   alpha1 = alpha*(180/pi); %converting alpha angles
   DH1 = [q1(1)  d(1) a(1)   alpha1(1)];
   DH2 = [q1(2)	d(2) a(2)	alpha1(2)];
   DH3 = [q1(3)	d(3) a(3)	alpha1(3)];
   DH4 = [q1(4)	d(4) a(4)	alpha1(4)];
   DH5 = [q1(5)	d(5) a(5)	alpha1(5)];
   DH6 = [q1(6)	d(6) a(6)	alpha1(6)];
   DH7 = [q1(7)	d(7) a(7)	alpha1(7)]; 
   % Forward Kinematics function that you can see in FK.m
   T = FK(q1, DH1, DH2, DH3, DH4, DH5, DH6,DH7);
   %Exam the Forward Kinematics solution
   Direct = R.fkine(q0);
   %Received results are equal, so it's a right function
   %%
   %IK method
   %Calculating theta4 via geometric approach
   dot_c = [0 0 0.560];
   dot_0 = [0 0 0];
   R = real(sqrt((dot_c(1)-dot_0(1))^2 +(dot_c(2)-dot_0(2))^2 +(dot_c(3)-dot_0(3))^2));
   X = (2*0.280^2-R^2)/(2*0.280^2);
   Y = real(sqrt(1-cos(X)^2)); %take into consideration only real numbers,
   %excluding outliers from workspace
   theta_41 =real(acos(X));
   %%
   