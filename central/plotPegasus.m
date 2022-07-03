close all
clc

load('points3d1.csv')
ext1 = [0.169405740049064,-0.555621036317241,2.873008239498214];
ext2 = [0.707024233421587,-0.554006815225048,3.325344810895880];
init =  points3d1(1,1:3);
newPts = points3d1(:,1:3)-init;
ext1 = ext1-init;
ext2 = ext2-init;
v = ext1;

k = [1,0,0];
cosPhi = dot(v,k)/norm(v);
sinPhi = sqrt(1-cosPhi^2);
u = (cross(v,k)/norm(cross(v,k)));
u1 = u(1);
u2 = u(2);
u3 = u(3);
R = [cosPhi+u1*u1*(1-cosPhi),u1*u2*(1-cosPhi)-u3*sinPhi,u2*sinPhi+u1*u3*(1-cosPhi);
     u1*u2*(1-cosPhi)+u3*sinPhi,cosPhi+u2*u2*(1-cosPhi),u2*u3*(1-cosPhi)-u1*sinPhi;
     u1*u3*(1-cosPhi)-u2*sinPhi,u2*u3*(1-cosPhi)+u1*sinPhi,cosPhi+u3*u3*(1-cosPhi)];
newPts = (R*newPts')';
ext1 = (R*(ext1)')';
ext2 = (R*(ext2)')';

beta = -atan(-1.06/14.96);
R = roty(45+rad2deg(beta)/2);
newPts = (R*newPts')'+[0,0,14*0.0254];
ext1 = (R*(ext1)')'+[0,0,14*0.0254];
ext2 = (R*(ext2)')'+[0,0,14*0.0254];

beta = atan(ext2(2)/norm(ext2))/2;
R = rotz(rad2deg(beta));
newPts = (R*newPts')'-[0,0,14*0.0254];
ext1 = (R*(ext1)')'-[0,0,14*0.0254];
ext2 = (R*(ext2)')'-[0,0,14*0.0254];

beta = -atan(ext1(2)/norm(ext1));
R = rotx(rad2deg(beta)-1);
newPts = (R*newPts')'+[0,0,14*0.0254];
ext1 = (R*(ext1)')'+[0,0,14*0.0254];
ext2 = (R*(ext2)')'+[0,0,14*0.0254];

scatter3(newPts(:,1),newPts(:,3),-newPts(:,2),'b');
hold on
scatter3(0,0,0,linewidth=4);
%scatter3(ext1(:,1),ext1(:,3),-ext1(:,2),linewidth=4);
%scatter3(ext2(:,1),ext2(:,3),-ext2(:,2),linewidth=4);
axis('equal');
view(0,90);
zlabel('y')
ylabel('z')
zlim([-0.1,0.3]);
xlim([-0.4,0.4]);
ylim([-0.1,0.4]);

% pegasus trajectory
%figure()
%hold on
r1 = 14*0.0254;
angle0 = linspace(pi/2,atan(-1.06/14.96), 534);
arc0 = [r1*cos(angle0);0*(angle0);r1*sin(angle0)]';
scatter3(arc0(:,1),arc0(:,3),-arc0(:,2),'r',linewidth=4);

r1 = 14*0.0254;
alpha = atan(-0.53/-14.99);
angle1 = linspace(atan(-1.06/14.96), pi+atan(-0.53/-14.99),1038);
arc1 = [r1*cos(angle1);0*(angle1);r1*sin(angle1)]';
scatter3(arc1(:,1),arc1(:,3),-arc1(:,2),'r',linewidth=4);

center=[0,0,0];
r2 = 9*0.0254;
% r = 9*0.0254;
% p1=[-8.9944;-0.318;0]*0.0254;
% p2=[-1.2841;-0.047;-8.9]*0.0254;
% x=linspace(p1(1),p2(1),418);
% y=linspace(p1(2),p2(2),418);
% z=real(center(3)+sqrt((r^2)-((x-center(1)).^2)-((y-center(2)).^2)));
% arc2 = [x-5*0.0254;-z;y]';
% scatter3(arc2(:,1),arc2(:,3),-arc2(:,2),'r',linewidth=4), grid on, hold on

angle2 = linspace(pi, pi-asin(8.9/9),413);
arc2 = [r2*cos(angle2)-5*0.0254;-r2*sin(angle2);0*(angle2)]';
scatter3(arc2(:,1),arc2(:,3),-arc2(:,2),'r',linewidth=2);

grid on
axis('equal');
view(-0,90);
zlim([-0.1,0.3]);
xlim([-0.4,0.4]);
ylim([-0.1,0.4]);
xlabel('x')
ylabel('z')
zlabel('y')


% complete trajectory
%first: 577
%ext1 idx: 1111
%ext2 idx: 2251
%final: 2766

k=2;
optitrack = newPts(577:2766,k)';

figure()
plot(optitrack,'b-s')
hold on
plot(0:533,arc0(:,k), 'r-s')
plot(636:1673,arc1(:,k), 'c-s')
plot(1776:2188,arc2(:,k), 'm-s')
figure()
hold on
plot(0:533,arc0(:,k)'-optitrack(1:534), 'r-s')
plot(636:1673,arc1(:,k)'-optitrack(637:1674), 'c-s')
plot(1776:2188,arc2(:,k)'-optitrack(1777:2189), 'm-s')


