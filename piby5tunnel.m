clc
clear
close all

x_0 = 30; %start position of vehicle and tunnel
y_0 = 0; %start position of vehicle and tunnel
z_0 = 0; %start position of vehicle
slice = 20; %how many slices of the tunnel is wanted
sides = 8; %how many sides for each slice => gives more normal vectors
Position = [x_0 y_0 z_0]; %start position of vehicle
vehicle_velocity = 1;
initial_velocity_direction = [0 1 0];
initial_normal_vector = [1 0 0];
T_s = 0.1;

boundary = 0.1;
Ud = 1.485;
us = 1;
ubar = 1.5;
Vd = 1; %control parameter - rate of change of distance function
dtr = 7.3; %memory distance when switching (get it from single_d scope)
d_star = 0.6; %safety distance to wall
sigma = 0.005; %boundary of xi function
const_v = 0.5;
delta_arrow = 0.0295;  %when |d_distance| <= dtr, control law switches
mu = 0.1;
% mu_s = mu + delta_arrow < vehicle_velocity
% delta_arrow < mu
% mu*(mu_s+Vd)+Ud*(vehicle_velocity-sqrt(vehicle_velocity^2-mu_s^2)) < 

radius_circle = 8; %radius of circle
circle  = circleToPolygon2([x_0 y_0 radius_circle], sides); 
%circle contains coordinates of points that are connected
[x, y, t] = revolutionSurface(circle, [0 0 0 1], linspace(0, pi/2, slice)); 
%[x,y,x] = .. (circle = takes the polygon that is to be revoluted
%[x,y,x] = .. (~, [] = revolution axis [x0, y0, dx, dy] where x0 and y0 are
%origin of the line and dx, dy is direction vector of line
%[x,y,x] = .. (~, ~, linespace (from, increment, to) = 
figure(1);
h1=surf(x, y, t);
h1.EdgeColor = [0 0 0];
h1.FaceColor = [0 0 1];
alpha 0.3;
hold on
plot3(Position(1,1),Position(1,2),Position(1,3),'.','Color','r');
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-10 40])
ylim([-10 40])
zlim([-25 25])
hold on
co=1;
P=zeros(sides*slice,3);% all the points of the tunnels
for k=1:slice % which plane you want to access
    for i=1:sides % which point you want to access
        point =  [x(i,k) y(i,k) t(i,k)];
         P(co,:)=point;
        co=co+1;
    end
end
P_cs =zeros(sides*slice,3); % center points of the sides-plane
V_p=zeros(sides*slice,3); % edges  perpendicular to direction of tunnel 
V_q=zeros(sides*(slice-1),3); %edges in direction of tunnel
N_n =zeros(sides*(slice-1),3); %normal of planes (side)
co2=1; %for loop
co5=1; %for loop
for q=1:sides % which slice you want to start for example when it is one it will give center points of the planes between plane 1 and 2
    for m=1:slice % center of the plane       
        if q<sides
            if m<slice
                sum = P(((m-1)*sides +q),:)+P(((m-1)*sides +q+1),:)+P((m*sides +q),:)+P((m*sides +q+1),:);
            end
            v_p =  P((((m-1)*sides)+q+1),:)-P((((m-1)*sides)+q),:);
        end%vector from q to q+1
        if q==sides
            if m<slice
                sum = P(((m-1)*sides +1),:)+P((m*sides),:)+P(((m+1)*sides),:)+P((m*sides +1),:);
            end
            v_p =  P(m*sides,:)-P((((m-1)*sides)+1),:);
        end
        if m<slice
            v_q = P(((m*sides)+q),:)-P((((m-1)*sides)+q),:);
            V_q(co5,:)=v_q;
            co5=co5+1;
        end
        P_cs(co2,:) = sum;
        V_p(co2,:)=v_p;
        P_c= P_cs/4;
        co2=co2+1;
    end
end
%cutting out not required data points from the V_p and P_c_pp at end of
%tunnel
V_p_f =zeros(sides,3);
V_p_pp =zeros(sides*(slice-1),3); % important one
P_c_pp =zeros(sides*(slice-1),3); % important one
cons10=1;
cons11=1;
for i=1:length(V_p)
    b=mod(i,slice);
    if b==0
        V_p_f(cons10,:)=V_p(i,:);
        cons10=cons10+1;
    else
        V_p_pp(cons11,:) =V_p(i,:);
        P_c_pp(cons11,:) =P_c(i,:);
        cons11=cons11+1;
    end
end

for var=1:length(V_p_pp)
    if(var>(sides-1)*(slice-1))
        N_normal=normalize(-cross( V_p_pp(var,:), V_q(var,:)));
    else
        N_normal=normalize(cross( V_p_pp(var,:), V_q(var,:)));
    end 
    N_n(var,:)=N_normal; %normal to plane (sides)
    %quiver3(P_c_pp(var,1),P_c_pp(var,2), P_c_pp(var,3),N_normal(1,1),N_normal(1,2),N_normal(1,3));
    hold on
end


sum_ovf=zeros(slice,3);
sum_ov=0;
N_n_p=zeros(slice,3); %normal in direction of tunnel for each slice
for w=1:slice
    for r=1:sides
        sum_ov=sum_ov+P(((w-1)*sides+r),:);
    end
        sum_ova=sum_ov/sides;
        sum_ov=0;
        sum_ovf(w,:)=sum_ova; %center points of slices
      N_normal_p=normalize(-cross( V_p(w,:), V_p(w+slice,:)));
      N_n_p(w,:)=N_normal_p;
      %tells the direction of the motion
      %quiver3(sum_ova(1,1),sum_ova(1,2), sum_ova(1,3),N_normal_p(1,1),N_normal_p(1,2),N_normal_p(1,3))
end

%equation of the planes %commented out as not needed
% digitsOld = digits(5);
% pf1=[];
% pf2=[];
% syms x y z
% P11 = [x,y,z];
% for var2=1:length(V_p_pp)
% planefunction1 = dot(N_n(var2,:), P11-P_c_pp(var2,:));
% ppp1=vpa(planefunction1);
% pf1=[pf1;ppp1];
% end
% for tt=1:length(N_n_p)%%% vertical plane equations
% planefunction2 = dot(N_n_p(tt,:), P11-sum_ovf(tt,:));
%  ppp2=vpa(planefunction2); 
%  pf2=[pf2;ppp2];
% end

%distance from the walls
dis_arr=zeros(length(N_n),1);
for kk=1:length(N_n)
dis=dot(Position-P_c_pp(kk,:),N_n(kk,:));
dis_arr(kk) =dis;
end
%distance from the sections
dis_arrs=zeros(length(N_n_p),1); 
for kk2=1:length(N_n_p)
%to figure out in between which two slices the vehicle is
%distance will be positive if vehicle have passed through
dis2=dot(Position-sum_ovf(kk2,:),N_n_p(kk2,:)); 
dis_arrs(kk2) =dis2;
end

N_nfinal=zeros(slice-1,3*sides);
P_c_final=zeros(slice-1,3*sides);
%distributing the normal vectors into the group 
for ui=1:(length(N_n)/(slice-1))
    r_start=((ui-1)*(slice-1)+1);
    r_endp=(slice-1)*ui;
    c_start=(3*(ui-1)+1);
    c_end=(3*ui);
    N_nfinal(1:(slice-1),c_start:c_end)=N_n(r_start:r_endp,:);
    P_c_final(1:(slice-1),c_start:c_end)=P_c_pp(r_start:r_endp,:);
end
counter=1;
for count=1:length(dis_arrs)
if(dis_arrs(count)>0)
    counter=counter+1;
end
end
slice_num=counter;
bound_dis=zeros(sides,1);
for nn=1:sides
normal_tmp=N_nfinal(slice_num,3*(nn-1)+1: 3*nn);
point_tmp=P_c_final(slice_num,3*(nn-1)+1: 3*nn);
dis3=dot(Position-point_tmp,normal_tmp);
bound_dis(nn)=dis3;
%quiver3(Position(1,1),Position(1,2), Position(1,3),normal_tmp(1,1),normal_tmp(1,2),normal_tmp(1,3))
hold on;
end
%plot3(Position(1,1),Position(1,2),Position(1,3))
