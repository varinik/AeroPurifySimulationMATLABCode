
clear all
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Parameters and initialize Variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%set(0,'DefaultFigureWindowStyle','docked')
x = [0, 0, 0];
r_new = x;
x_N = 1;
x_R = 0.001;
T = 500;
N = 1000;
V = 10;
x_P = zeros(N, 4);
map_condentration();
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize particles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:N-1
    x_P(i, :) = [x(1) + 5*rand, x(2) + 5*rand, x(3) + 5*rand, 0];
    plot3(x_P(i,1),x_P(i,2),x_P(i,3),'.k','markersize',15)
    hold on
end
x_P(i+1, :) = [0.6,3.1,2.9, 0];
    plot3(x_P(i+1,1),x_P(i+1,2),x_P(i+1,3),'.k','markersize',35,'color',[0 0 1])
    hold on
   x_P(i+1, :) = [0,3,3, 0];
    plot3(x_P(i+1,1),x_P(i+1,2),x_P(i+1,3),'.k','markersize',35,'color',[0 0 1])
    hold on

%points = [0 0 0 gaussian_field(0,0,0)]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numNodes = 500;
q_start.coord = [0,0,0];
flag_i=true;
q_start.cost = 0;
q_start.parent = 0;
q_new.coord = [0.1 0.1 0.1];
q_new.cost = 0;
q_new.parent = 0;
q_rand.coord = [1 1 1];
q_rand.cost = 0;
q_rand.parent = 0;
readings = table();
figure(1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Print the graph of the plume. This will take 2 mins. Start the drone after
%this is done.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%A1_map_condentration();
hold on
t= title('Trajectory of the drone and the waypoints checked');
t.FontSize =16;
q_new = q_start;
q_rand = q_new;
flag=false;
xlabel('X'); 
ylabel('Y') ;
zlabel('Z');
for t = 1:T
    q_randX.coord = [0.5+q_new.coord(1) q_new.coord(2) q_new.coord(3)];
    plot3(q_randX.coord(1), q_randX.coord(2), q_randX.coord(3), 'x', 'Color',  [0 0.4470 0.7410]);
    hold on
    x = q_randX.coord(1);
    y = q_randX.coord(2);     
    z = q_randX.coord(3);
    q_randX.cost = A1_gaussian_field(x, y, z);
    
    q_randY.coord = [q_new.coord(1) 0.5+q_new.coord(2) q_new.coord(3)];
    plot3(q_randY.coord(1), q_randY.coord(2), q_randY.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
    x = q_randY.coord(1);
    y = q_randY.coord(2);     
    z = q_randY.coord(3);
    q_randY.cost = A1_gaussian_field(x, y, z);   

    q_randZ.coord = [q_new.coord(1) q_new.coord(2) 0.5+q_new.coord(3)];
    plot3(q_randZ.coord(1), q_randZ.coord(2), q_randZ.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
    x = q_randZ.coord(1);
    y = q_randZ.coord(2);     
    z = q_randZ.coord(3);
    q_randZ.cost = A1_gaussian_field(x, y, z);   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xfactor = q_randX.cost-q_new.cost;
    yfactor = q_randY.cost-q_new.cost;
    zfactor = q_randZ.cost-q_new.cost;
    sumfactor = abs(xfactor+yfactor+zfactor);
    q_near.coord = [q_new.coord(1)+0.1*(xfactor/sumfactor) q_new.coord(2)+0.1*(yfactor/sumfactor) q_new.coord(3)+0.1*(zfactor/sumfactor)];
    q_near.cost = A1_gaussian_field(q_near.coord(1), q_near.coord(2), q_near.coord(3)); 
    %move(droneObj,[-1*(xfactor/sumfactor), -1*(yfactor/sumfactor), -1*(zfactor/sumfactor)],'WaitUntilDone',false);
    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
    cellreadings = {q_near.coord(1),q_near.coord(2),q_near.coord(3),q_near.cost};
    readings =[readings;cellreadings];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    remove = [];
    z_update=[];
    add=[];
    r_new = q_near.coord;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (q_near.cost ~=0)
         for j = 1:1:size(x_P,1)
              z_update(j) = A1_gaussian_field_guess(x_P(j,1),x_P(j,2),x_P(j,3),r_new(1),r_new(2),r_new(3));
              if (z_update(j) ~= 0)
                 diff = abs(q_near.cost/z_update(j));
              else 
                 diff = 0.01;
              end    
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              %
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              if ((diff > 5000000) ||(diff < 0.0000005 ))
                 remove = [remove; j];
                 plot3(x_P(j,1),x_P(j,2),x_P(j,3),'.k','markersize',15,'color',[1 1 1]);
              else
                 x_P(j,4) =  diff;
              end
          end
         p = []
         if (length(remove) ~= 0)
             for k = 1:1:size(x_P,1)
                 if(ismember(remove, k) == false)
                    %x_P(k,4) = 0; 
                    p = [p;x_P(k,:)];
                end
             end
            x_P = p;
        end
        p=[]; 
        sortrows(x_P,4);
        save_i =0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Generate 10 particles for each of the 10 top choices
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if (size(x_P,1) < 200)
            if (flag_i == true) save_i = i; flag_i=false; 
            end
            p=x_P;
            if (size(x_P,1) <100) 
                d_max = size(x_P,1)
            else
                d_max = 100

            end
            for (d=1:d_max)
                for (f =1:2)
                    x_cord =x_P(d,1)+0.1*randn;
                    y_cord = x_P(d,2)+0.1*randn;
                    z_cord = x_P(d,3)+0.1*randn;
                    p = [p;[x_cord,y_cord,z_cord,0]];
                    plot3(x_cord,y_cord,z_cord,'.k','markersize',15,'color',[1 0 0])
                end
            end
            x_P =p; 
        end    
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (q_near.cost >=   q_new.cost)
        q_new = q_near;
    else
        if (q_near.cost ==0)
            q_new = q_near;
        else    
            %found the min
            break;
        end
    end
    
    drawnow
    hold on
    
end
    plot3(q_new.coord(1), q_new.coord(2), q_new.coord(3), 'O', 'Color', [1, 0, 0])  
    text1 = ['Computed Number of steps to destination once less than 10 points are potential ',num2str(save_i)];
    %text2 = ['Final Coordinate: ' ,'X=',num2str(q_new.coord(1)), ' Y=',num2str(q_new.coord(2)), ' Z=', num2str(q_new.coord(3))];
    %text3 = ['AQI max detected: ' num2str(q_new.cost)];
    text4 = ['Start Coordinate: ' ,'X=',num2str(q_start.coord(1)), ' Y=',num2str(q_start.coord(2)), ' Z=', num2str(q_start.coord(3))];
    subtitle({text1,text4});
    
    
