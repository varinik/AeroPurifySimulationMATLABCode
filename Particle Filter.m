clear all
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define Parameters and initialize Variables and drone
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% droneObj = ryze("TELLO-92DC81");
x = [0, 0, 0];
r_new = x;
x_N = 1;
x_R = 0.001;
T = 100;
N = 8e00;
V = 10;
x_P = zeros(N, 5);
%%A1_map_condentration();
figure(2)
%A1_map_condentration();
dist = 0;
step_gain = 0.2;
sx = 0
sy = 4
sz = 3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Print the graph of the plume. This will take 2 mins. Start the drone after
%this is done.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A1_map_condentration(sx, sy, sz);
hold on
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize particles and takeoff the drone
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:N-1
   x_P(i, :) = [5*rand(1,1), 5*rand(1,1),5*rand(1,1), 0, 0];
   plot3(x_P(i,1),x_P(i,2),x_P(i,3),'.k','markersize',15)
   hold on
end
%DRONE TAKEOFF
% takeoff(droneObj);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Setup start coordinates for the search. Should be 0 0 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
numNodes = 500;
q_start.coord = [0, 0, 0];
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
t= title('Trajectory of the drone and the waypoints checked');
t.FontSize =16;
q_new = q_start;
q_rand = q_new;
flag=false;
xlabel('X');
ylabel('Y') ;
zlabel('Z');
change_q_next = false;
change_more = true;
cluster_more = false;
repeat = 0

for t = 1:T
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %check x-axis direction values
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   q_randX.coord = [step_gain+q_new.coord(1) q_new.coord(2) q_new.coord(3)];
   plot3(q_randX.coord(1), q_randX.coord(2), q_randX.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
   hold on
   x = q_randX.coord(1);
   y = q_randX.coord(2);    
   z = q_randX.coord(3);
   q_randX.cost = A1_gaussian_field(sx, sy, sz, x, y, z);
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %check y-axis direction values
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   q_randY.coord = [q_new.coord(1) step_gain+q_new.coord(2) q_new.coord(3)];
   plot3(q_randY.coord(1), q_randY.coord(2), q_randY.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
   x = q_randY.coord(1);
   y = q_randY.coord(2);    
   z = q_randY.coord(3);
   q_randY.cost = A1_gaussian_field(sx, sy, sz, x, y, z);  
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %check z-axis direction values
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   q_randZ.coord = [q_new.coord(1) q_new.coord(2) step_gain+q_new.coord(3)];
   plot3(q_randZ.coord(1), q_randZ.coord(2), q_randZ.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
   x = q_randZ.coord(1);
   y = q_randZ.coord(2);    
   z = q_randZ.coord(3);
   q_randZ.cost = A1_gaussian_field(sx, sy, sz, x, y, z);  
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %Calutate the direction in 3-d plane depending on increments in the
   %concentration reading
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   xfactor = q_randX.cost-q_new.cost ;
   yfactor = q_randY.cost-q_new.cost ;
   zfactor = q_randZ.cost-q_new.cost ;
   sumfactor = abs(xfactor+yfactor+zfactor);
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %account for zero readings and choose the new point to go to based on
   %propotional values
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   if ((xfactor ==0)&&(yfactor==0)&&(zfactor==0)) || sumfactor < 0.1
      q_near.coord = [q_new.coord(1)+0.5 q_new.coord(2)+0.5 q_new.coord(3)+0.5]
   else
       q_near.coord = [q_new.coord(1)+step_gain*(xfactor/sumfactor) q_new.coord(2)+step_gain*(yfactor/sumfactor) q_new.coord(3)+step_gain*(zfactor/sumfactor)]
   end          
   q_near.cost = A1_gaussian_field(sx, sy, sz, q_near.coord(1), q_near.coord(2), q_near.coord(3));
   %move(droneObj,[-1*(xfactor/sumfactor), -1*(yfactor/sumfactor), -1*(zfactor/sumfactor)],'WaitUntilDone',false);
   line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
   dist = dist + norm(q_near.coord-q_new.coord);
   cellreadings = {q_near.coord(1),q_near.coord(2),q_near.coord(3),q_near.cost};
   readings =[readings;cellreadings];
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %calcuate learning gain. Lower the change in values bigger the gain.
   %Make the step between 0.2 to 1
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   step_gain = 0.2+1/((abs(q_near.cost-q_new.cost) + 1));
   remove = [];
   z_update=[];
   add=[];
   r_new = q_near.coord;
 
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %Start the Particle filter algorithm as soon as a non-zero reading is
   %received. Only run it for one jump
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   p=[];
   if (q_near.cost ~=0 && change_more)
        for j = 1:1:size(x_P,1)
             plot3(x_P(j,1),x_P(j,2),x_P(j,3),'.k','markersize',15,'color',[0 1 1])
             z_update(j) = A1_gaussian_field_guess(x_P(j,1),x_P(j,2),x_P(j,3),r_new(1),r_new(2),r_new(3));
             if (z_update(j) ~= 0)
               q_near_adjusted = q_near.cost;
               z_update_adjusted =z_update(j);
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % adjust the reading to 0.0000001 as the lowest value
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               if q_near.cost < 0.00000001 q_near_adjusted  = 0.00000001; end
               if (z_update(j) < 0.00000001) z_update_adjusted  = 0.00000001; end
               diff = abs(q_near_adjusted/z_update_adjusted)
                else
               diff = 0
             end   
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % Update the particles with weights
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             x_P(j,4) = diff;
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             % Remove the particles that are impossible
             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
             if ((x_P(j,4) > 1.5) ||(x_P(j,4) < 0.3 ))
                remove = [remove; j];
                plot3(x_P(j,1),x_P(j,2),x_P(j,3),'.k','markersize',15,'color',[0.4, 0.8, 0.8])
             else
                 x_P(j,5) = x_P(j,5) + 1;
             end
            
           
        end
        %x_P=sortrows(x_P,5,'descend');
        %p = [x_P(1,:)
        disp(x_P)
        if (length(remove) ~= 0)
           disp("inhere")
           for k = 1:1:size(x_P,1)
                % disp("inhere2")
                if(ismember(remove, k) == false)
                   %x_P(k,4) = 0;
                   disp("inhere3")
                   p = [p;x_P(k,:)];
               end
           end
           if(length(p) ~= 0)
               x_P = p;
               repeat = 0
           else
               repeat = repeat + 1;
           end

           change_q_next = true;
        else
            if (flag_i == true) save_i = t; flag_i=false; end
        end
       % columnSums = sum(x_P,1);
       % for i = 1:size(x_P,1)
       %     x_P(i,4) = x_P(i,4)./columnSums(4);
       % end
       disp(t)
       x_P=sortrows(x_P,5,'descend');
       save_i =0;
       p=[];

       
             
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %If particles have been removed, take the mean of the data points.
       %Go to that point
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       if ((change_q_next && change_more))   
          mn = mean(x_P, 1);
          q_new.coord(1) = mn(1);
          q_new.coord(2) = mn(1,2);
          q_new.coord(3) = mn(1,3);
          line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
          q_new.cost = A1_gaussian_field(sx, sy, sz, q_near.coord(1),q_near.coord(2),q_near.coord(3));
          dist = dist + norm(q_near.coord-q_new.coord);
          change_more = false;
          change_q_next = false;
         
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % Generate 10 particles for each of t0he 10 top choices
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       if (size(x_P,1) < 200)
           if (flag_i == true) save_i = t; flag_i=false;
           end
           p=x_P;
           if (size(x_P,1) <10)
               x_P_index = size(x_P,1);
               f_index = 10;
           else
               x_P_index = floor(size(x_P,1)/2);
               f_index = 2;
           end   
           for (d=1:x_P_index)
               for (f =1:f_index)
                   x_cord =abs(x_P(d,1)+0.025*randn);
                   y_cord = abs(x_P(d,2)+0.025*randn);
                   z_cord = abs(x_P(d,3)+0.025*randn);
                   p = [p;[x_cord,y_cord,z_cord,0,0]];
                   plot3(x_cord,y_cord,z_cord,'.k','markersize',15,'color',[1 0 0])
               end
           end
           x_P =p;
       end
       if(repeat>3)
           break
       end

   end
  
  disp("exit checking")
  hererere = repeat
  if(repeat>3)
      break
  end
   
  if (~change_q_next && ~change_more )
      change_q_next = true;
      change_more=true;
      q_new.coord(1) = mn(1);
      q_new.coord(2) = mn(2);
      q_new.coord(3) = mn(3);
      line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
      change_more = true;
  else
 
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
  end
  drawnow
  hold on
  if(repeat>3)
     break
  end
end
disp("here")
plot3(q_new.coord(1), q_new.coord(2), q_new.coord(3), 'X', 'Color', [0, 1, 0]) 
plot3(q_new.coord(1), q_new.coord(2), q_new.coord(3), '*', ...
      'Color', [1, 0.5, 0], ...
      'MarkerFaceColor', [1, 1, 0], ...
      'MarkerSize', 14, ...
      'LineWidth', 2);
disp("plotted")
text1 = ['Computed Number of steps to destination ',num2str(t)];
text2 = ['Final Coordinate: ' ,'X=',num2str(q_new.coord(1)), ' Y=',num2str(q_new.coord(2)), ' Z=', num2str(q_new.coord(3))];
text3 = ['AQI max detected: ' num2str(A1_gaussian_field(sx, sy, sz, q_new.coord(1),q_new.coord(2),q_new.coord(3)))];
text4 = ['Start Coordinate: ' ,'X=',num2str(q_start.coord(1)), ' Y=',num2str(q_start.coord(2)), ' Z=', num2str(q_start.coord(3))];
%text5 = ['AQI of the green cloud: ' mean(z_update)];
subtitle({text1,text2,text3,text4});
dist
