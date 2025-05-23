clearvars
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize Variables
% numNodes is the number of maximum iterations
% q_start.coord = [1, 4, 4] is the start coordinate of the drone for
% simulation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dist = 0;
numNodes = 500;        
q_start.coord = [1, 4, 4]; 
q_start.cost = 0;
q_start.parent = 0;
q_new.coord = [0.1 0.1 0.1];
q_new.cost = 0;
q_rand.coord = [1 1 1];
q_rand.cost = 0;
q_rand.parent = 0;
readings = table();
figure(1)
step_gain = 0.2;
%Print the graph of the plume. This will take 2 mins. Start the drone after
%this is done.
A1_map_condentration();
hold on
t= title('Trajectory of the drone and the waypoints checked');
t.FontSize =16;
q_new = q_start;
q_rand = q_new;
flag=false;
xlabel('X'); 
ylabel('Y') ;
zlabel('Z');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:numNodes
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check x-axis direction values
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q_randX.coord = [step_gain+q_new.coord(1) q_new.coord(2) q_new.coord(3)];
    plot3(q_randX.coord(1), q_randX.coord(2), q_randX.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
    hold on
    x = q_randX.coord(1);
    y = q_randX.coord(2);     
    z = q_randX.coord(3);
    q_randX.cost = A1_gaussian_field(x, y, z);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check y-axis direction values
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q_randY.coord = [q_new.coord(1) step_gain+q_new.coord(2) q_new.coord(3)];
    plot3(q_randY.coord(1), q_randY.coord(2), q_randY.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
    x = q_randY.coord(1);
    y = q_randY.coord(2);     
    z = q_randY.coord(3);
    q_randY.cost = A1_gaussian_field(x, y, z);   

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %check z-axis direction values
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q_randZ.coord = [q_new.coord(1) q_new.coord(2) step_gain+q_new.coord(3)];
    plot3(q_randZ.coord(1), q_randZ.coord(2), q_randZ.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
    x = q_randZ.coord(1);
    y = q_randZ.coord(2);     
    z = q_randZ.coord(3);
    q_randZ.cost = A1_gaussian_field(x, y, z);   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Calutate the direction in 3-d plane depending on increments in the
    %concentration reading
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xfactor = q_randX.cost-q_new.cost;
    yfactor = q_randY.cost-q_new.cost;
    zfactor = q_randZ.cost-q_new.cost;
    sumfactor = abs(xfactor+yfactor+zfactor);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %account for zero readings and choose the new point to go to based on
    %propotional values
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (xfactor ==0)&&(yfactor==0)&&(zfactor==0)
        q_near.coord = [q_new.coord(1)+step_gain q_new.coord(2)+step_gain q_new.coord(3)+step_gain]
    else
           q_near.coord = [q_new.coord(1)+step_gain*(xfactor/sumfactor) q_new.coord(2)+0.1*(yfactor/sumfactor) q_new.coord(3)+0.1*(zfactor/sumfactor)]
    end   
    q_near.cost =A1_gaussian_field(q_near.coord(1), q_near.coord(2), q_near.coord(3)); 
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
    text1 = ['Number of steps to destination ',num2str(i)];
    text2 = ['Final Coordinate: ' ,'X=',num2str(q_new.coord(1)), ' Y=',num2str(q_new.coord(2)), ' Z=', num2str(q_new.coord(3))];
    text3 = ['AQI max detected: ' num2str(q_new.cost)];
    text4 = ['Start Coordinate: ' ,'X=',num2str(q_start.coord(1)), ' Y=',num2str(q_start.coord(2)), ' Z=', num2str(q_start.coord(3))];
    subtitle({text1,text4,text2,text3});
    plot3(q_new.coord(1), q_new.coord(2), q_new.coord(3), 'O', 'Color',  [1 0 0]);
   dist


