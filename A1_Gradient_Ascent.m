function plot = A1_Gradient_Ascent_Final(sx, sy, sz, gx, gy, gz)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Initialize Variables
    % numNodes is the number of maximum iterations
    %
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    numNodes = 500;        
    q_start.coord = [sx, sy, sz];
    q_start.cost = 0;
    q_start.parent = 0;
    q_new.coord = [sx+0.1 sy+0.1 sz+0.1];
    q_new.cost = 0;
    q_rand.coord = [sx+1 sy+1 sz+1];
    q_rand.cost = 0;
    dist = 0;
    
    readings = table();
    figure(1)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Print the graph of the plume. This will take 2 mins. Start the drone after
    %this is done.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    A1_map_condentration(gx,gy,gz);
    hold on
    t= title('');
    t.FontSize =16;
    q_new = q_start;
    q_rand = q_new;
    flag=false;
    xlabel('X'); 
    ylabel('Y'); 
    zlabel('Z'); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     for i = 1:1:numNodes
%          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             q_randX.coord = [0.5+q_new.coord(1) q_new.coord(2) q_new.coord(3)];
%             plot3(q_randX.coord(1), q_randX.coord(2), q_randX.coord(3), 'x', 'Color',  [0 0.4470 0.7410])
%             hold on
%             x = q_randX.coord(1);
%             y = q_randX.coord(2);     
%             z = q_randX.coord(3);
%             q_randX.cost = A1_gaussian_field(gx, gy, gz, x, y, z);
%             q_randY.coord = [q_new.coord(1) 0.5+q_new.coord(2) q_new.coord(3)];
%             plot3(q_randY.coord(1), q_randY.coord(2), q_randY.coord(3), 'x', 'Color',  [0 0.4470 0.7410]);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             x = q_randY.coord(1);
%             y = q_randY.coord(2);     
%             z = q_randY.coord(3);
%             q_randY.cost = A1_gaussian_field(gx, gy, gz, x, y, z);   
%             q_randZ.coord = [q_new.coord(1) q_new.coord(2) 0.5+q_new.coord(3)];
%             plot3(q_randZ.coord(1), q_randZ.coord(2), q_randZ.coord(3), 'x', 'Color',  [0 0.4470 0.7410]);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%             x = q_randZ.coord(1);
%             y = q_randZ.coord(2);     
%             z = q_randZ.coord(3);
%             q_randZ.cost = A1_gaussian_field(gx, gy, gz, x, y, z);   
% 
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%             xfactor = q_randX.cost-q_new.cost ;
%             yfactor = q_randY.cost-q_new.cost ;
%             zfactor = q_randZ.cost-q_new.cost ;
%             sumfactor = abs(xfactor+yfactor+zfactor);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%             if (xfactor ==0)&&(yfactor==0)&&(zfactor==0)
%                 q_near.coord = [q_new.coord(1)+0.1 q_new.coord(2)+0.1 q_new.coord(3)+0.1];
%             else
%                 q_near.coord = [q_new.coord(1)+0.1*(xfactor/sumfactor) q_new.coord(2)+0.1*(yfactor/sumfactor) q_new.coord(3)+0.1*(zfactor/sumfactor)];
%             end    
%             q_near.cost = A1_gaussian_field(gx, gy, gz,q_near.coord(1), q_near.coord(2), q_near.coord(3)); 
%             %move(droneObj,[-1*(xfactor/sumfactor), -1*(yfactor/sumfactor), -1*(zfactor/sumfactor)],'WaitUntilDone',false);
%             line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'k', 'LineWidth', 2);
%             dist = dist + norm(q_near.coord-q_new.coord);
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%             cellreadings = {q_near.coord(1),q_near.coord(2),q_near.coord(3),q_near.cost};
%             readings =[readings;cellreadings];
% 
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         
%             if (q_near.cost >=   q_new.cost)
%                 q_new = q_near;
%             else
%                 if (q_near.cost ==0)
%                     q_new = q_near;
%                 else    
%                 %found the min
%                     break;
%                 end
%             end
%             drawnow
%             hold on
%     end
%     dist
%         text1 = ['Number of Steps to Destination ',num2str(i)];
%         text2 = ['Final Coodinate: ' ,'X=',num2str(q_new.coord(1)), ' Y=',num2str(q_new.coord(2)), ' Z=', num2str(q_new.coord(3))];
%         text3 = ['AQI Max Detected: ' num2str(q_new.cost)];
%         text4 = ['Start Coordinate: ' ,'X=',num2str(q_start.coord(1)), ' Y=',num2str(q_start.coord(2)), ' Z=', num2str(q_start.coord(3))];
%         text5 = ['Time Taken: '];
%         text6 = ['Algorithm: GA']
%         text7 = ['Actual End Points: ', 'X=',num2str(gx), ' Y=',num2str(gy), ' Z=', num2str(gz)]
%         subtitle({text1,text4,text2,text3, text4, text7, ['\bf', text6]});
%         plot = plot3(q_new.coord(1), q_new.coord(2), q_new.coord(3), 'O', 'Color',  [1 0 0]);
% end


