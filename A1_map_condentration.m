function A1_map_condentration(x0, y0, z0)
readings = table();
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_factor = 2
for x = 0:0.1:5
     for y =0:0.1:5
           for z =0:0.1:5
                % x, y, z: Coordinates where concentration is to be calculated
                % x0, y0, z0: Source location
                % u: Wind speed
                % s_y: Horizontal dispersion parameter (standard deviation in y-direction)
                % s_z: Vertical dispersion parameter (standard deviation in z-direction)
            
                % Calculate Gaussian plume
                % Define parameters
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % Take care of divide by zero
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                concentration = A1_gaussian_field(x0, y0, z0, x,y,z);
                %xs =0 ; ys = 3; xz=3;
                %concentration = gauss_func(10,1,20,x,y,z,xs,ys,1,1,1,4);
                ii = find(isnan(concentration) | isinf(concentration));
                  concentration(ii) = 0;  
                cellreadings = {x,y,z,concentration};
                readings =[readings;cellreadings];
            end
     end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Take care of divide by zero
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
readings(~readings.Var4,:) = [];
s=scatter3(readings,'Var1','Var2','Var3','filled');
s.AlphaData = readings.Var4;
s.MarkerFaceAlpha = 'flat';
xlabel('X coordinate of the area in consideration'); 
ylabel('Y coordinate of the area in consideration'); 
zlabel('Z coordinate of the area in consideration'); 
end
%end
