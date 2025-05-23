function concentration = A1_gaussian_field_guess(x0, y0, z0, x,y,z )
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % x, y, z: Coordinates where concentration is to be calculated
    % x0, y0, z0: Source location
    % u: Wind speed
    % s_y: Horizontal dispersion parameter (standard deviation in y-direction)
    % s_z: Vertical dispersion parameter (standard deviation in z-direction)
    % Calculate Gaussian plume
    % Define parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    u = 1; % Wind speed
    H =0;
    ay = 0.5;  by = 0.92;  
    s_y = 1*ay*abs(x-x0).^by ;
    az = 0.5; bz = 0.87;  
    s_z = 1*az*abs(x-x0).^bz ;
    %s_y=0.5*(x);
    %s_z=0.5*(x);
    %[s_y,s_z]=calc_sigmas(4,x)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Take care of divide by zero
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     if s_y == 0 s_y = 0.002; end
     if s_z == 0 s_z = 0.002; end   
    %if s_y<1 s_y=1; end
    %if s_z<1 s_z=1; end
    EXP1 = -(y - y0).^2 / ( s_y^2);
    EXP2 = -(z - z0-H).^2 / (2 * s_z^2);
    EXP3 = -(z - z0+H).^2 / (2 * s_z^2);
    concentration = 100*1 / (2 * pi * s_y * s_z*u);
    if (x<x0) concentration = 0; end
    concentration = concentration*exp(EXP1);
    concentration = concentration*(exp(EXP2) +exp(EXP3));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Cap max concentration to 300
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    %
    if concentration >300 concentration = 300; end

end