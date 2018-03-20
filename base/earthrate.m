            function omega_n_ie = earthrate(L)
% %===========================================================%
% %           function omega_n_ie = earthrate(L)              %
% %                                                           %
% %   This function returns the Earth rate vector expressed   %
% %   in noth-east-down coordinates.  The input is latitude   %
% %   (L) in units of radians.  The output is in units of     %
% %   radians per second.                                     %
% %                                                           %
% %   Programmer:     Demoz Gebre-Egziabher                   %
% %   Created:        July 2, 1998                            %
% %   Last Modified:  March 26, 2009                          %
% %                                                           %
% %     Copywrite 2009 Demoz Gebre-Egziabher                  %
% %     License: BSD, see bsd.txt for details                 %
% %===========================================================%

%   Load ellipsoid constants

wgs_84_parameters;

%   Compute Earth rate vector
 
omega_n_ie = omega_ie*[cos(L);0;-sin(L)];%%ZXY

%===========================================================%     

