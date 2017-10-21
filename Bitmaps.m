clear ('variables'); close all;

%The "size" of the workspace
%When dealing with a flat plane, this corresponds to the length of a square
%in which the sphere is located at the center
size_workspace = 10;
%The number of pixels used on one side of the square box defining the full
%workspace of the robot
n = 500; %1000; %1100; %1250;
%For the main program, n/2 needs to be an integer
%Thus, n should be even:
if(mod(n,2)); fprintf('ERROR: n is not even'); return; end
%For better readability of the main program the value n is saved to a file
f = fopen('n.txt','w'); fprintf(f,'%i',n); fclose(f);

%The robot (sphere):
tic
% delete('Sphere_robot/*');
rho = n/size_workspace; %Radius of the sphere in pixels
% %Again, for better readability of the main program the value of rho is
% %saved to a file:
% f = fopen('rho.txt','w'); fprintf(f,'%i',rho); fclose(f);
% centerX = n/2;
% centerY = centerX;
% for i=1:(2*rho)
%     z = -rho + i - 1;
%     %fprintf('true radius = %f \t', sqrt(rho*rho - z*z));
%     radius = round(sqrt(rho*rho - z*z));
%     %fprintf('radius = %i \n', radius);
%     d = (5 - radius * 4)/4;
%     x = 0;
%     y = radius;
%     Z = false(n);
%     while(x<=y)
%         Z(centerX + x, centerY + y) = 1;
%         Z(centerX + x, centerY - y) = 1;
%         Z(centerX - x, centerY + y) = 1;
%         Z(centerX - x, centerY - y) = 1;
%         Z(centerX + y, centerY + x) = 1;
%         Z(centerX + y, centerY - x) = 1;
%         Z(centerX - y, centerY + x) = 1;
%         Z(centerX - y, centerY - x) = 1;
%         if(d < 0)
%             d = d + 2 * x + 1;
%         else
%             d = d + 2 * (x - y) + 1;
%             y = y-1;
%         end
%         x = x + 1;
%     end
%     imwrite(Z,['Sphere_robot/' int2str(i) '.png'],'png');
% end
% Z = false(n);
% for i=(2*rho+1):n
%     imwrite(Z,['Sphere_robot/' int2str(i) '.png'],'png');
% end
% clear Z;
% toc
tic
%The surfaces and walls:
%Store the surfaces and walls in parametric equations form
f_functions = {@(s,t)s %Flat surf
               @(s,t)s %Tilted surf x
               @(s,t)s %Tilted surf y
               @(s,t)s %Tilted surf x,y
               @(s,t)s %Sine wave x, surf
               @(s,t)s %Sine wave y, surf
               @(s,t)s %Sine wave x,y, surf
               @(s,t)s %Gaussian dome, surf
               @(s,t)s %Sine wall 1
               @(s,t)s %Sine wall 2
               @(s,t)s %Sine tilted wall 1
               @(s,t)s %Sine tilted wall 2
               @(s,t)s %Tilted wall 1
               @(s,t)s}; %Tilted wall 2 
g_functions = {@(s,t)t %Flat surf
               @(s,t)t %Tilted surf x
               @(s,t)t %Tilted surf y
               @(s,t)t %Tilted surf x,y
               @(s,t)t %Sine wave x, surf
               @(s,t)t %Sine wave y, surf
               @(s,t)t %Sine wave x,y, surf
               @(s,t)t %Gaussian dome, surf
               @(s,t)rho*sin(s/rho/4)+7.1*rho %Sine wall 1
               @(s,t)rho*sin(s/rho/4)+2.9*rho %Sine wall 2
               @(s,t)s+2.5*rho+rho*sin(s/rho/2) %Sine tilted wall 1
               @(s,t)s-2.5*rho-rho*sin(s/rho/2) %Sine tilted wall 2
               @(s,t)s+3*rho %Tilted wall 1
               @(s,t)s-3*rho}; %Tilted wall 2
h_functions = {@(s,t)n/2+1+0*s+0*t%Flat surf
               @(s,t)s %Tilted surf x
               @(s,t)t %Tilted surf y
               @(s,t)s/2+t/2 %Tilted surf x,y
               @(s,t)rho*sin(s/rho/4)+rho+1 %Sine wave x, surf
               @(s,t)rho*sin(t/rho/4)+rho+1 %Sine wave y, surf
               @(s,t)rho*sin((s+t)/rho/8)+rho+1 %Sine wave x,y, surf
               @(s,t)rho*sqrt(rho)*(exp(-((s-n/2).^2+(t-n/2).^2)/rho^3.8))-ceil((rho*(sqrt(rho)-7.8)+abs(rho*(sqrt(rho)-7.8)))/2); %Gaussian dome, surf
               @(s,t)t %Sine wall 1
               @(s,t)t %Sine wall 2
               @(s,t)t %Sine tilted wall 1
               @(s,t)t %Sine tilted wall 2
               @(s,t)t/3.5 %Tilted wall 1
               @(s,t)t/3.5}; %Tilted wall 2
%The names of all the different surfaces and walls
names = {'Flat_Surf'; 'Tilted_x_Surf'; 'Tilted_y_Surf'; 'Tilted_xy_Surf'; ...
    'Sine_x_Surf'; 'Sine_y_Surf'; 'Sine_xy_Surf'; 'Gaussian_dome_Surf'; ...
    'Sine_Top_Wall'; 'Sine_Bot_Wall'; 'Sine_Tilted_Top_Wall'; ...
    'Sine_Tilted_Bot_Wall'; 'Tilted_Top_Wall'; 'Tilted_Bot_Wall'};
num_surf = length(names);
%for k = 1:num_surf
for k=13:14
    tic
    name = names{k};
    %mkdir(name);
    delete([name '/*']);
    %Grab the correct functions
    f = f_functions{k}; g = g_functions{k}; h = h_functions{k}; 
    I = false(n,n,n);      % Stores the surface in 3D matrix form
    s = (1:n); t = (1:n);  % Parametric vectors
    [S,T] = meshgrid(s,t); % Corresponding parametric matrices
    X = f(S,T); Y = g(S,T); Z = h(S,T); % Surface in 3 matrices form
    X(X<1) = 1; Y(Y<1) = 1; Z(Z<1) = 1; % Values must be > 1
    X(X>n) = n; Y(Y>n) = n; Z(Z>n) = n; % Values must be < n+1
    %Reshape the matrices into vector form
    x = round(reshape(X,[1,numel(X)]));
    y = round(reshape(Y,[1,numel(Y)]));
    z = round(reshape(Z,[1,numel(Z)]));
    v = unique([x',y',z'],'rows'); %Remove duplicates
    x = v(:,1); y = v(:,2); z = v(:,3);
    linIndex = sub2ind(size(I), x, y, z); %Transforming into linear index
    I(linIndex) = true; % Store the surface into the 3D matrix
    for i = 1:n
        imwrite(I(:,:,i),[name '/' int2str(i) '.png'], 'png');
    end
    %Save the picture
    handle = figure;set(handle, 'Visible', 'off');
    mesh(round(X),round(Y),round(Z));
    view(150,130); axis equal; box on; colormap cool;
    xlabel('x-axis'); ylabel('y-axis'); zlabel('z-axis');
    saveas(handle,[int2str(k) '.fig']);
    %In order to display the figure, one needs to do the following:
    %openfig('figure.fig', 'Visible');
    toc
end
toc
clear ('variables'); close all;
