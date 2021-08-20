function [] = CSpace(linkLengths)
%linkLengths: an array of lengths for each links, e.g., CSpace([10 12 14 7]) 
%would create a manipulator with 4 links where links 1, 2, 3, 4 have lengths 10, 12, 14, 7, respectively.

 global sim; %so that other functions have access to it
  nrLinks = length(linkLengths);  %number of links    
  sim.m_thetas = zeros(1, nrLinks); % joint values
  sim.m_lengths = linkLengths; % lengths of links
  sim.m_positionsX = zeros(1, nrLinks + 1); %x-positions of links: link i starts at m_positionsX(i) and ends at m_positionsX(i + 1)
  sim.m_positionsY = zeros(1, nrLinks + 1); %y-positions of links: link i starts at m_positionsY(i) and ends at m_positionsY(i + 1)
 
  sim.m_obstacles = [];  % obstacles represented as rectangles    
  sim.m_obstaclesInRange = []; % obstacles that could possibly intersect with the manipulator. If an obstacle is outside the circle with radius r = sum(sim.m_lengths) then that obstacle cannot intersect with the manipulator. So sim.m_obstaclesInRange keeps track of those obstacles that intersect the circle with radius r since only those obstacles could possibly intersect with the manipulator. This is done to speed up collision checking, which is needed for the computation of c-space. The list should be updated when a new obstacle is added or when a new link is added to the manipulator, a link is removed, or the length of a link is changed.
 
  sim.m_dtheta1 = 0.1; %granularity at which to change theta1 for the c-space computation
  sim.m_dtheta2 = 0.1; %granularity at which to change theta2 for the c-space computation
  sim.m_theta1Range = 0 : sim.m_dtheta1 : 2 * pi; %range of values for theta1 to use for the c-space computation
  sim.m_theta2Range = 0 : sim.m_dtheta2 : 2 * pi; %range of values for theta2 to use for the c-space computation
  sim.m_collision1 = []; %keeps track of pairs of (theta1, theta2) angles that result in collisions
  sim.m_collision2 = []; %with theta1 stored in sim.m_collision1 and theta2 stored in sim.m_collision2
 
  sim.m_rgb = [1 0 0];  %main program uses red to draw the manipulator when not in collision; color is set to black when in collision
  sim.m_action = ActionNone(); %various menu options, see help (by default no option is selected)
  sim.m_selectedLink = 1; %selected link to allow the user to change link angle or length (user can change the selection by choosing an action and then a link number)
  
  
   clf;
   figure(1);
   set(gcf, 'KeyPressFcn', {@MyKeyPressFcn}); %intercept keyboard presses and take appropriate action when user presses a key
 
   Help();
   ForwardKinematics();
   Draw();
end   

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Available Actions
%
function a = ActionNone()  %key associated with that action
 a = 'z';
end
function a = ActionChangeJointAngle() %key associated with that action
  a = 'a';
end
function a = ActionChangeLinkLength() %key associated with that action
  a = 'b';
end
function a = ActionAddLink() %key associated with that action
 a = 'c';
end
function a = ActionRemoveLastLink() %key associated with that action
 a = 'd';
end
function a = ActionAddObstacle() %key associated with that action
 a = 'e';
end
function a = ActionComputeCSpace() %key associated with that action
 a = 'f';
end
function a = ActionHelp() %key associated with that action
a = 'h';
end

function [] = Help() %available actions
  disp(['Press a, b, c, d, e, f, or h to select action:']);
  disp([ActionChangeJointAngle(), '. ChangeJointAngle']);
  disp([' ...then press 1,..,9 to select link']);
  disp([' ...then press + or - to change angle']);   
  disp([ActionChangeLinkLength(), '. ChangeLinkLength']);
  disp([' ...then press 1,..,9 to select link']);
  disp([' ...then press + or - to change link length']);
  disp([ActionAddLink(), '. AddLink']);
  disp([ActionRemoveLastLink(), '. RemoveLastLink']);
  disp([ActionAddObstacle(), '. AddObstacle']);
  disp([ActionComputeCSpace(), '. ComputeCSpace']);
  disp([ActionHelp(), '. DisplayHelpMenu']);     
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Manipulator Functionality
%  - Get link start/end positions
%  - Forward kinematics, i.e., compute the link positions given the angle values
%  - Add/remove links
%  - Check for collisions with obstacles
%  - Update color (red: no collision, black: collision)
%  - Draw manipulator in its current configuration
%
function [x, y] = GetLinkStart(i)
global sim;
x = sim.m_positionsX(i);
y = sim.m_positionsY(i);
end
        
function [x, y] = GetLinkEnd(i)
global sim;
x = sim.m_positionsX(i + 1);
y = sim.m_positionsY(i + 1);
end

function [] = ForwardKinematics()
 global sim;
 
     Mall = [1 0 0; 0 1 0; 0 0 1];
     M    = [1 0 0; 0 1 0; 0 0 1];
     n  = length(sim.m_lengths);
      
         for i = 1 : 1 : n
           ctheta = cos(sim.m_thetas(i));
           stheta = sin(sim.m_thetas(i));

           M(1, 1) = ctheta;  
           M(1, 2) = -stheta; 
           M(1, 3) = sim.m_lengths(i) * ctheta;
	   M(2, 1) = stheta;  
	   M(2, 2) =  ctheta; 
	   M(2, 3) = sim.m_lengths(i) * stheta;
        
           Mall = Mall * M;
       
           sim.m_positionsX(i + 1) = Mall(1, 3);
           sim.m_positionsY(i + 1) = Mall(2, 3);
         end       
end


function [] = AddLink(linkLength)
  global sim;
  sim.m_thetas(end + 1) = 0;
  sim.m_lengths(end + 1) = linkLength;
  sim.m_positionsX(end + 1) = 0.0;
  sim.m_positionsY(end + 1) = 0.0;
end

function [] = RemoveLastLink()
  global sim;
  sim.m_thetas(end) = [];
  sim.m_lengths(end) = [];
  sim.m_positionsX(end) = [];
  sim.m_positionsY(end) = [];
end


function collision = IsInCollision()
global sim;
nrLinks = length(sim.m_lengths);
n = length(sim.m_obstaclesInRange);

for i = 1 : 1 : nrLinks 
  [xs, ys] = GetLinkStart(i);
  [xe, ye] = GetLinkEnd(i);
  for j = 1 : 4 : n
     if SegmentRectangleCollision([xs, ys, xe, ye], [sim.m_obstaclesInRange(j), sim.m_obstaclesInRange(j+1), sim.m_obstaclesInRange(j+2), sim.m_obstaclesInRange(j+3)]) == true
       collision = true;
       return;
     end
  end  	  
end
collision = false;
end

function [] = UpdateColor()
global sim;
  if IsInCollision()
   sim.m_rgb = [0, 0, 0];
  else
   sim.m_rgb = [1, 0, 0];
  end
end
  
function [] = DrawManipulator()
global sim;
  plot(sim.m_positionsX(1:length(sim.m_positionsX) - 1), ...
       sim.m_positionsY(1:length(sim.m_positionsY) - 1), 'o', 'color', sim.m_rgb, 'MarkerSize', 5);
  line(sim.m_positionsX, sim.m_positionsY, 'Color', sim.m_rgb, 'LineWidth', 2);       
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute C-Space
%  first, compute the obstacles that are in range since only those could possibly collide with the manipulator
%  then iterate over joint1 and joint2 values
%  if in collision, mark it
%  - Draw c-space configurations (superimpose manipulator configurations that result in collision)
%  - Draw c-space configurations (plot with joint1 and joint2 values that result in collision)
function [] = ComputeCSpace()
global sim;

ComputeObstaclesInRange();
sim.m_collision1 = [];
sim.m_collision2 = [];
n1 = length(sim.m_theta1Range);
n2 = length(sim.m_theta2Range);
for k1 = 1 : 1 : n1
printf('considering theta1 = %f [%d out of %d values]\n', sim.m_theta1Range(k1), k1, n1);
for k2 = 1 : 1 : n2
  sim.m_thetas(1) = sim.m_theta1Range(k1);
  sim.m_thetas(2) = sim.m_theta2Range(k2);
  ForwardKinematics();
  if IsInCollision() == true
    sim.m_collision1(end + 1) = sim.m_thetas(1);
    sim.m_collision2(end + 1) = sim.m_thetas(2);
    printf('collision found at %f %f\n', sim.m_thetas(1), sim.m_thetas(2));
  end
end
end
Help();
end

%draw joint1 and joint2 angles that result in collisions
function [] = DrawCSpaceObstacles()
  global sim;
   dt1 = sim.m_dtheta1;
   dt2 = sim.m_dtheta2;
   for k = 1 : 1 : length(sim.m_collision1)
      theta1 = sim.m_collision1(k);
      theta2 = sim.m_collision2(k);
      fill([theta1 - 0.5 * dt1, theta1 + 0.5 * dt1, theta1 + 0.5 * dt1, theta1 - 0.5 * dt1], ...
           [theta2 - 0.5 * dt2, theta2 - 0.5 * dt2, theta2 + 0.5 * dt2, theta2 + 0.5 * dt2], [1 0 0]);
   end   
end

%superimpose manipulator configurations that result in collision
function [] = DrawCSpaceCfgs()
  global sim;
  rgb = sim.m_rgb;
  sim.m_rgb = [1, 0, 0];
   for k = 1 : 1 : length(sim.m_collision1)
      sim.m_thetas(1)= sim.m_collision1(k);
      sim.m_thetas(2)= sim.m_collision2(k);
      ForwardKinematics();
      DrawManipulator();
   end   
   sim.m_rgb = rgb;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% General Basic and Obstacle Functionality
%  - Set plot dimensions
%  - Set min/max obstacle (rectangle) size
%  - Generate random rectangles
%  - Compute which obstacles are in range of manipulator
%  - Draw obstacles
%
function a = MinMaxX()
  a = [-100, 100];
end

function a = MinMaxY()
 a = [-100, 100];
end

function a = MinMaxRectangleSize()
 a = [4, 10];
end

function r = RandomNumber(a, b)
  r = rand() * (b - a) + a;
end

function [] = AddRandomRectangle()
  global sim;
  xlims = MinMaxX();
  ylims = MinMaxY();
  sizes = MinMaxRectangleSize();
  xdim = RandomNumber(sizes(1), sizes(2));
  ydim = RandomNumber(sizes(1), sizes(2));
  xpos = RandomNumber(xlims(1) + 0.5 * xdim, xlims(2) - 0.5 * xdim);
  ypos = RandomNumber(ylims(1) + 0.5 * ydim, ylims(2) - 0.5 * ydim);
       
  sim.m_obstacles(end + 1) = xpos - 0.5 * xdim;
  sim.m_obstacles(end + 1) = ypos - 0.5 * ydim;  
  sim.m_obstacles(end + 1) = xpos + 0.5 * xdim;
  sim.m_obstacles(end + 1) = ypos + 0.5 * ydim;
          
end

function [] = ComputeObstaclesInRange()
global sim;
r = sum(sim.m_lengths);
r2 = r * r;

sim.m_obstaclesInRange = [];
for k = 1 : 4 : length(sim.m_obstacles)
  xmin = sim.m_obstacles(k);
  ymin = sim.m_obstacles(k+1);
  xmax = sim.m_obstacles(k+2);
  ymax = sim.m_obstacles(k+3);
  if (xmin * xmin + ymin * ymin) <= r2 || ...
     (xmax * xmax + ymin * ymin) <= r2 || ...
     (xmax * xmax + ymax * ymax) <= r2 || ...
     (xmin * xmin + ymax * ymax) <= r2
     sim.m_obstaclesInRange(end + 1) = xmin;
     sim.m_obstaclesInRange(end + 1) = ymin;
     sim.m_obstaclesInRange(end + 1) = xmax;
     sim.m_obstaclesInRange(end + 1) = ymax;
  end     
end

end

function [] = DrawObstacles()
global sim;
        % Draw obstacles, assumed to be rectangles
        n = length(sim.m_obstacles);
        for i = 1 : 4 : n
          fill([sim.m_obstacles(i), sim.m_obstacles(i + 2), sim.m_obstacles(i + 2), sim.m_obstacles(i)], ...
               [sim.m_obstacles(i+1), sim.m_obstacles(i + 1), sim.m_obstacles(i + 3), sim.m_obstacles(i+3)], [0, 0, 1]);                 
        end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Overall Program
%   - Draw: 
%       window 1 shows main program, 
%       window 2 shows c-space obstacles (joint angles), 
%       window 3 shows c-space configurations that result in collision (superimposed manipulator configurations) 
%   - MyKeypressFcn
%       intercepts keyboard presses and run action based on user choice
  
function [] = Draw()
global sim;  
   clf;
   h = figure(1);
   set(h, 'name', 'Main Program');
   title('Manipulator and Obstacles');
  
   set(gca, 'xlim', MinMaxX()); 
   set(gca, 'ylim', MinMaxY());
   grid on;
   hold on;
   DrawObstacles();
   DrawManipulator();   
 
   h = figure(2);
   clf;
   set(h, 'name', 'Configurations in Collision: Angles');
   title('Configurations in Collisions for theta1, theta2 [showing angles]');
   set(gca, 'xlim', [sim.m_theta1Range(1), sim.m_theta1Range(end)]);
   set(gca, 'ylim', [sim.m_theta2Range(1), sim.m_theta2Range(end)]);
   grid on;
   hold on;
   DrawCSpaceObstacles();
   
   h = figure(3);
   clf;
   set(h, 'name', 'Configurations in Collision: Forward Kinematics');
   title('Configurations in Collisions for theta1, theta2 [showing manipulator]');
   set(gca, 'xlim', MinMaxX()); 
   set(gca, 'ylim', MinMaxY());
   grid on;
   hold on;
   DrawCSpaceCfgs();
   DrawObstacles();
   
   figure(1);
end

%by how much to change the joint angles when user selects ActionChangeJointAngle
function a = DeltaTheta()
 a = 0.1;
end

%by what factor to change the link length when user selects ActionChangeLinkLength
function a = LinkLengthFactor()
 a = 0.1;
end
  
 
function [] = MyKeyPressFcn(varargin)
  global sim;
 
  nrLinks = length(sim.m_lengths);
  
  key = get(gcf, 'CurrentCharacter');
  if key == ActionNone()
    sim.m_action = ActionNone();
  elseif key == ActionHelp()
    sim.m_action = ActionHelp();
    Help();    
  elseif key == ActionChangeJointAngle()
        sim.m_action = ActionChangeJointAngle();
  elseif key == ActionChangeLinkLength()
        sim.m_action = ActionChangeLinkLength();
  elseif key == ActionAddLink()
        sim.m_action = ActionAddLink();
        AddLink(sim.m_lengths(end));
        ForwardKinematics();
        ComputeObstaclesInRange();
        UpdateColor();  
        Draw();
  elseif key == ActionRemoveLastLink()
        sim.m_action = ActionRemoveLastLink();
        RemoveLastLink();
        ForwardKinematics();        
        ComputeObstaclesInRange();
        UpdateColor();  
        Draw();
  elseif key == ActionAddObstacle()
  	sim.m_action = ActionAddObstacle();
	AddRandomRectangle();	
        ComputeObstaclesInRange();
        UpdateColor();
  	Draw();
  elseif key == ActionComputeCSpace()
       sim.m_action = ActionComputeCSpace();
       ComputeCSpace();
       Draw(); 	
  elseif sim.m_action == ActionChangeJointAngle()
     if key >= '1' && key <= '9'
        sim.m_selectedLink = key - '0';
     end 
     if (key == '+' || key == '-') && sim.m_selectedLink >= 1 && sim.m_selectedLink <= nrLinks
        if key == '+'
           sim.m_thetas(sim.m_selectedLink) = sim.m_thetas(sim.m_selectedLink) + DeltaTheta();
        else
           sim.m_thetas(sim.m_selectedLink) = sim.m_thetas(sim.m_selectedLink) - DeltaTheta();
        end
     	ForwardKinematics();
        UpdateColor();
        Draw();   
     end
   elseif sim.m_action == ActionChangeLinkLength()
     if key >= '1' && key <= '9'
        sim.m_selectedLink = key - '0';
     end 
     if (key == '+' || key == '-') && sim.m_selectedLink >= 1 && sim.m_selectedLink <= nrLinks
        if key == '+'
           sim.m_lengths(sim.m_selectedLink) = sim.m_lengths(sim.m_selectedLink) * (1 + LinkLengthFactor());
        else
           sim.m_lengths(sim.m_selectedLink) = sim.m_lengths(sim.m_selectedLink) * (1 - LinkLengthFactor());
        end
     	ForwardKinematics();     	
        ComputeObstaclesInRange();
        UpdateColor();
        Draw();   
     end   
  end

end
  
