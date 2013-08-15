%function sendWPsToFlightgear(u)
%SENDWPsTOFLIGHTGEAR Sends all waypoints to flightgear
%   Sends all waypoints in waypoint array along with initial point (GS) to
%   flightgear via the UDP stream u that must be defined in the path.

% PORT = 56832; % obtained from startFlightgear.m

% Load waypoints and ground station
%run('simulationWPFile');

if ~exist('Xpoints','var') || ~exist('Ypoints','var') || ~exist('Zpoints','var') || ~isvector(Xpoints) || ~isvector(Ypoints) || ~isvector(Zpoints) || ~exist('wpCount','var')
    error('Simulation WP file was never loaded.');
end
if ~exist('u','var') || ~strcmp(class(u),'udp')
    error('A udp socket ''u'' was never created.');
end

if wpCount < 1
    disp('No waypoints to send to flightgear.');
    return;
end

% Wait for flightgear to load
%pause(LOAD_DELAY);

% Make telnet connection
%u = udp('127.0.0.1',PORT);
%fopen(u);
%pause(1);

disp('Sending waypoints to flightgear...');

% Load waypoints and clear old ones
% udp packets are in the following format:
%   waypoint, update_waypoints, l2_vecx, l2_vecy, l2_vecz, l2_veclen, l2_draw
% except fields are separated by the tab character '\t'

sendWPCommandToFlightgear(u,'@CLEAR'); % clear existing waypoints
pause(0.2);
sendWPCommandToFlightgear(u,'@DELETE0'); % clear airport
pause(0.2);

% Draw the initial point first
%sendWPToFlightgear(u,Geod_0(1),Geod_0(2),Geod_0(3));
%pause(0.2);
 
% Send each waypoint
for i=1:wpCount
    sendWPToFlightgear(u,Xpoints(i),Ypoints(i), Zpoints(i));
    %fwrite(u,sprintf('%f,%f@%.0f\t0\t0\t0\t0\t0\t0\n',Ypoints(i),Xpoints(i),...
    %    (meter_to_feet(Zpoints(i) + ALTITUDE_OFFSET)) ));
    pause(0.2);
end

% Send WP1 to close loop
sendWPToFlightgear(u,Xpoints(1),Ypoints(1), Zpoints(1));
pause(0.2);

sendWPCommandToFlightgear(u,'waypoint'); % draws all waypoints and legs
pause(0.2);


fprintf('Sent %d waypoints to flightgear to be drawn.',wpCount + 1);

%end