function [ output_args ] = sendWPCommandToFlightgear( u, cmd_str )
%sendWPCommandToFlightgear Sends a command to a flightgear via udp
%   Sends the given command to flightgear via the udp stream u.
%   Valid commands are either route-manager commands, or update commands.
%   Route manager commands can be:
%       @CLEAR - clears the waypoint list
%       @DELETE# - deletes the waypoint with the given index from the list
%   Update commands can be:
%       waypoint - draws all listed waypoint and legs between
%       aimpoint - draws an aim point at the listed waypoints
%
k = findstr(cmd_str, '@');
if length(k) > 1
    k = k(1);
end

if k == 1
    % Route manager command
    fwrite(u,sprintf('%s\t0\t0\t0\t0\n',cmd_str));
else
    % Update command
    fwrite(u,sprintf('\t%s\t0\t0\t0\n',cmd_str));
end


end

