classdef Helpers
   methods(Static)
       function h = plotVehicle(h, east, north, yaw, length)
            if isempty(h)
                h = fill(east, north, 'r');
            end
            h.XData = east - length*[0; sin(yaw + pi/6); sin(yaw - pi/6)];
            h.YData = north - length*[0; cos(yaw + pi/6); cos(yaw - pi/6)];
        end

        function [n ,e] = LLtoNE(lat0, lon0, lat, lon)
            cl0 = cos(lat0);
            EARTH_RADIUS = 6367444.6571;
            n = (lat - lat0) * EARTH_RADIUS;
            e = (lon - lon0) * EARTH_RADIUS * cl0;
        end

        function [lat, lon] = NEtoLL(lat0, lon0, n, e)
            cl0 = cos(lat0);
            EARTH_RADIUS = 6367444.6571;
            lat = double(n) / EARTH_RADIUS + lat0;
            lon = double(e) / EARTH_RADIUS / cl0 + lon0;
        end

        function [x, y] = NEtoXY(N, E, yaw)
            x = N.*cos(yaw) + E.*sin(yaw);
            y = E.*cos(yaw) - N.*sin(yaw);
        end
   end
end