% Z1 Logs viewer script
% Created Jan 22, 2020
% Bertrand Bevillard <bertrand@zenithaero.com>
%
% --------------------------------------------------------------------- #
% Ardupilot log viewer
% --------------------------------------------------------------------- #
%
% Handy log plotting tool
% --------------------------------------------------------------------- #

% Instanciate the core
if ~exist('core', 'var'); core = Core(); end

% Time range presets -----
core.timeRange = {-1, -1}; % Plots time window
% ------------------------

% Figure preferences -----
core.mainTitle    = '';             % Optional main title
core.mainTitleTags = false;         % Appends the plotName to the title
core.overrideFigs  = true;          % Override previous figure
core.dockFigs      = true;          % Docs all figures
core.saveFigs      = false;         % Save figures
% -----------------------

% Plot descriptor --------
core.logFiles     = {};     % {folder of logs, log files or empty for latest sim logs}
core.cacheLogs    = false;  % Keep logs in memory (prevent re-loading)
% ------------------------

% ------------------------
% 2d      - 2d pos plot
% 2d_map  - 2d pos plot with map
% 3d_map  - 3d pos plot with map
% long    - Longitudinal plot
% lat     - Lateral plot
% ------------------------
core.plots = {'2d', '3d_map', 'long', 'lat'};

core.init();
toDeg = 180/pi;

% Main loop
loopSize = [length(core.logFiles), length(core.plots)];
for k = 1:prod(loopSize)
    % Unpack index & init loop
    [l, p] = ind2sub(loopSize, k);
    log = core.getLog(l, p);
    
    % Position plots ----------------------------------------------------------
    if core.isPlot('2d') || core.isPlot('2d_map') || core.isPlot('3d_map')
        core.figureInit();
        rows = 1;
        % Scale AGL
        aglScaling = 1;
        if core.isPlot('3d_map'); aglScaling = 3; end
        
        [N, E, agl, ~, NCmd, ECmd, aglCmd, ~] = getPosVar(core, log);
        
        % Load map
        mapPaddingPercent = 0.2; minLength = 20; % [m]
        [boundN, boundE] = deal([min(N), max(N)], [min(E), max(E)]);
        [padN, padE] = deal(max(diff(boundN)*mapPaddingPercent, minLength/2), max(diff(boundE)*mapPaddingPercent, minLength/2));
        [boundN, boundE] = deal(boundN + [-padN, padN], boundE + [-padE, padE]);
        [mapN, mapE, mapImg] = load_google_map(origLL, boundN, boundE);
        
        core.subplotInit(rows);
        
        % Plot map
        if ~core.isPlot('2d')
            image(mapE, mapN, mapImg); alpha(.6);
        end
        axis xy; hold on
        
        % Plot command
        plot3(ECmd, NCmd, aglCmd * aglScaling, '-.g', 'linewidth', 2);
        scatter3(ECmd, NCmd, aglCmd * aglScaling, 100, 'b', 'filled');

        plot3(E, N, agl * aglScaling, 'r', 'linewidth', 2);
        legend('Cmd', 'Pos');
        axis equal; xlim(sort([mapE(1), mapE(end)])); ylim(sort([mapN(1), mapN(end)]));
        zlabelstr = 'Altitude AGL [m]';
        if aglScaling ~= 1; zlabelstr = sprintf('%s (x%.1f)', zlabelstr, aglScaling); end
        xlabel('Easting [m]'); ylabel('Northing [m]'); zlabel(zlabelstr);
        grid on;
        view(0, 90);
        if core.isPlot('3d_map'); view(-30, 30); end
    end
    
    % Body longitudinal ---------------------------------------------------
    if core.isPlot('long')
        core.figureInit();
        rows = [2, 3]; % [column 1, column 2]...
        
        [N, E, agl, t, NCmd, ECmd, aglCmd, tCmd] = getPosVar(core, log);
        % [X, Y] = Helpers.NEtoXY(N, E, estimator.yaw);

        % N pos
        core.subplotInit(rows);
        core.plotCurve(tCmd, NCmd, 'nCmd', '-.');
        core.plotCurve(t, N, 'N', '-', 2);
        core.subplotFinalize('time (s)', 'm', 'N pos');
        
        % E pos
        core.subplotInit(rows);
        core.plotCurve(tCmd, ECmd, 'ECmd', '-.');
        core.plotCurve(t, E, 'E', '-', 2);
        core.subplotFinalize('time (s)', 'm', 'E pos');
        
        % Alt
        core.subplotInit(rows);
        core.plotCurve(tCmd, aglCmd, 'aglCmd', '-.');
        core.plotCurve(t, agl, 'agl', '-', 2);
        core.subplotFinalize('time (s)', 'm', 'AGL');

        % Velocity
        core.subplotInit(rows);
        core.plotCurve(log.TECS.timestamp, log.TECS.spdem, 'spCmd', '-.');
        core.plotCurve(log.TECS.timestamp, log.TECS.sp, 'sp', '-');
        core.subplotFinalize('', 'm/s', 'Speed');

        % Pitch
        core.subplotInit(rows);
        core.plotCurve(log.ATT.timestamp, log.ATT.DesPitch, 'pitchCmd', '-.');
        core.plotCurve(log.ATT.timestamp, log.ATT.Pitch, 'pitch', '-');
        core.subplotFinalize('', 'deg', 'Pitch');
    end
    
    % Body lateral ---------------------------------------------------
    if core.isPlot('lat')
        core.figureInit();
        rows = [2, 2]; % [column 1, column 2]...
        
        [N, E, agl, t, NCmd, ECmd, aglCmd, tCmd] = getPosVar(core, log);
        % [X, Y] = Helpers.NEtoXY(N, E, estimator.yaw);

        % N pos
        core.subplotInit(rows);
        core.plotCurve(tCmd, NCmd, 'nCmd', '-.');
        core.plotCurve(t, N, 'N', '-', 2);
        core.subplotFinalize('time (s)', 'm', 'N pos');
        
        % E pos
        core.subplotInit(rows);
        core.plotCurve(tCmd, ECmd, 'ECmd', '-.');
        core.plotCurve(t, E, 'E', '-', 2);
        core.subplotFinalize('time (s)', 'm', 'E pos');

        % Roll
        core.subplotInit(rows);
        core.plotCurve(log.ATT.timestamp, log.ATT.DesRoll, 'rollCmd', '-.');
        core.plotCurve(log.ATT.timestamp, log.ATT.Roll, 'roll', '-');
        core.subplotFinalize('', 'deg', 'Roll');
        
        % Yaw
        core.subplotInit(rows);
        core.plotCurve(log.ATT.timestamp, log.ATT.DesYaw, 'yawCmd', '-.');
        core.plotCurve(log.ATT.timestamp, log.ATT.Yaw, 'yawm', '-');
        core.subplotFinalize('', 'deg', 'Yaw');
    end
    
    % 2d anim -------------------------------------------------------------
    if core.isPlot('2d_anim')
        % TODO: update
        error('Needs update')
        
        core.figureInit(); %#ok<UNRCH>
        rows = 1; % [column 1, column 2]...
        
        % Settings
        speed = 60;
        saveAnim = true;
        frameRate = 20;
        
        [~, Ncmd] = core.trim(internal2, auto_controller.NPosCmd);
        [~, Ecmd] = core.trim(internal2, auto_controller.EPosCmd);
        [~, NcmdFilt] = core.trim(internal2,auto_controller.NPosCmdFilt);
        [~, EcmdFilt] = core.trim(internal2,auto_controller.EPosCmdFilt);
        [~, N] = core.trim(internal2,estimator.ENU_N);
        [~, E] = core.trim(internal2,estimator.ENU_E);
        [~, lN] = core.trim(internal2,auto_controller.CTOL_leaderN);
        [~, lE] = core.trim(internal2,auto_controller.CTOL_leaderE);
        [~, yaw] = core.trim(internal2,estimator.yaw);
       
        % Plot constant info
        events = logical([0; diff(Ncmd) ~= 0 | diff(Ecmd) ~= 0]);
        scatter(Ecmd(events), Ncmd(events), 'MarkerFaceColor', 'k'); hold on
        plot(lE, lN, '--', 'LineWidth', 2);
        xlabel('Easting [m]'); ylabel('Northing [m]');
        axis equal; grid on
        
        % Create the plots
        leaderLine =  plot([0, 0], [0, 0], '-');
        cmd = plot(0, 0, 'Marker', 'square', 'MarkerSize', 15, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');
        leader = plot(0, 0, 'Marker', 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');
        posTrack = plot(0, 0, ':', 'LineWidth', 2);
        
        % Set bounds
        nBounds = [min(N), max(N)];
        eBounds = [min(E), max(E)];
        padding = 0.1*max(nBounds(2) - nBounds(1), eBounds(2) - eBounds(1));
        xlim([eBounds(1) - padding, eBounds(2) + padding]);
        ylim([nBounds(1) - padding, nBounds(2) + padding]);

        vehicleLength = padding/3;
        vehicle = plotVehicle([], 0, 0, 0, vehicleLength);
        
        % Setup videoWriter
        if saveAnim
            v = VideoWriter(strrep(core.mainTitle, ' ', '_'), 'MPEG-4'); %#ok<TNMLP>
            v.FrameRate = frameRate;
            open(v);
        end
        
        for z = 1:speed:length(Ncmd)
            t = tic;
            cmd.XData = Ecmd(z);
            cmd.YData = Ncmd(z);
            leader.XData = lE(z);
            leader.YData = lN(z);
            d = sqrt((Ncmd(z) - N(z))^2 + (Ecmd(z) - E(z))^2);
            leaderLine.XData = [E(z), lE(z)];
            leaderLine.YData = [N(z), lN(z)];
            posTrack.XData(end + 1) = E(z);
            posTrack.YData(end + 1) = N(z);
            plotVehicle(vehicle,  E(z), N(z),yaw(z), vehicleLength);
            drawnow;
            if saveAnim; writeVideo(v, getframe(gcf)); end
            pause(max(1/frameRate - toc(t), 0));
        end
        
        if saveAnim; close(v); end
    end
    
    core.figureFinalize();
end

% Helper functions
function [N, E, agl, t, NCmd, ECmd, aglCmd, tCmd] = getPosVar(core, log)
    pos = log.POS;
    cmd = log.CMD;
    toRad = pi/180;
    
    % Trim position
    origLL = [log.ORGN.Lat(end) * toRad, log.ORGN.Lng(end) * toRad];
    [N, E] = Helpers.LLtoNE(origLL(1), origLL(2), pos.Lat * toRad, pos.Lng * toRad);
    [t, N] = core.trim(pos.timestamp, N);
    [~, E] = core.trim(pos.timestamp, E);
    [~, agl] = core.trim(log.NKF5.timestamp, log.NKF5.HAGL);
    [t, N, E, agl] = Core.align(t, N, E, agl);

    % Trim commands
    [NCmd, ECmd] = Helpers.LLtoNE(origLL(1), origLL(2), cmd.Lat * toRad, cmd.Lng * toRad);
    [tCmd, NCmd] = core.trim(cmd.timestamp, NCmd);
    [~, ECmd] = core.trim(cmd.timestamp, ECmd);
    cmd.Alt(1) = 0; % FIX ABSOLUTE COMMAND - Add logged struct
    % altCmd = cmd.Alt - (cmd.Frame == 0) .* orgn.Alt(end);
    [~, aglCmd] = core.trim(cmd.timestamp, cmd.Alt);
    % Mask out invalid commands
    mask = (cmd.Lat == 0) | (cmd.Lng == 0) | (tCmd < 0);
    tCmd(mask) = []; NCmd(mask) = []; ECmd(mask) = []; aglCmd(mask) = [];
end

