% Z1 Logs viewer script
% Created Jan 22, 2020
% Bertrand Bevillard <bertrand@zenithaero.com>

classdef Core < handle
    properties
        timeRange = {[], []};  % Tuple of {modeDesc, time or [] for start/end}
        mainTitle = [];        % Optional main title
        mainTitleTags = true;  % Appends the plotName to the title
        overrideFigs = false;  % Override previous figure
        dockFigs = false;      % Doc figures
        saveFigs = false;      % Save figures

        % Plot descriptor
        plots = {};       % Plot list
        logFiles = {};    % {folder of logs, log files or empty for newlogs}
        cacheLogs = true; % keep logs in memory to speedup multiple executions
        
        % Inner structures
        l = 0; % Log index
        p = 0; % Plot index
        logs = containers.Map();
        subplotInd = 0;
        subplotAxes = [];
    end
    
    methods
        function obj = Core(~)
        end
        
        function init(obj)                                 
            % Load new logs automatically if needed
            if isempty(obj.logFiles)
                dir = fileparts(mfilename('fullpath'));
                logpath = fullfile(dir, '../../Sim/out/logs/log.mat');
                obj.logFiles = {logpath};
                obj.cacheLogs = false; % Reload sim logs every time
            elseif length(obj.logFiles) == 1 && isfolder(obj.logFiles{1})
                obj.logFiles = obj.findLogs(obj.logFiles{1});
            end
            
            % Clear log cache if needed
            if  ~obj.cacheLogs; obj.logs = containers.Map(); end
            
            % Clear plots
            close all;
        end
        
        function indices = loopIndices(obj)
            indices = 1:prod([length(obj.logFiles), length(obj.plots)]);
        end
        
        function setIndex(obj, k)
            indices = obj.loopIndices();
            [obj.l, obj.p] = ind2sub(indices, k);
        end
                
        function rtn = isPlot(obj, name)
            rtn = strcmp(name, obj.plotName());
        end
        
        function logName = logName(obj)
            filename = obj.logFiles{obj.l};
            logFile = split(filename, '/'); 
            logName = sprintf('%s', logFile{end});
        end
        
        function plotName = plotName(obj)
            plotName = obj.plots{obj.p};
        end
        
        function figName = figName(obj)
           figName = sprintf('%s - %s', obj.logName(), obj.plotName());
        end
        
        % Logs loaders ----------------------------------------------------
        function logs = findLogs(obj, folder)  %#ok<*AGROW>
            logs = {};
            cont = dir(folder);
            for k = 1:length(cont)
                [~, name, ext] = fileparts(cont(k).name);
                file = [cont(k).folder, '/', name, ext];
                if isfolder(file) && ~any(strcmp(ext, {'.', '..'})) 
                    logs = [logs; obj.findLogs(file)];
                elseif any(strcmp(ext, {'.mat'}))
                    logs{end + 1} = file;
                end
            end
        end

        function log = getLog(obj)
            filename = obj.logFiles{obj.l};
            if ~exist(filename, 'file'); error('%s not found', filename); end
            if ~obj.logs.isKey(obj.l)
                fprintf('Loading %s...\n', filename);
                obj.logs(int2str(obj.l)) = load(filename);
            end
            log = obj.logs(int2str(obj.l));
        end
        
        % Figure helpers ----------------------------------------------------------
        function figureInit(obj)
            dockModes = {'normal', 'docked'};
            set(0,'DefaultFigureWindowStyle', dockModes{obj.dockFigs + 1});
            figure('Name', obj.figName(), 'NumberTitle', 'off');
            set(gcf,'color','w');
            obj.subplotAxes = [];
            obj.subplotInd = 0;
        end

        function subplotInit(obj, rows, linkaxe)
            if ~exist('linkaxe', 'var'); linkaxe = true; end
            size = [lcms(rows), length(rows)];
            map = reshape(1:prod(size), fliplr(size))';
            % Create index
            index = cell(sum(rows), 1);
            for k = 1:prod(size)
                [i, j] = ind2sub(size, k);
                ind = sum(rows(1:j-1)) + ceil(i/size(1)*rows(j));
                index{ind}(end + 1) = map(k);
            end
            obj.subplotInd = obj.subplotInd + 1;
            ax = subtightplot(size(1), size(2), obj.subplotInd, 0.05);
            if linkaxe
                obj.subplotAxes(end + 1) = ax;
            end
        end

        function subplotFinalize(obj, xlabelStr, ylabelStr, titleStr)
            if ~isempty(obj.subplotAxes)
                linkaxes(obj.subplotAxes, 'x');
            end
            xlabel(xlabelStr); ylabel(ylabelStr); title(titleStr);
            grid on; hold on; legend show
            set(legend, 'location', 'southeast');
        end
        
        function figureFinalize(obj)
            % Print title
            if ~isempty(obj.mainTitle) 
                ttl = obj.mainTitle;
                if obj.mainTitleTags
                   ttl = sprintf('%s - %s', ttl, obj.figName());
                end
                suplabel(ttl, 't', [0 0 1 0.97]);
            end
            if obj.saveFigs; obj.saveFigure(); end
        end
        
        function saveFigure(obj)
            dir = [fileparts(mfilename('fullpath')), '/figures'];
            if ~exist(dir, 'dir')
                mkdir(dir);
            end
            print(gcf, [dir, '/', obj.figName(), '.png'], '-dpng', '-r150')
        end

        % Plot helpers ------------------------------------------------------------
        function plotCurve(obj, time, vect, legendStr, style, width, smoothFactor)
            if ~exist('width', 'var') || isempty(width); width = 2; end
            args = {'DisplayName', legendStr, 'LineWidth', width}; 
            if exist('style', 'var') && ~isempty(style); args = [args, 'LineStyle', style]; end
            [t, x] = obj.trim(time, vect);
            if exist('smoothFactor', 'var'); x = Core.smooth(x, smoothFactor); end
            plot(t, x, args{:}); hold on
        end

        function [t, x] = trim(obj, time, vect)
            % Find id & time range and cache them
            idx = [1, length(time)];
            for i = 1:2
                found = [];
                if obj.timeRange{i} >= 0
                    found = find(time >= obj.timeRange{i}, 1);
                end
                if ~isempty(found)
                    idx(i) = found;
                end
            end
            range = idx(1):idx(2);
            t = time(range);
            x = vect(range);
        end
    end
    
    % Helper functions -----------------------------------
    methods(Static)
        function xs = smooth(x, window)
            x = [x(1)*ones(window, 1); x];
            b = (1/window)*ones(1,window);
            xs = filter(b, 1, x);
            xs = xs(window + 1:end, :);
        end

        function varargout = align(varargin)
            minDim = inf;
            for k = 1:length(varargin)
                minDim = min(length(varargin{k}), minDim);
            end
            for k = 1:length(varargin)
                varargout{k} = varargin{k}(1:minDim);
            end
        end
    end
end

