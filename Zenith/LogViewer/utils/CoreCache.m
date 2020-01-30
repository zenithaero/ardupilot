% Z1 Logs viewer script
% Created Jan 22, 2020
% Bertrand Bevillard <bertrand@zenithaero.com>

classdef CoreCache < handle
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
        vars = struct;
        logs = containers.Map();
    end
    
    methods
        function obj = CoreCache(~)
        end
        
        function init(obj)
            % Clear structures
            obj.vars  = struct;
                                 
            % Load new logs automatically if needed
            if isempty(obj.logFiles)
                dir = fileparts(mfilename('fullpath'));
                logpath = fullfile(dir, '../../Sim/logs/log.mat');
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
                
        function log = getLog(obj, l, p)
            [obj.vars.logFile, obj.vars.plotName] = deal(obj.logFiles{l}, obj.plots{p});

            % Populate case specific fields
            obj.vars.logKey = @(key) sprintf('%s_%s', obj.vars.logFile, key);
            obj.vars.key = @(key) sprintf('l%d_p%d_%s', l, p, key);
            logFile = split(obj.vars.logFile, '/'); 
            obj.vars.logName = sprintf('%s', logFile{end});

            % Load roots
            obj.loadLogs(obj.logFiles{l});
            log = obj.logs(obj.vars.logKey('root'));
        end
        
        function rtn = isPlot(obj, name)
            rtn = strcmp(name, obj.vars.plotName);
        end
        
        % Loaders -----------------------------------------------------------------
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

        function loadLogs(obj, filename)
            if ~exist(filename, 'file'); error('%s not found', filename); end
            if ~obj.logs.isKey(obj.vars.logKey('root'))
                fprintf('Loading %s...\n', filename);
                obj.logs(obj.vars.logKey('root')) = load(filename);
            end
        end
        
        % Figure helpers ----------------------------------------------------------
        function figureInit(obj)
            dockModes = {'normal', 'docked'};
            set(0,'DefaultFigureWindowStyle', dockModes{obj.dockFigs + 1});
            figHandles = get(groot, 'Children'); fig = [];
            figName = sprintf('%s - %s', obj.vars.logName, obj.vars.plotName);
            obj.vars.figName = figName;
            for k = 1:length(figHandles)
                if strcmp(figHandles(k).Name, figName) && obj.overrideFigs 
                    fig = figHandles(k);
                    figure(fig); 
                    break;
                end
            end
            if isempty(fig); figure('Name', figName, 'NumberTitle', 'off'); end
            set(gcf,'color','w');
        end

        function subplotInit(obj, rows, linkaxe)
            if ~exist('linkaxe', 'var'); linkaxe = true; end
            key = @(k) [obj.vars.key('subplot_'), k];
            if ~isfield(obj.vars, key('id'))
                obj.vars.(key('id')) = 0;
                obj.vars.(key('ax')) = [];
                size = [lcms(rows), length(rows)];
                map = reshape(1:prod(size), fliplr(size))';
                % Create index
                index = cell(sum(rows), 1);
                for k = 1:prod(size)
                    [i, j] = ind2sub(size, k);
                    ind = sum(rows(1:j-1)) + ceil(i/size(1)*rows(j));
                    index{ind}(end + 1) = map(k);
                end
                obj.vars.(key('size')) = size;
                obj.vars.(key('index')) = index;
            end
            obj.vars.(key('id')) = obj.vars.(key('id')) + 1;
            plotInd = obj.vars.(key('index')){obj.vars.(key('id'))};
            ax = subtightplot(obj.vars.(key('size'))(1),...
                obj.vars.(key('size'))(2), plotInd, 0.05);
            if linkaxe
                obj.vars.(key('ax'))(end + 1) = ax;
            end
        end

        function subplotFinalize(obj, xlabelStr, ylabelStr, titleStr)
            key = @(k) [obj.vars.key('subplot_'), k];
            if ~isempty(obj.vars.(key('ax')))
                linkaxes(obj.vars.(key('ax')), 'x');
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
                    ttl = sprintf('%s - %s', ttl, obj.vars.figName); % obj.vars.plotName);
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
            print(gcf, [dir, '/', obj.vars.figName, '.png'], '-dpng', '-r150')
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

        function [t, x] = trim(obj, time, vect, useCache)
            if ~exist('useCache', 'var'); useCache = 0; end
            key = @(k) obj.vars.key(sprintf('%s', k));
            if ~(useCache && isfield(obj.vars, key('idx')))
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
                obj.vars.(key('idx')) = idx(1):idx(2);
                obj.vars.(key('time')) = time(idx(1):idx(2));
            end
            t = obj.vars.(key('time'));
            x = vect(obj.vars.(key('idx')));
        end

        function plotEvents(obj, timeList, namesList, sepStyle, dir)
            ylimits = ylim; yext = ylimits(2) - ylimits(1);
            yLast = 0; tLast = 0; STEP = 0.05;
            for k = 1:length(timeList)
                if ~iscell(namesList{k}); namesList{k} = namesList(k); end
                for i = 1:length(namesList{k})
                    tMode = timeList(k);
                    if tMode < obj.vars.tRange(1) || tMode > obj.vars.tRange(2); continue; end
                    if tLast <= tMode; yLast = ylimits(1 + (1 - dir)/2) + dir*yext*STEP; tLast = 0; end
                    plot([tMode, tMode], ylimits, sepStyle);
                    t = text(tMode, yLast, sprintf(' %s', namesList{k}{i}), 'Interpreter', 'none');
                    set (t, 'Clipping', 'on');
                    tLast = max(tLast, t.Extent(1) + t.Extent(3));
                    yLast = yLast +dir*t.Extent(4);
                end
            end
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

