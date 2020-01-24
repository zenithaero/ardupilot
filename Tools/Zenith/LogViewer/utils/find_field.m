% Field finder script
% Created Jan 23, 2020
% Bertrand Bevillard <bertrand@zenithaero.com>

% -------------------------------------------------------------------------
% Crawl structure fields and print out all the match
% If structure is empty, the whole workspace will be scanned
% -------------------------------------------------------------------------

function numMatch = find_field(data, fieldName, ignoreCase, fullWord, prefix)
    if ~exist('ignoreCase', 'var'); ignoreCase = true; end
    if ~exist('fullWold', 'var'); fullWord = false; end
    if ~exist('prefix', 'var'); prefix = ''; end
    numMatch = 0;
    findRec = @(str, dat) find_field(dat, fieldName, ignoreCase, fullWord, [prefix, str]);
    if isstruct(data)
        for i = 1:numel(data)
            pref = prefix;
            if length(data) > 1
                pref = [prefix, '(', num2str(i), ')'];
            end
            fnames = fieldnames(data(i));
            for k = 1:length(fnames)
                match = contains(fnames{k}, fieldName, 'IgnoreCase', ignoreCase);
                match = match & (~fullWord || length(fieldName) == length(fnames{k}));
                if match
                    fprintf('%s.%s\n', pref, fnames{k});
                    numMatch = numMatch + 1;
                end
                numMatch = numMatch + findRec(['.', fnames{k}], data(i).(fnames{k}));
            end
        end
    elseif iscell(data)
        for k = 1:length(data)
            numMatch = numMatch + findRec(['{', num2str(k), '}'], data{k});
        end
    end
    if strlength(prefix) == 0
        fprintf('Found %d match(es)\n', numMatch);
    end
end