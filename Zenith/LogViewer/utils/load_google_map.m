% Google Map loader script
% Created Jan 23, 2020
% Bertrand Bevillard <bertrand@zenithaero.com>



% Load Google map around the bounds
function [imagN, imagE, imag] = load_google_map(origLL, boundN, boundE)
    % Google API key
    API_KEY = 'AIzaSyCoJPPv42LH6r6Kl9IfGlXRUjRrGVOp3Nk';
    % Check if the map exits in the cache
    filename = sprintf(['map', repmat('_%f', 1, 6)], origLL(:), boundN(:), boundE(:));
    dir = fullfile(fileparts(mfilename('fullpath')), 'maps_cache');
    path = fullfile(dir, [filename, '.mat']);
    if exist(path, 'file')
        load(path, 'imagN', 'imagE', 'imag');
        return;
    end    
    toRad = pi/180; toDeg = 1/toRad;
    [boxLat, boxLong] = Helpers.NEtoLL(origLL(1), origLL(2), boundN, boundE);
    [boxLong, boxLat] = deal(boxLong*toDeg, boxLat*toDeg);
    figure('Name', 'Maps'); clf; plot(boxLong, boxLat);
    [lonVec, latVec, imag] = plot_google_map('MapType', 'satellite', 'apikey', API_KEY);

    % find the smallest square box that holds the desired bounds
    idx = find( (lonVec >= boxLong(1) & lonVec <= boxLong(2)) | (latVec >= boxLat(1) & latVec <= boxLat(2)));
    lonVec = lonVec(idx);
    latVec = latVec(idx);
    imag = imag(idx, idx, :);
    
    [imagN, imagE] = Helpers.LLtoNE(origLL(1), origLL(2), latVec*toRad, lonVec*toRad);
    
    close('Maps');
    % Save map to the cache
    if ~exist(dir, 'dir'); mkdir(dir); end
    save(path, 'imagN', 'imagE', 'imag');
end