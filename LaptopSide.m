% Serial port setup
clear s;
serial_list = serialportlist;
port = serial_list(length(serial_list)-1);
%port = 'COM17';
s = serialport(port, 9600, 'timeout', 600);

% Camera setup
close all;
cam = webcam(2); % opens camera
x_res = 1920;
y_res = 1080;
scalingfactor = 0.15;  % Used to lower the resolution of the camera output
res = sprintf(('%dx%d'), x_res, y_res);
cam.Resolution=res;

% Constant Variables
minArea = round((x_res/100*scalingfactor)^2); % Minimum area threshold
min_circularity = 0.7; % Disgards target if less than this margin

% Centering the turret
centering_window = figure();
set(gcf,'CurrentCharacter','~');  % Define current character
elev_val = 0;
azi_val = 0;
finish = 0;
% Prevent the camera window from going out of focus when keys are pressed
preventDefault(centering_window,'WindowKeyPressFcn');
function preventDefault(obj,property)
  set(obj,property,@(~,~)0);
end
% Centering turret loop
while finish == 0
  % Receive data from pico telling us it is ready for a new instruction
  pico_output = s.readline();
  flush(s);
  % Get data from camera
  source_image = snapshot(cam); % grab 1 image
  lowres_image = imresize(source_image, scalingfactor);  % Lowers the resolution for increased performance
  imshow(lowres_image);
  hold on;
  title('Manual Turret Control');
  text(0, y_res+20, 'Directional Control: WASD | STOP: SPACE | QUIT: Q', 'Color', 'b'); % Prints legend
  key=get(gcf,'CurrentCharacter');
  if key~='~'
    if key=='q'
        finish=1;
    elseif key=='w'
        elev_val = 1;
        azi_val = 0;
    elseif key=='s'
        elev_val = -1;
        azi_val = 0;
    elseif key=='d'
        elev_val = 0;
        azi_val = 1;
    elseif key=='a'
        elev_val = 0;
        azi_val = -1;
    elseif key== ' '
        elev_val = 0;
        azi_val = 0;
    end
    key
    set(gcf,'CurrentCharacter','~'); % reset the character
  end
  % Write to RP2040
  msg_to_pico = sprintf(('%d,%d,%d'), elev_val, azi_val, finish);
  s.writeline(msg_to_pico);
end

%%_____MAIN PROGRAM LOOP_____%%
while 1    
    % Receive data from pico telling us it is ready for a new instruction
    pico_output = s.readline();

    % Get data from camera
    source_image = snapshot(cam); % grab 1 image
    source_image = imresize(source_image, scalingfactor);  % Lowers the resolution for increased performance
    
    %______RED_TARGETS______
    % Apply color threshold
    [red_thresholded_image, red_masked_image] = redLABMask(source_image); 
    % Extract properties for all objects
    red_props = regionprops(red_thresholded_image, 'Centroid', 'Area', 'BoundingBox', 'Circularity');
    
    %______YELLOW_TARGETS______
    % Apply color threshold
    [yellow_thresholded_image, yellow_masked_image] = yellowLABMask(source_image); 
    % Extract properties for all objects
    yellow_props = regionprops(yellow_thresholded_image, 'Centroid', 'Area', 'BoundingBox', 'Circularity');

    %______ORANGE_TARGETS______
    % Apply color threshold
    [orange_thresholded_image, orange_masked_image] = orangeLABMask(source_image); 
    % Extract properties for all objects
    orange_props = regionprops(orange_thresholded_image, 'Centroid', 'Area', 'BoundingBox', 'Circularity');
    
    % Add Together All Masked Images
    masked_image = red_masked_image + yellow_masked_image + orange_masked_image;
    
    % Filter out small objects (noise)
    filtered_red_props = red_props([red_props.Area] > minArea);
    filtered_yellow_props = yellow_props([yellow_props.Area] > minArea);
    filtered_orange_props = orange_props([orange_props.Area] > minArea);

    % Draw annotated image to the screen for live preview
    %imshowpair(source_image, masked_image, 'montage');  % Use for increased readability
    imagesc(masked_image);  % Use for increased perfomance over imshowpair
    axis off;
    hold on;
    
    % Annotate Red Props
    for k = 1:length(filtered_red_props)
        rectangle('Position', filtered_red_props(k).BoundingBox, 'EdgeColor', 'r', 'LineWidth', 2);
    end
    % Annotate Yellow Props
    for k = 1:length(filtered_yellow_props)
        rectangle('Position', filtered_yellow_props(k).BoundingBox, 'EdgeColor', 'y', 'LineWidth', 2);
    end
    % Annotate Orange Props
    for k = 1:length(filtered_orange_props)
        rectangle('Position', filtered_orange_props(k).BoundingBox, 'EdgeColor', 'm', 'LineWidth', 2);
    end
    title('Live Preview');
    hold off;
    
    % Label the image (RED PROPS)
    for k = 1:length(filtered_red_props)
        box = filtered_red_props(k).BoundingBox; % Finds the centroid
        centroid = [(box(1)+box(3))/2, (box(2)+box(4))/2]; % Finds the centroid
        circularity = filtered_red_props(k).Circularity; % Finds the circularity
        if circularity > min_circularity
            color = 'g';
        else
            color = 'r';
        end
        text(filtered_red_props(k).Centroid(1)-5, filtered_red_props(k).Centroid(2), sprintf('%.2f', circularity), 'Color', color); % Prints circularity value at centroid
    end
    % Label the image (YELLOW PROPS)
    for k = 1:length(filtered_yellow_props)
        box = filtered_yellow_props(k).BoundingBox; % Finds the centroid
        centroid = [(box(1)+box(3))/2, (box(2)+box(4))/2]; % Finds the centroid
        circularity = filtered_yellow_props(k).Circularity; % Finds the circularity
        if circularity > min_circularity
            color = 'g';
        else
            color = 'r';
        end
        text(filtered_yellow_props(k).Centroid(1)-5, filtered_yellow_props(k).Centroid(2), sprintf('%.2f', circularity), 'Color', color); % Prints circularity value at centroid
    end
    % Label the image (ORANGE PROPS)
    for k = 1:length(filtered_orange_props)
        box = filtered_orange_props(k).BoundingBox; % Finds the centroid
        centroid = [(box(1)+box(3))/2, (box(2)+box(4))/2]; % Finds the centroid
        circularity = filtered_orange_props(k).Circularity; % Finds the circularity
        if circularity > min_circularity
            color = 'g';
        else
            color = 'r';
        end
        text(filtered_orange_props(k).Centroid(1)-5, filtered_orange_props(k).Centroid(2), sprintf('%.2f', circularity), 'Color', color); % Prints circularity value at centroid
    end
    
    text(x_res*scalingfactor/2-1, y_res*scalingfactor/2-1, '.', 'Color', 'c'); % Prints crosshair

    % Filter out noise
    target_red_props = filtered_red_props([filtered_red_props.Circularity] > min_circularity);
    target_yellow_props = filtered_yellow_props([filtered_yellow_props.Circularity] > min_circularity);
    target_orange_props = filtered_orange_props([filtered_orange_props.Circularity] > min_circularity);
    red_target_printout = sprintf(('Red Targets: %d'), length(target_red_props));
    yellow_target_printout = sprintf(('Yellow Targets: %d'), length(target_yellow_props));
    orange_target_printout = sprintf(('Orange Targets: %d'), length(target_orange_props));
    text(2, 3, red_target_printout, 'Color', 'w'); % Prints legend
    text(2, 8, yellow_target_printout, 'Color', 'w'); % Prints legend
    text(2, 13, orange_target_printout, 'Color', 'w'); % Prints legend

    % Choose a waypoint
    % "Right now, this just chooses the biggest target on the screen."
    try
        [area, idx] = max(filtered_red_props.Area);
        centroid = [NaN,NaN];
        centroid = filtered_red_props.Centroid(idx,:);
        % Determine elevation and azimuth errors
        elevation_error = -int32((centroid(2))-y_res*scalingfactor/2);
        azimuth_error = int32((centroid(1))-x_res*scalingfactor/2);
    catch
        elevation_error = 0;
        azimuth_error = 0;
    end

    % Send elevation and azimuth errors to pico
    msg_to_pico = sprintf(('%d,%d'), elevation_error, azimuth_error);
    s.writeline(msg_to_pico);
    
end
