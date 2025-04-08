% Serial port setup
clear s;
serial_list = serialportlist;
port = serial_list(length(serial_list)-1);
port = 'COM10';
s = serialport(port, 9600, 'timeout', 600);
flush(s);

% Camera setup
close all;
cam = webcam(2); % opens camera
x_res = 1920;
y_res = 1080;
scalingfactor = 0.15;  % Used to lower the resolution of the camera output
res = sprintf(('%dx%d'), x_res, y_res);
%cam.Resolution=res;

% Constant Variables
%minArea = round((x_res/100*scalingfactor)^2); % Minimum area threshold
min_circularity = 0.7; % Disgards target if less than this margin
color_desired = "r";
number_of_targets = 2;

% Variables Updated in Program Loop
rel_pos_x = 0;
rel_pos_y = 0;
current_target_number = 1;
error = 0;
elevation_error = 999;
azimuth_error = 999;
pico_shot = 0;

%______________________________________________________________

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
flush(s);
while finish == 0
  pause(0.1);
  % Receive data from pico telling us it is ready for a new instruction
  pico_output = s.readline();
  split_output = split(pico_output.strip(), ',');
  color_desired = split_output(1,1);
  minArea = str2num(split_output(2,1));
  maxArea = str2num(split_output(3,1));
  %flush(s);
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
    pico_output = str2num(pico_output.strip());
    
    if pico_output == 2
        pico_shot = 1;
    else
        pico_shot = 0;
    end

    % Get data from camera
    source_image = snapshot(cam); % grab 1 image
    source_image = imresize(source_image, scalingfactor);  % Lowers the resolution for increased performance
    
    if color_desired == "r"
        % Apply color threshold
        [thresholded_image, masked_image] = redLABMask(source_image); 
    elseif color_desired == "y"
        % Apply color threshold
        [thresholded_image, masked_image] = yellowLABMask(source_image); 
    elseif color_desired == "o"
        % Apply color threshold
        [thresholded_image, masked_image] = orangeLABMask(source_image);
    end

    % Extract properties for all objects
    props = regionprops(thresholded_image, 'Centroid', 'Area', 'BoundingBox', 'Circularity');

    % Filter out small objects (noise)
    filtered_props = props([props.Area] > minArea);
    filtered_props = filtered_props([filtered_props.Area] < maxArea);

    % Draw annotated image to the screen for live preview
    %imshowpair(source_image, masked_image, 'montage');  % Use for increased readability
    imagesc(masked_image);  % Use for increased perfomance over imshowpair
    axis off; hold on;
    
    % Annotate Props
    for k = 1:length(filtered_props)
        rectangle('Position', filtered_props(k).BoundingBox, 'EdgeColor', 'r', 'LineWidth', 2);
    end
    title('Live Preview'); hold off;
    
    % Label the image
    for k = 1:length(filtered_props)
        box = filtered_props(k).BoundingBox; % Finds the centroid
        centroid = [(box(1)+box(3))/2, (box(2)+box(4))/2]; % Finds the centroid
        circularity = filtered_props(k).Circularity; % Finds the circularity
        if circularity > min_circularity
            color = 'g';
        else
            color = 'r';
        end
        text(filtered_props(k).Centroid(1)-5, filtered_props(k).Centroid(2), sprintf('%.2f', circularity), 'Color', color); % Prints circularity value at centroid
    end
    
    % Prints crosshair
    text(x_res*scalingfactor/2-1, y_res*scalingfactor/2-1, '.', 'Color', 'c'); % Prints crosshair

    % Filter out everything but circles
    target_props = filtered_props([filtered_props.Circularity] > min_circularity);
    target_printout = sprintf(('Number of Targets: %d'), length(target_props));
    text(2, 3, target_printout, 'Color', 'w'); % Prints legend

    % Choose a waypoint
    if length(target_props) ~= number_of_targets
        sprintf(('ERROR: %d targets detected'), length(target_props))
        elevation_error = 0;
        azimuth_error = 0;
        error = 1;  % Sets error flag to true
    else
        centroids = [filtered_props.Centroid];
        centroid_1 = [centroids(1),centroids(2)];  % Target 1
        centroid_2 = [centroids(3),centroids(4)];  % Target 2
        
        if rel_pos_x == 0 & rel_pos_y == 0  % If relative position not generated
            rel_pos_x = centroid_2(1) - centroid_1(1);
            %rel_pos_y = centroid_2(2) - centroid_1(2);
        end
        
        % Sets the leftmost target as the first to be shot
        if rel_pos_x > 0
            targets = [centroid_1; centroid_2];
        else
            targets = [centroid_2; centroid_1];
        end
        
        % Selects the centroid corresponding to the target we want
        current_centroid = targets(current_target_number, :);

        % Determine elevation and azimuth errors
        elevation_error = -int32((current_centroid(2))-y_res*scalingfactor/2);
        azimuth_error = int32((current_centroid(1))-x_res*scalingfactor/2);
        [elevation_error, azimuth_error]
        error = 0;  % Sets error flag to false
    end

    % Send elevation and azimuth errors to pico
    msg_to_pico = sprintf(('%d,%d'), elevation_error, azimuth_error);
    s.writeline(msg_to_pico);
    
    % Check if Pico shot
    if pico_shot == 1
        current_target_number = current_target_number + 1;
    end
    
    % Check to see if all targets shot
    if current_target_number > number_of_targets
        sprintf(('Program Complete'))
        break;
    end
    
end
