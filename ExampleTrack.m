load('camera_param.mat');

% Configuration
tracking_distance = 4.0;                   % Desired distance from the Aruco marker in meters
max_altitude      = 3.0;                   % Maximum allowed altitude in meters
aruco_ids         = [1, 2, 3, 4];          % Aruco marker IDs to follow
aruco_scale_mm    = 150;                   % Size of the Aruco marker in mm

function pose = find_aruco(camera_fd, camera_intrinsics, id)
  img = snapshot(camera_fd);
  if isempty(img) || any(size(img) == 0)
    pose = [];
    return;
  end

  try
    [ids, ~, poses] = readArucoMarker(img, "DICT_5X5_1000", camera_intrinsics, aruco_scale_mm);
  catch
    pose = [];
    return;
  end

  if isempty(ids)
    pose = [];
    return;
  end

  for i = 1:length(ids)
    if ids(i) == id
      pose = poses(i);
      return;
    end
  end
  pose = [];
end

function [dx, dy, dz] = calculate_commands(aruco_pose, drone_ideal_distance)
  if isempty(aruco_pose) || ~isfield(aruco_pose, 'Translation')
    dx = 0;
    dy = 0;
    dz = 0;
    return;
  end

  dx = aruco_tag.Translation(3) / 1000; % Convert from mm to meters
  dz = aruco_tag.Translation(1) / 1000; % Convert from mm to meters
  dy = aruco_tag.Translation(2) / 1000; % Convert from mm to meters
  dx = dx - drone_ideal_distance;
end


% Function to read the height of the drone
function altitude = get_height(drone)
  [altitude, ~] = readHeight(drone);
end

% Move to a specific Aruco marker and take a photo if needed
function move_to_marker(camera_fd, camera_intrinsics, drone, target_id, tracking_distance, max_altitude, take_photo)
  marker_found = false;
  while ~marker_found
    aruco_position_3d = find_aruco(camera_fd, camera_intrinsics, target_id);
    if ~isempty(aruco_position_3d)
      marker_found = true;
    end
  end

  while true
    pause(1.0);
    aruco_position_3d = find_aruco(camera_fd, camera_intrinsics, target_id);
    if ~isempty(aruco_position_3d)
      [dx, dy, dz] = calculate_commands(aruco_position_3d, tracking_distance);  
      current_altitude = get_height(drone);
      if isempty(current_altitude)
        current_altitude = 0;
      end
      % Adjust drone position based on the distance calculations
      if dx > 0.1
        moveforward(drone, 0.2);
      elseif dx < -0.1
        moveback(drone, 0.2);
      end
      if dy > 0.1
        moveright(drone, 0.2);
      elseif dy < -0.1
        moveleft(drone, 0.2);
      end
      if dz > 0.1 && current_altitude < max_altitude
        moveup(drone, 0.2);
      elseif dz < -0.1
        movedown(drone, 0.2);
      end
    else
      disp('Aruco marker not found. Hovering...');
    end
  end
end

% Initialize Drone
drone = parrot('Bebop2');
camera_fd = camera(drone, 'FPV');
drone.takeoff();

% Stage 1 (Move)
move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_ids(1), tracking_distance, max_altitude, false);
land(drone);

% Stage 2 (Move - Take Photo)
% move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_ids(2), tracking_distance, max_altitude, true);
% Stage 3 (Move - Land - Takeoff)
% move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_ids(3), tracking_distance, max_altitude, false);
% land(drone);
% pause(5); % Wait for 5 seconds
% takeoff(drone);
% Stage 4 (Move - Take Photo)
% move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_ids(4), tracking_distance, max_altitude, true);
% Stage 5 (Move - Land)
% move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_ids(1), tracking_distance, max_altitude, false);
% land(drone);
