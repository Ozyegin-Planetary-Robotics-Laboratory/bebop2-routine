aruco_start_id = 101;
aruco_marker_102 = 102;
aruco_marker_103 = 103;
aruco_marker_104 = 104;
load('camera_param.mat');
tracking_distance = 4.0; % Desired distance from the Aruco marker in meters
max_altitude = 5.0; % Maximum allowed altitude in meters

function pose = scan_arucos(camera_fd, camera_intrinsics, id)
aruco_scale_mm = 150; % Size of the Aruco marker in mm
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

function [dx, dy, dz] = calculate_commands(aruco_tag, drone_ideal_distance)
if isempty(aruco_tag) || ~isfield(aruco_tag, 'Translation')
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

% Initialize the drone
drone = parrot('Bebop2');
camera_fd = camera(drone, 'FPV');
drone.takeoff();

% Function to read the height of the drone
function altitude = readDroneHeight(drone)
[~, altitude] = readHeight(drone);
end

% Move to a specific Aruco marker and take a photo if needed
function move_to_marker(camera_fd, camera_intrinsics, drone, target_id, tracking_distance, max_altitude, take_photo)
marker_found = false;
while ~marker_found
  aruco_found = scan_arucos(camera_fd, camera_intrinsics, target_id);
  if ~isempty(aruco_found)
    marker_found = true;
  end
end

while true
  aruco_found = scan_arucos(camera_fd, camera_intrinsics, target_id);
  if ~isempty(aruco_found)
    [dx, dy, dz] = calculate_commands(aruco_found, tracking_distance);
    
    % Read the current altitude
    current_altitude = readDroneHeight(drone);
    
    % Ensure altitude reading is valid
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
    
    % Check if the drone is within 0.5 meters of the marker
    if abs(dx) <= 0.5 && abs(dy) <= 0.5 && abs(dz) <= 0.5
      if take_photo
        img = snapshot(camera_fd);
        filename = sprintf('marker_%d_photo.png', target_id);
        imwrite(img, filename);
      end
      break;
    end
  else
    % Handle case where Aruco is not found
    disp('Aruco marker not found. Hovering...');
    hover(drone);
  end
  pause(1.0);
end
end

% Follow the specified path
move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_start_id, tracking_distance, max_altitude, false);
move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_marker_102, tracking_distance, max_altitude, true);
move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_marker_103, tracking_distance, max_altitude, false);
land(drone);
pause(5); % Wait for 5 seconds
takeoff(drone);
move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_marker_104, tracking_distance, max_altitude, true);
move_to_marker(camera_fd, cameraParams.Intrinsics, drone, aruco_start_id, tracking_distance, max_altitude, false);
land(drone);
