aruco_target_id = 3;
load('camera_param.mat');
tracking_distance = 4.0;

function pose = scan_arucos(camera_fd, camera_intrinsics, id)
  aruco_scale_mm = 173;
  img = snapshot(camera_fd);
  if isempty(img) || any(size(img) == 0)
    pose = [];
    return
  end
  
  [ids, ~, poses] = readArucoMarker(img, "DICT_4X4_250", camera_intrinsics, aruco_scale_mm);
  
  if isempty(ids)
    pose = [];
    return
  end

  for i = 1:length(ids)
    if ids(i) == id
      pose = [poses(i)];
      return;
    end
  end
  pose = [];

end

function distance = calculate_commands(aruco_tag, drone_ideal_distance)
  dx = aruco_tag.Translation(3)/1000;
  distance = dx - drone_ideal_distance;
end

% Initialize the drone
drone = parrot('Bebop2');
camera_fd = camera(drone, 'FPV');
drone.takeoff();

% Scan for the first aruco
aruco_tracking = false;
while aruco_tracking == false
  first_aruco_found = scan_arucos(camera_fd, cameraParams.Intrinsics, aruco_target_id);
  if ~isempty(first_aruco_found)
    aruco_tracking = true;
  end
end

% Start tracking the aruco
while true
  aruco_found = scan_arucos(camera_fd, cameraParams.Intrinsics, aruco_target_id);
  if ~isempty(aruco_found)
    distance = calculate_commands(aruco_found, tracking_distance);
    if distance > 0
      moveforward(drone, 0.4);
    else
      moveback(drone, 0.4);
    end
  end
  pause(1.0);
end