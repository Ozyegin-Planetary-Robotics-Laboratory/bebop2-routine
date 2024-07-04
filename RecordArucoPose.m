%---------------%
% Configuration % 
%---------------%
target_aruco_dict   = 'DICT_4x4_50';
target_aruco_len    = 110;  % mm
target_aruco_id     = 1;
frames_per_sec      = 3;    % hz
total_duration      = 10;   % secs
load('cameraParams.mat'); 
%----------------%

% Create a connection to the drone
drone = parrot('Bebop2');
drone_cam = camera(drone, 'FPV');

total_frames = frames_per_sec * total_duration;
camera_poses = []

% Scan ArUcos for the total duration.
for i = 1:total_frames
  pause(1.0 / frames_per_sec);
  frame = snapshot(drone_cam);
  [ids, ~, poses] = readArucoMarker(frame, target_aruco_dict, cameraParams.Intrinsics, target_aruco_len);
  if isempty(ids)
    continue;
  end
  for j = 1:length(ids)
    if ids(j) == target_aruco_id
      timestamp = datetime('now');
      camera_poses = [camera_poses; [poses(j), timestamp]];
      break;
    end
  end
end

% Display camera positions.
grid on
axis equal
axis manual
origin = rigidtform3d;
origin.Rotation = eye(3);
origin.Translation = [0, 0, 0];
for i = 1:size(camera_poses)
  rigid_transform = camera_poses(i)
  cam_plot = plotCamera('AbsolutePose', rigid_transform, 'Size', 0.1);
end
