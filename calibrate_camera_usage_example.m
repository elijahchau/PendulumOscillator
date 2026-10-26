%example for how to use properly call the camera calibration function
function calibrate_camera_usage_example()  
    %file folder where your video is located
    %make sure that video_path ends with '\'
    video_path = 'C:\Users\taylorott\Desktop\pendulum_videos\';
    
    %file name of the calibration video
    video_fname = 'calibration_video01.mov';

    %file path of folder where we want to save processed data
    %make sure that save_path ends with '\'
    save_path = 'C:\Users\taylorott\Documents\';

    %file name of where we want to store processed data
    data_fname = 'camera_intrinsics01.mat';

    %describe the calibration board being used
    board_params = struct();
    %type of board (aprilTag vs charuco, etc.)
    board_params.marker_type = 'charuco-board';
    %marker family for pattern on board
    board_params.markerFamily = 'DICT_4X4_1000';
    %(integer) grid dimension of checkerboard 
    board_params.patternDims = [7, 10];
    %height/width of each checker square in mm
    board_params.checkerSize = 23;
    %height/width of aruco marker square (the inner black square), in mm
    board_params.markerSize = 12;

    %maximum number of video frames to use in calibration
    %more frames means more quality, but also will take longer!
    max_num_frames = 200;

    %run the camera calibration function
    camera_intrinsics = calibrate_camera([video_path,video_fname], board_params, max_num_frames);

    %save result in a file
    save([save_path,data_fname],'camera_intrinsics');
end

