%computes camera intrinsics from a calibration video
function camera_intrinsics = calibrate_camera(fname, board_params, max_num_frames)  

    %create a new VideoReader object to process the data
    vidObj = VideoReader(fname);

    %extract number of frames, width, and height of video
    nFrames = vidObj.NumFrames;
    width = vidObj.Width;
    height = vidObj.Height;

    %unpack board_params
    marker_type = board_params.marker_type;
    markerFamily = board_params.markerFamily;
    patternDims = board_params.patternDims;
    checkerSize = board_params.checkerSize;
    markerSize = board_params.markerSize;

    %define reference point location for each grid corner in world frame
    worldPoints = patternWorldPoints(marker_type,patternDims,checkerSize);

    %locations of detected board points in the image frame
    imagePoints = zeros([size(worldPoints),0]);

    %choose a subset of (evenly spaced) frames to use
    frames_to_use = 1:ceil(nFrames/max_num_frames):nFrames;

    %used to determine when to print processing update
    prev_percent_complete = -1;

    %iterate through a subset of the video frames
    for n = 1:length(frames_to_use)

        %compute progress in percent
        percent_complete = floor(100*n/length(frames_to_use));

        %if the progress has incremented by 1%...
        if percent_complete ~= prev_percent_complete
            %then print the current progress
            disp(['Extracting Grid Points: ',num2str(percent_complete),'%']);
            prev_percent_complete = percent_complete;
        end
        
        %choose a video frame
        vidFrame = read(vidObj,frames_to_use(n));

        %detect grid points in the video image
        detected_points = detectCharucoBoardPoints(vidFrame,patternDims,markerFamily,checkerSize,markerSize);

        %if all of the gridpoints have been detected...
        if size(detected_points,1)==size(worldPoints,1)
            %add result to data matrix
            imagePoints(:,:,end+1) = detected_points;
        end
        
    end
    disp(' ');
    disp('Computing Camera Intrinsics');

    %turn off warning message
    warning('off','vision:calibrate:invalidHomographies');

    %estimate the camera parameters from the collected data
    [calibrated_camera_params,~,~] = estimateCameraParameters(imagePoints,worldPoints,'ImageSize',[height,width]);

    %turn warning message back on
    warning('on','vision:calibrate:invalidHomographies');

    %extract camera intrinsics from parameters
    camera_intrinsics = calibrated_camera_params.Intrinsics;

    disp(' ');
    disp('Calibration Complete!');
    disp(' ');
    disp('Intrinsic Matrix:');
    disp(' ');
    disp(camera_intrinsics.K);
end

