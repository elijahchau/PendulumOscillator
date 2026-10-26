%example for how to call pendulum estimation code
function pendulum_estimator_usage_example()
    
    %folder where your video is located
    %make sure that video_path ends with '\'
    video_path = 'C:\Users\taylorott\Desktop\pendulum_videos\';
    %file name of the pendulum video
    video_fname = 'pendulum_video01.mov';

    %folder where camera intrinsics is located
    camera_calibration_path = 'C:\Users\taylorott\Documents\';
    %file name of camera calibration data
    camera_calibration_fname = 'camera_intrinsics01.mat';

    %load calibrated camera intrinsics from file
    calibration_data = load([camera_calibration_path,camera_calibration_fname]);
    camera_intrinsics = calibration_data.camera_intrinsics;

    %folder for where to save/load pendulum intermediate data
    pendulum_temp_path = 'C:\Users\taylorott\Documents\';
    %file name of intermediate data for pendulum estimator
    pendulum_temp_fname = 'pendulum_temp.mat';

    %folder where pendulum data will be stored
    pendulum_data_path = 'C:\Users\taylorott\Documents\';
    %file name of pendulum data
    pendulum_fname = 'pendulum_data01.mat';

    
    %size of aprilTags (length of black in black square) in mm
    tagSize = 41.5;
    %family of aprilTags used in the video
    tagFamily = 'tag36h11';

    %create an instance of the pendulum estimator class
    myEstimator = pendulum_estimator();

    %flag used to toggle video on/off
    %flag can have up to two elements
    %First element options
    %0: display nothing
    %1: show color image
    %2: show grayscale image
    %3: show grayscale image w/mask determined by apriltags
    %4: show boundary lines of grayscale image
    %Second element options
    %0: no fit
    %any other value: show a fit
    %if there is no second element, fit is shown by default
    video_flag = [1,1];
    
    %Preprocess the pendulum video
    %COMMENT THIS BLOCK OUT AFTER IT SUCCESSFULLY RUNS ON THE VIDEO
    %Identify aprilTag positions in the video
    myEstimator.identify_aprilTags([video_path,video_fname],tagFamily,camera_intrinsics,tagSize);
    %save intermediate state of estimator to a file
    myEstimator.save_to_file([pendulum_temp_path,pendulum_temp_fname]);

    %Process the pendulum video
    %load intermediate state of estimator from a file
    myEstimator.load_from_file([pendulum_temp_path,pendulum_temp_fname]);
    %estimate the angle of pendulum as a function of time
    [t_list,theta_list] = myEstimator.compute_pendulum_angles(video_flag);
    %save result in file
    save([pendulum_data_path,pendulum_fname],'t_list','theta_list');
    
    %plot results
    figure();
    hold on;
    plot(t_list,theta_list,'b','linewidth',2);
    plot(t_list,theta_list,'ro','markerfacecolor','r','markersize',4);
    title('Measured Pendulum Angle as a Function of Time');
    xlabel('Time (seconds)');
    ylabel('Angle (radians)');
end


