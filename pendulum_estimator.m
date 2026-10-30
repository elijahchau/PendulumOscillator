%used to estimate pose of the camera via apriltags
classdef pendulum_estimator < handle
    properties
        myGraph;                    %factor graph for pose estimation
        keypoint_node_dictionary;   %maps AprilTag keypoints to node ID's
        tag_counts;                 %number of times each tag was seen
        tag_centroid_dictionary;    %dictionary of tag centroid positions
        tag_centroid_mat;           %locations of tag centroids as a matrix
        frame_node_dictionary;      %maps image frames to node ID's
        frame_pose_dictionary;      %dictionary of image frame poses
        aprilTag_ID_set;            %set of AprilTag ID's
        aprilTag_frame_data;        %recorded AprilTag info
        anchor_node_ID;             %ID of origin node
        num_nodes;                  %number of nodes in the factor graph
        direction_vectors;          %list of +/- basis vectors
        keypoints_per_tag;          %number of keypoints per tag
        world_frame_mask;           %locations of mask points in world frame
        plane_pose;                 %frame define by pendulum oscillation plane
        camera_intrinsics;          %camera intrinsic matrix
        video_fname;                %file name of pendulum video
        tag_family;                 %tag family for april tags
        tagSize;                    %size of april tags
        canny_threshold;            %used for canny edge detection
        
        %true if a video frame node has been connected to origin anchor
        %via a relative pose. False otherwise
        starting_frame_anchored;    
    end

    methods

        %initialize the class
        function obj = pendulum_estimator()
            obj.myGraph = factorGraph();
            obj.keypoint_node_dictionary = dictionary([],[]);
            obj.tag_counts = dictionary([],[]);
            obj.tag_centroid_dictionary = dictionary([],{});
            obj.tag_centroid_mat = [];
            obj.frame_node_dictionary = dictionary([],[]);
            obj.frame_pose_dictionary = dictionary([],{});
            obj.aprilTag_ID_set = dictionary([],[]);
            obj.aprilTag_frame_data = dictionary([],{});
            obj.direction_vectors = [[0;0;0],eye(3),-eye(3)];

            obj.keypoints_per_tag = size(obj.direction_vectors,2);
            obj.world_frame_mask = [];
            obj.plane_pose = [];

            obj.camera_intrinsics = [];
            obj.video_fname = [];
            obj.tag_family = [];
            obj.tagSize = [];

            obj.num_nodes = 0;
            obj.starting_frame_anchored = 0;
            obj.canny_threshold = .1;

            obj.create_anchor_node();
        end

        %saves object properties to a file
        function save_to_file(obj,fname)
            keypoint_node_dictionary = obj.keypoint_node_dictionary;
            tag_counts = obj.tag_counts;
            tag_centroid_dictionary = obj.tag_centroid_dictionary;    
            tag_centroid_mat = obj.tag_centroid_mat;           
            frame_node_dictionary = obj.frame_node_dictionary;      
            frame_pose_dictionary = obj.frame_pose_dictionary;      
            aprilTag_ID_set = obj.aprilTag_ID_set;            
            aprilTag_frame_data = obj.aprilTag_frame_data;        
            anchor_node_ID = obj.anchor_node_ID;             
            num_nodes = obj.num_nodes;                  
            direction_vectors = obj.direction_vectors;          
            keypoints_per_tag = obj.keypoints_per_tag;          
            world_frame_mask = obj.world_frame_mask;           
            plane_pose = obj.plane_pose;                 
            starting_frame_anchored = obj.starting_frame_anchored;
            camera_intrinsics = obj.camera_intrinsics;
            video_fname = obj.video_fname;
            tag_family = obj.tag_family;
            tagSize = obj.tagSize;
            canny_threshold = obj.canny_threshold;

            save(fname,'keypoint_node_dictionary',...
                        'tag_counts',...
                        'tag_centroid_dictionary',...
                        'tag_centroid_mat',...
                        'frame_node_dictionary',...
                        'frame_pose_dictionary',...
                        'aprilTag_ID_set',...
                        'aprilTag_frame_data',...
                        'anchor_node_ID',...
                        'num_nodes',...
                        'direction_vectors',...
                        'keypoints_per_tag',...
                        'world_frame_mask',...
                        'plane_pose',...
                        'starting_frame_anchored',...
                        'camera_intrinsics',...
                        'video_fname',...
                        'tag_family',...
                        'tagSize');
        end

        %load object properties from a file
        function load_from_file(obj,fname)
            data = load(fname);

            obj.keypoint_node_dictionary = data.keypoint_node_dictionary;
            obj.tag_counts = data.tag_counts;
            obj.tag_centroid_dictionary = data.tag_centroid_dictionary;    
            obj.tag_centroid_mat = data.tag_centroid_mat;           
            obj.frame_node_dictionary = data.frame_node_dictionary;      
            obj.frame_pose_dictionary = data.frame_pose_dictionary;      
            obj.aprilTag_ID_set = data.aprilTag_ID_set;            
            obj.aprilTag_frame_data = data.aprilTag_frame_data;        
            obj.anchor_node_ID = data.anchor_node_ID;             
            obj.num_nodes = data.num_nodes;                  
            obj.direction_vectors = data.direction_vectors;          
            obj.keypoints_per_tag = data.keypoints_per_tag;          
            obj.world_frame_mask = data.world_frame_mask;           
            obj.plane_pose = data.plane_pose;                 
            obj.starting_frame_anchored = data.starting_frame_anchored;
            obj.camera_intrinsics = data.camera_intrinsics;
            obj.video_fname = data.video_fname;
            obj.tag_family = data.tag_family;
            obj.tagSize = data.tagSize;
        end

        function set_canny_threshold(obj,val_in)
            obj.canny_threshold = val_in;
        end

        %finds node ID of aprilTag keypoint. If no node ID exists,
        %then it creates a node ID and returns that value
        function node_ID = get_keypoint_node_ID(obj,tag_ID,keypoint_num)
            %record that a given tag was seen
            obj.aprilTag_ID_set(tag_ID) = 0;

            %compute index of aprilTag keypoint
            q = tag_ID*obj.keypoints_per_tag+mod(keypoint_num,obj.keypoints_per_tag);

            %check if node ID is stored in dictionary
            if ~isKey(obj.keypoint_node_dictionary,q)
                %if not, assign a new node, and store in the dictionary
                obj.num_nodes = obj.num_nodes+1;
                obj.keypoint_node_dictionary(q) = obj.num_nodes;
            end

            %return the node ID stored in dictionary
            node_ID = obj.keypoint_node_dictionary(q);
        end

        %finds node ID of image frame. If no node ID exists,
        %then it creates a node ID and returns that value
        function node_ID = get_frame_node_ID(obj,frame_num)
            %check if node ID is stored in dictionary
            if ~isKey(obj.frame_node_dictionary,frame_num)
                %if not, assign a new node, and store in the dictionary
                obj.num_nodes = obj.num_nodes+1;
                obj.frame_node_dictionary(frame_num) = obj.num_nodes;
            end

            %return the node ID stored in dictionary
            node_ID = obj.frame_node_dictionary(frame_num);
        end

        %creates anchor node
        function create_anchor_node(obj)
            obj.num_nodes = obj.num_nodes+1;
            obj.anchor_node_ID = obj.num_nodes;
            origin_prior = factorPoseSE3Prior(obj.anchor_node_ID);
            addFactor(obj.myGraph,origin_prior);
            
        end

        %moors an image frame to the anchor node
        function connect_frame_to_anchor(obj,frame_num)
            %look up the node_ID of the image frame
            node_ID = obj.get_frame_node_ID(frame_num);
            %creates a new factor in the graph to link frame to anchor
            mooring_factor = factorTwoPoseSE3([obj.anchor_node_ID, node_ID],Measurement=[0 0 0 1 0 0 0]);
            %adds the factor to the factor graph
            addFactor(obj.myGraph,mooring_factor);
            %indicate that a frame has been moored
            obj.starting_frame_anchored = 1;
        end


        %adds aprilTags to factor graph for current frame
        function add_tag_landmarks(obj,frame_num,use_tag_dict,tagSize)

            tag_struct = obj.aprilTag_frame_data(frame_num);
            tag_struct = tag_struct{1};

            %unpack tag_struct
            tag_ID_list =   tag_struct.tag_ID_list;
            tag_poses   =   tag_struct.tag_poses;

            %checks to see if there is at least one usable aprilTag
            %in the current image frame
            use_this_frame = 0;
            %iterate through each aprilTag
            for n = 1:numel(tag_ID_list)
                tag_ID = tag_ID_list(n);
                use_this_frame = use_this_frame || use_tag_dict(tag_ID);
            end
            
            %stop if there are no tags to add and frame is useless
            if ~use_this_frame
                return
            end

            %if an image frame has not been moored yet, do so.
            if obj.starting_frame_anchored == 0
                obj.connect_frame_to_anchor(frame_num);
            end

            %get the node ID of the current frame
            frame_node_ID = obj.get_frame_node_ID(frame_num);

            %keypoint positions in the tag frame
            pos_tag_frame = [(.5*tagSize)*obj.direction_vectors;ones(1,obj.keypoints_per_tag)];

            %iterate through each aprilTag
            for n = 1:numel(tag_ID_list)
                tag_ID = tag_ID_list(n); %get ID of current tag

                if ~use_tag_dict(tag_ID)
                    continue;
                end

                %keypoint positions in the observed frame
                keypoint_pos = tag_poses(n).A*pos_tag_frame;

                for m = 1:obj.keypoints_per_tag
                    %get the node ID of the aprilTag keypoint
                    keypoint_node_ID = obj.get_keypoint_node_ID(tag_ID,m);
                    %create a new factor between frame and the keypoint
                    keypoint_factor = factorPoseSE3AndPointXYZ([frame_node_ID, keypoint_node_ID],Measurement=keypoint_pos(1:3,m)');
                    %add factor to graph
                    addFactor(obj.myGraph,keypoint_factor);
                end
            end
        end

        %optimize the factor graph, and assign poses/positions
        %to all image frames and aprilTag centroids.
        function solve_graph(obj)
            solver_options = factorGraphSolverOptions(VerbosityLevel=0);
            optimize(obj.myGraph,solver_options);

            frame_num_list = keys(obj.frame_node_dictionary)';

            for frame_num = frame_num_list
                node_ID = obj.get_frame_node_ID(frame_num);
                Q = se3(nodeState(obj.myGraph, node_ID),'xyzquat');
                obj.frame_pose_dictionary(frame_num) = {tform(Q)};
            end

            tag_ID_list = keys(obj.aprilTag_ID_set)';
            num_tags = length(tag_ID_list);

            obj.tag_centroid_mat = zeros(3,num_tags);

            
            for n = 1:num_tags
                tag_ID = tag_ID_list(n);
                node_ID = obj.get_keypoint_node_ID(tag_ID,1);
                tag_centroid = nodeState(obj.myGraph, node_ID)';
                obj.tag_centroid_mat(:,n) = tag_centroid;
                obj.tag_centroid_dictionary(tag_ID) = {tag_centroid};
            end

            
        end

        %computes a set of world points corresponding to portion of image
        %that we want to actually use (where the pendulum is)
        function compute_world_frame_mask(obj,tagSize,q)
            
            num_tags = size(obj.tag_centroid_mat,2);
            centroid_location = mean(obj.tag_centroid_mat,2);

            for n = 1:num_tags
                v = centroid_location-obj.tag_centroid_mat(:,n);
                mask_pos = obj.tag_centroid_mat(:,n) + q*tagSize*(v/norm(v));
                obj.world_frame_mask(:,n) = mask_pos;
            end
        end

        %computes frame corresponding to plane of aprilTags
        function compute_plane_frame(obj)
            centroid_location = mean(obj.tag_centroid_mat,2);
            plane_vectors = pca(obj.tag_centroid_mat');
            plane_normal = plane_vectors(:,3);

            min_frame_num = min(keys(obj.frame_pose_dictionary));
            first_image_pose = obj.frame_pose_dictionary(min_frame_num);

            image_x_axis = first_image_pose{1}(1:3,1);
            image_z_axis = first_image_pose{1}(1:3,3);


            if dot(plane_normal,image_z_axis)>0
                plane_normal = -plane_normal;
            end

            plane_x_axis = image_x_axis-dot(image_x_axis,plane_normal)/norm(plane_normal);
            plane_x_axis = plane_x_axis/norm(plane_x_axis);

            plane_y_axis = cross(plane_normal,plane_x_axis);
            plane_y_axis = plane_y_axis/norm(plane_y_axis);

            R = [plane_x_axis,plane_y_axis,plane_normal];

            obj.plane_pose = [[R,centroid_location];[0,0,0,1]];
        end


        function [t_list,theta_list] = compute_pendulum_angles(obj,video_flag)
            %create a new VideoReader object to process the data
            vidObj = VideoReader(obj.video_fname);
        
            %extract number of frames, width, and height of video
            nFrames = vidObj.NumFrames;

            %extract frame rate
            Fs = vidObj.FrameRate;
            width = vidObj.Width;
            height = vidObj.Height;
            L = height+width;

            frame_list = 1:nFrames;

            theta_list = [];
            t_list = [];

            if video_flag(1)~=0
                fig1 = figure();

                pendulum_image = [];
                aprilTag_dots = [];

                if video_flag(1)==1
                    pendulum_image = imshow(read(vidObj,1));
                end

                if video_flag(1) == 2 || video_flag(1) == 3
                    pendulum_image = imshow(im2gray(read(vidObj,1)));
                end

                if video_flag(1) == 4
                    pendulum_image = imshow(edge(im2gray(read(vidObj,1)),'canny',obj.canny_threshold));
                end

                title('Pendulum Video with Line Fit');

                if isscalar(video_flag) || video_flag(2)~=0
                    hold on
                    pendulum_line_image = plot(0,0,'r','linewidth',3);
                end

                if ~isscalar(video_flag) && video_flag(2)==2
                    hold on
                    aprilTag_dots = plot(0,0,'ro','markerfacecolor','r','markersize',3);
                end

                fig2 = figure();
                hold on;
                theta_plot1 = plot(0,0,'b','linewidth',2);
                theta_plot2 = plot(0,0,'ro','markerfacecolor','r','markersize',4);
                plot([0,nFrames/Fs],[0,0],'k--','linewidth',2);
                xlabel('Time (sec)');
                ylabel('Angle (rad)');
                title('Pendulum Motion Visualization');
            end


            %used to determine when to print processing update
            prev_percent_complete = -1;

            for n = frame_list
                %compute progress in percent
                percent_complete = floor(100*n/nFrames);

                %if the progress has incremented by 1%...
                if percent_complete ~= prev_percent_complete
                    %then print the current progress
                    disp(['Pendulum Angle Extraction: ',num2str(percent_complete),'%']);
                    prev_percent_complete = percent_complete;
                end

                vidFrame = read(vidObj,n);

                if ~isKey(obj.frame_pose_dictionary,n)
                    continue
                end

                frame_pose = obj.frame_pose_dictionary(n);
                frame_pose = frame_pose{1};


                %use aprilTags to compute an image mask
                mask_polygon = compute_mask_polygon(obj,frame_pose);
                image_mask = ~poly2mask(mask_polygon(1,:),mask_polygon(2,:),height,width);
    
                %convert image to grayscale
                temp_image_greyscale = im2gray(vidFrame);
                
                %use canny edge detection to extract image edges
                temp_image = edge(temp_image_greyscale,'canny',obj.canny_threshold);

                %apply image mask to the detected edges
                temp_image(image_mask) = 0;

                %use hough transform to extract pendulum line
                [H,hough_theta,rho] = hough(temp_image);
                hough_thresh = ceil(0.3*max(H(:)));
                peaks = houghpeaks(H,1,Threshold=hough_thresh);
            
                lines = houghlines(temp_image,hough_theta,rho,peaks,FillGap=15,MinLength=60);
        
                if isempty(lines)
                    continue
                end

                %add in other canny points that might be useful here
                pendulum_line = lines(1);

                P1 = pendulum_line.point1';
                P2 = pendulum_line.point2';
                d = norm(P1-P2);
         
                pixel_index_list = find(temp_image);
            
                x_coord = ceil(pixel_index_list/height);
                y_coord = rem(pixel_index_list,height);   
            
                tan_vec = [(P2(1)-P1(1));(P2(2)-P1(2))]/d;
                perp_vec = [-(P1(2)-P2(2));(P1(1)-P2(1))]/d;
            
                perp_dist = abs((x_coord - P1(1))*perp_vec(1) + (y_coord - P1(2))*perp_vec(2));
                tan_dist = ((x_coord - P1(1))*tan_vec(1) + (y_coord - P1(2))*tan_vec(2))/d;

                tangential_weight = .2;
                tan_dist_sorted = sort(tan_dist);
                perp_dist_sorted =  sort(perp_dist);
            
                perp_thresh = 1.3*perp_dist_sorted(ceil(.7*length(perp_dist_sorted)));
                tan_thresh1 = tan_dist_sorted(ceil(tangential_weight*length(tan_dist_sorted)));
                tan_thresh2 = tan_dist_sorted(floor((1-tangential_weight)*length(tan_dist_sorted)));

                pixel_filter = (perp_dist<=perp_thresh).*(tan_dist>=tan_thresh1).*(tan_dist<=tan_thresh2);
                indices_to_use = find(pixel_filter);

                x_coord = x_coord(indices_to_use);
                y_coord = y_coord(indices_to_use);

                x_mean = mean(x_coord);
                y_mean = mean(y_coord);

                pendulum_frame_in_image = pca([x_coord,y_coord]);

                line_vec_image = pendulum_frame_in_image(:,1);

                line_vec_camera = obj.camera_intrinsics.K\[line_vec_image;0];
                line_vec_world = frame_pose*[line_vec_camera;0];
        
                line_plane_x = dot(line_vec_world,-obj.plane_pose(:,2));
                line_plane_y = dot(line_vec_world,obj.plane_pose(:,1));
        
                theta = atan2(line_plane_y,line_plane_x);

                theta_list(end+1) = theta;
                t_list(end+1) = n/Fs;

                if video_flag(1)~=0
                    set(0,'currentfigure',fig1);

                    if video_flag(1)==1
                        set(pendulum_image,'cdata',vidFrame);
                    end

                    if video_flag(1)==2
                        set(pendulum_image,'cdata',temp_image_greyscale);
                    end

                    if video_flag(1)==3
                        temp_image_greyscale(image_mask) = 0;
                        set(pendulum_image,'cdata',temp_image_greyscale);
                    end

                    if video_flag(1)==4
                        set(pendulum_image,'cdata',temp_image);
                    end

                    if isscalar(video_flag) || video_flag(2)~=0
                        set(pendulum_line_image,'xdata',x_mean+L*pendulum_frame_in_image(1,1)*[-1,1],...
                            'ydata',y_mean+L*pendulum_frame_in_image(2,1)*[-1,1]);
                    end

                    if ~isscalar(video_flag) && video_flag(2)==2
                        tag_struct = obj.aprilTag_frame_data(n);
                        tag_struct = tag_struct{1};
                        
                        xdots = []; ydots = [];

                        for tag_iter = 1:size(tag_struct.tag_loc,3)
                            xdots = [xdots;tag_struct.tag_loc(:,1,tag_iter)];
                            ydots = [ydots;tag_struct.tag_loc(:,2,tag_iter)];
                        end

                        set(aprilTag_dots,'xdata',xdots,'ydata',ydots);

                        % aprilTag_dots = plot(0,0,'ro','markerfacecolor','r','markersize',3);
                    end

                    set(0,'currentfigure',fig2);
                    set(theta_plot1,'xdata',t_list,'ydata',theta_list);
                    set(theta_plot2,'xdata',t_list,'ydata',theta_list);
                    xlim([0,40*ceil(n/40)/Fs])
                    ylim([min(theta_list)-.1,max(theta_list)+.1]);

                    drawnow;
                end

            end
        end

        function mask_polygon = compute_mask_polygon(obj,frame_pose)

            num_tags = size(obj.world_frame_mask,2);

            world_frame_mask_pose = [obj.world_frame_mask;ones(1,num_tags)];
            mask_pos_in_camera_frame = frame_pose\world_frame_mask_pose;
            mask_pos_in_image = obj.camera_intrinsics.K*mask_pos_in_camera_frame(1:3,:);

            mask_pos_list = [mask_pos_in_image(1,:)./mask_pos_in_image(3,:);...
                             mask_pos_in_image(2,:)./mask_pos_in_image(3,:)];


            mask_polygon = mask_pos_list(:,convhull(mask_pos_list'));

        end


        function identify_aprilTags(obj,fname,tag_family,camera_intrinsics,tagSize)
            obj.camera_intrinsics = camera_intrinsics;
            obj.video_fname = fname;
            obj.tag_family = tag_family;
            obj.tagSize = tagSize;

            %create a new VideoReader object to process the data
            vidObj = VideoReader(fname);
        
            %extract number of frames, width, and height of video
            nFrames = vidObj.NumFrames;

            frame_list = 1:nFrames;

            %used to determine when to print processing update
            prev_percent_complete = -1;

            

            for n = frame_list
                %compute progress in percent
                percent_complete = floor(100*n/nFrames);

                vidFrame = read(vidObj,n);
        
                [tag_ID_list,tag_loc,tag_poses] = readAprilTag(vidFrame,tag_family,camera_intrinsics,tagSize);
                
                %if the progress has incremented by 1%...
                if percent_complete ~= prev_percent_complete
                    %then print the current progress
                    disp(['AprilTag Tracking: ',num2str(percent_complete),'%']);
                    prev_percent_complete = percent_complete;
                end

                for tag_ID = tag_ID_list
                    if ~isKey(obj.tag_counts,tag_ID)
                        obj.tag_counts(tag_ID) = 0;
                    end

                    obj.tag_counts(tag_ID) = obj.tag_counts(tag_ID)+1;
                end

                %combine tag information in a struct
                tag_struct = struct();
                tag_struct.tag_ID_list = tag_ID_list;
                tag_struct.tag_loc     = tag_loc;
                tag_struct.tag_poses   = tag_poses;
    
                %store tag information in dictionary
                obj.aprilTag_frame_data(n) = {tag_struct};
            end

            max_tag_count = 0;

            for tag_ID = keys(obj.tag_counts)'
                max_tag_count = max(obj.tag_counts(tag_ID),max_tag_count);
            end

            use_tag_dict = dictionary([],[]);
 
            for tag_ID = keys(obj.tag_counts)'
                use_tag_dict(tag_ID) = obj.tag_counts(tag_ID)>=.1*max_tag_count;
            end

            for n = frame_list
                obj.add_tag_landmarks(n,use_tag_dict,tagSize);
            end

            obj.solve_graph();
            obj.compute_plane_frame();
            obj.compute_world_frame_mask(tagSize,1.5);
        end
        

    end

end