classdef Scanner
    properties
        % used to receive image and depth data from the robot's RGB-D camera
        imgSub
        depthSub
        % these properties represent the current tilt angle and the control of increment or decrement in tilt angle 
        tiltAngle = -0.5
        deltaTilt = 0.2
        %properties stored for camera intrinsic parameters related to the camera's field of view and calibration.
        fx = 554.254691191187
        fy = 554.254691191187
        cx = 320.5
        cy = 240.5
    end
    
    methods
        function obj = Scanner(rosMasterURI) 
            %setenv('ROS_MASTER_URI', rosMasterURI);
            rosinit; % This is essential for establishing a connection with the ROS master and initialising ROS nodes.
            obj.moveCamera(0, 0.3, 1);  % This instructs the camera to move with a pan angle of 0, a tilt angle of 0.3, and a movement duration of 1 second setting the initial camera position
            obj.imgSub = rossubscriber('/head_camera/rgb/image_raw');  % subscriber used to receive raw RGB image data from the robot for image processing
            obj.depthSub = rossubscriber('/head_camera/depth_registered/image_raw'); % subscriber used to receive raw depth image data from the corresponding RGB image for depth-based object detection
        end
        
        % the function  and returns X,Y,Z representing the camera coordinates of a detected colour object.
        function [X,Y,Z] = scan(obj, objectUsed) 
            found = false; % helps to track whether a colour object has been detected

            % loop begins until the colour object is found
            while ~found
                disp('Scanning...'); %this message is displayed in the command window to indicate that the scanning process is ongoing
                obj.moveCamera(0.0, obj.tiltAngle, 0.5); %the pan is left the same, but the tilt is adjusted given a movement time of 0.5 seconds
                pause(0.8); %for stabilsation and also given time to capture the image
                
                % 10 seconds are given to receive the image message from the subscribe, to garuntee something is sent 
                imgMsg = receive(obj.imgSub, 10);
                img = readImage(imgMsg); %reads the image data from the received message, making the image data available for processing
                imshow(img) %  the received image is displayed, providing a visual representation of what the camera sees
                % this analyses the image to detect colour objects and returns multiple values: found (detection status), and pixel positions of the detected object
                [found, leftPixelColumn, rightPixelColumn, topPixelRow, bottomPixelRow] = obj.detectObject(img, objectUsed);

                if found %checks whether a colour object has been found
                    foundImage = img; %If a colour object is found, it stores the current image and this is done to keep the first found image, which can be replaced if a better one is found
                    obj.moveCamera(0.0, obj.tiltAngle + obj.deltaTilt, 1); %The camera is adjusted to change its tilt angle by adding deltaTilt to the current tilt angle to capture a different perspective
                    pause(1);%for stabilsation and also given time to capture the image

                    % See if we can get a better view by looking down more
                    imgMsg = receive(obj.imgSub, 10); 
                    newImg = readImage(imgMsg);%reads the image data from the received message, making the image data available for processing
                    % this analyses the image to detect colour objects and returns multiple values: foundNew (detection status), and pixel positions of the detected object
                    [foundNew, leftPixelColumn, rightPixelColumn, topPixelRow, bottomPixelRow] = obj.detectObject(newImg, objectUsed); 

                    if foundNew %checking if a has a new image been found 
                        disp('Found Better Image..'); %indicates that a better image has been found
                        foundImage = newImg; % It updates the  variable with the new and better image as it has been found.
                        break;
                    end
                    disp('Keeping first found Image..'); %the first image is kept if no better image is found
                    break;
                end

                obj.tiltAngle = obj.updateTiltAngle(); %tiltAngle is updated accordingly
            end

            % when a good image is found, it traingulates the image coordinates based on the pixel positions of the object in the image and returns these coordinates.
            [X,Y,Z] = obj.computeCoordinates(leftPixelColumn, rightPixelColumn, topPixelRow, bottomPixelRow, objectUsed);
        end
        
        %function that controls the movement of the RGB-D camera
        function moveCamera(~, pan, tilt, time)  
            % Create a publisher to send messages to the arm trajectory action goal.
            pub = rospublisher('/head_controller/follow_joint_trajectory/goal', 'control_msgs/FollowJointTrajectoryActionGoal');
            
            msg = rosmessage(pub); % This message will be populated with data to control the camera movement.
 
            msg.Goal.Trajectory.JointNames = {'head_pan_joint', 'head_tilt_joint'}; % the ROS message, indicates the joint names representing the pan and tilt joints of the camera
            point = rosmessage('trajectory_msgs/JointTrajectoryPoint'); % this ROS message defines a specific position (pan and tilt) for the camera
            point.Positions = [pan,tilt]; %trajectory point message which is set with the desired pan and tilt angles, that indicates the camera's desired orientation
            point.TimeFromStart = rosduration(time); % 1 second, which is how long it should take to move the camera to the desired position
            msg.Goal.Trajectory.Points = point; % The trajectory point which describes the desired motion of the camera      
            
            pause(time) % This is done to allow the camera to move and stabilise at the desired position
            send(pub, msg); % This will result in a command being sent to the robot's camera controller in ROS to move the camera

        end
        
        %this function is for image processing and is in reference to scanner and an image which is the input image to be processed 
        function [found, leftPixelColumn, rightPixelColumn, topPixelRow, bottomPixelRow] = detectObject(obj, img, objectUsed)
            %These arrays specify the lower and upper bounds for colour colour in the RGB colour space. 

            switch objectUsed
                case 'cube'
                    lowerThreshold = [0, 0, 100];
                    upperThreshold = [80, 80,255];
                case 'coke'
                    lowerThreshold = [40, 0, 0];
                    upperThreshold = [80, 30,30];
            end

            
            %This binary mask checks whether each pixel in the input image falls within the defined lower and upper bounds for colour in the RGB channels and if it is, it's considered a valid pixel
            mask = (img(:,:,1) >= lowerThreshold(1) & img(:,:,1) <= upperThreshold(1)) & ...
                       (img(:,:,2) >= lowerThreshold(2) & img(:,:,2) <= upperThreshold(2)) & ...
                       (img(:,:,3) >= lowerThreshold(3) & img(:,:,3) <= upperThreshold(3));
            
            %The code finds the boundaries of colour regions within the binary mask, by finding the first and last columns and rows that contain colour pixels
            leftPixelColumn = find(any(mask, 1), 1, 'first');
            rightPixelColumn = find(any(mask, 1), 1, 'last');
            topPixelRow = find(any(mask, 2), 1, 'first');
            bottomPixelRow = find(any(mask, 2), 1, 'last');
            
            found = false; %checking if colour pixels are detected
            %it verifies if the boundaries of colour pixels within the image have been found
            if ~isempty(leftPixelColumn) && ~isempty(rightPixelColumn) && ~isempty(topPixelRow) && ~isempty(bottomPixelRow) 
                fprintf('Correctly coloured pixels detected. Trying 1 more.\n'); %message is printed when there are colour pixels detected 
                found = true; %updating the variable
                imshow(img); %display the image 
            end

        end
        
        %function for updating the tilt angle
        function tiltAngle = updateTiltAngle(obj)
            %making sure that when incrementing or decrementing the tilt angle it is done correctly (for negative values or positive values) 
            if obj.tiltAngle >= 0.5
                obj.deltaTilt = -0.1;
            elseif obj.tiltAngle <= -0.5
                obj.deltaTilt = 0.1;
            end

            %this is for continous scanning to detect the object by incrementing or decrementing the tilt angle
            tiltAngle = obj.tiltAngle + obj.deltaTilt
        end
        
        %this function uses the boundary data of the colour pixels to find the center to mark a point 
        function [X,Y,Z] = computeCoordinates(obj, leftPixelColumn, rightPixelColumn, topPixelRow, bottomPixelRow, objectUsed)
            %This calculates the center position in terms of both columns and rows based on the provided from previous pixel boundaries 
            centerColumn = (leftPixelColumn + rightPixelColumn) / 2;

            switch objectUsed
                case 'coke'
                    centerRow = (bottomPixelRow + topPixelRow) / 2;

                case 'cube'
                    centerRow = (bottomPixelRow);
            end
          
      
            depthMsg = receive(obj.depthSub, 10); %10 seconds are given to receive the image message from the subscriber for processing
            depthImage = readImage(depthMsg); %reads the image data from the received message, making the image data available for processing
            depthAtCenter = depthImage(round(centerRow), round(centerColumn)); %This retrieves the depth information at the calculated center position from the depth image.
            
            % Peform Triangulation to convert pixel cooridantes to camera coordinates         
            % X, Y, and Z are calculated based on the depth information and the camera's intrinsic parameters. 
            X = depthAtCenter * (centerColumn - obj.cx) / obj.fx;
            Y = depthAtCenter * (centerRow - obj.cy) / obj.fy;
            Z = depthAtCenter;
            
            hold on;
            plot(centerColumn, centerRow, 'ro', 'MarkerSize', 10); %to visualise the detected objects centre as a red circle making it easier to see 
            hold off;
        end
    end
end
