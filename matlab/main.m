close all;
clear all;

% for ROS nodes to communicate with the ROS master

% Used to connect to my laptop from my home PC - nick
setenv('ROS_MASTER_URI','http://192.168.0.251:11311')


%% Scan for, and find the center of the object
rosshutdown  %shuts down existing ROS nodes

scanner = Scanner();

% so far can be 'cube' for blue cube that loads default
% or can be 'coke' for coke can object
objectUsed = 'cube';


cokeOffsets = [0.055, 0.02, 0.05];
blueCubeOffsets = [0.07 0.01 0.05];

switch objectUsed
    case 'cube'
        objectOffset = blueCubeOffsets;
        loweringOffset = 0.22;
    case 'coke'
        objectOffset = cokeOffsets;
        loweringOffset = 0.2;
end

% locating position in 3D space of blue object
[X,Y,Z] = scanner.scan(objectUsed);

% Transform the 3D point's coordinates to them from the camera coordinate frame into the global coordinate frame
ptBaseLink = applyTransform([X,Y,Z]);
disp(ptBaseLink)

% adjusting the obtained readings of coordinates, as we take the bottom
% front corner, so adjust more towards the center to account for that
% taking from the front corner showed to be much more accurate and reliable
% these offsets are based on the objects size.
% THIS SHOULD BE AROUND THE CENTER OF THE OBJECT.

X = ptBaseLink(1) + objectOffset(1);
Y = ptBaseLink(2) - objectOffset(2);
Z = ptBaseLink(3) + objectOffset(3);

% messages displayed for user to be notified where the coordinates of the object have been found
disp("Location Found: ")
disp([X,Y,Z])

%% Exeecture robot path

% grippers begin by being open to pick the object up
moveGripper(1)
% To reduce chances of hitting the table so the body of the robot needs to risen by 0.7 within 3 seconds
moveFetchTorso(0.65, 3.0); 
pause(3);
% The offsets here are to account for the camera findings and firstly moving a little higher than the object 
moveToGoal(X, Y, Z + 0.28);
% The offsets are lowered a little to make sure the camera readings are in the in middle of the object
moveToGoal(X, Y, Z + loweringOffset);
% grippers are to close for object to be grasped
moveGripper(0);
% once the grippers are closed, the arm of the robot then lifts up again to bring the object up from the table 
moveToGoal(X, Y, Z + 0.3);


% this is the function created to move the robot arm to the desired
% position, using a moveit! python bride that communicates with ros
function moveToGoal(X, Y, Z)  

    % Shuts down any existing ROS nodes then initialises ROS for the function to begin with a clean environment 
    rosshutdown;
    rosinit;
  
    % Create a ROS publisher for the custom topic to actually send messages that have pose information to be used for the end effectors position and orientation 
    pub = rospublisher('/end_effector_pose', 'geometry_msgs/Pose');
    
    % Create the Pose message that will hold the data
    msg = rosmessage(pub);
    
    % Connecting the 3D coordinates of X,Y,Z to the desired end effector position
    msg.Position.X = X;
    msg.Position.Y = Y;
    msg.Position.Z = Z;
    % The orientation is also set, with the quaternion parameters W, X, Y, Z with w=1 to normalise the rotational vector and y=1 to ensure its always down 
    msg.Orientation.W = 1;
    msg.Orientation.X = 0;
    msg.Orientation.Y = 1;
    msg.Orientation.Z = 0;
    
    % Publish the message which contains the desired 3D position and orientation for the robots end effector which is sent to '/end_effector_pose/' for the robots control system to receive
    send(pub, msg);
    
    % gives the robot some time to process the message and execute movement needed so that the function doesnt return immediately  
    pause(10)
end


% Function used to move the body of the robot up and down given a specific time
function moveFetchTorso(targetAngle, moveDuration)

    % Shuts down any existing ROS nodes then initialises ROS for the function to begin with a clean environment 
    rosshutdown;
    rosinit;

    % Create a publisher for the torso trajectory action goal which is responsible for sending specific types of messages to the topic for controlling the torso
    pub = rospublisher('/torso_controller/follow_joint_trajectory/goal', 'control_msgs/FollowJointTrajectoryActionGoal');
    
    % Joint name for the torso lift which is required to specify the joint to be controlled during the robot's torso movement.
    jointNames = {'torso_lift_joint'};
    
    % Create a new message which holds the data initialised for the torso trajectory action goal.
    msg = rosmessage(pub);
    
    % Set goal ID which is  generated using the current date and time to ensure each movement has a distinct identifier for accuracy
    msg.GoalId.Id = sprintf('move_torso_%s', datestr(now, 'yyyymmddHHMMSSFFF'));
    msg.GoalId.Stamp = rostime('now');
    
    % Set joint name which is to be controlled within the ROS message
    msg.Goal.Trajectory.JointNames = jointNames;
    
    % Create a trajectory point for the target position where the positions property of this point is set to targetAngle (torso position)
    trajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    trajPoint.Positions = targetAngle;
    %This property indicates how long the movement of the torso should take, and it is set to moveDuration
    trajPoint.TimeFromStart = rosduration(moveDuration);  % Use full duration
    
    % Append the final trajectory point to the ROS message
    msg.Goal.Trajectory.Points = trajPoint;
    
    % Publish the trajectory goal - movement goal to ROS system
    send(pub, msg);
    
    % Pause for the given duration ensuring time is given to execute the movement plus a little buffer.
    pause(moveDuration + 3.0);  % Add a buffer to be sure
end


%function to move the grippers positions, whether they are closed when grasping or open 
function moveGripper(position)

    %Shuts down any existing ROS nodes then initialises ROS for the function to begin with a clean environment 
    rosshutdown;
    rosinit;

    % Create a publisher for the gripper action goal  which is responsible for sending specific messages to the topic for controlling the grippers 
    [gripPub, gripMsg] = rospublisher('/gripper_controller/gripper_action/goal', 'control_msgs/GripperCommandActionGoal');

    % Wait for publisher to be ready - ensures that there are nodes in the ROS environment that are ready to receive the gripper control messages before proceeding
    while gripPub.NumSubscribers == 0
        pause(0.1);
    end
    
    % Set desired position (0.0 for fully closed) and force
    gripMsg.Goal.Command.Position = position;
    gripMsg.Goal.Command.MaxEffort = 50.0; 

    % Publish the message to close the gripper
    send(gripPub, gripMsg);

    %This pause allows the gripper to execute the command and achieve the desired position whether opened or closed which takes roughly 2 seconds
    pause(2)

    % TODO: Wait and check for feedback/result from the gripper action server
end

%function is responsible for moving a robot arm to a series of joint angles with a time taken into account 
function moveFetchArm(jointNames, jointAnglesList, moveDuration)

    %Shuts down any existing ROS nodes then initialises ROS for the function to begin with a clean environment 
    rosshutdown;
    rosinit;

    % Create a publisher to send messages to the arm trajectory action goal.
    pub = rospublisher('/arm_controller/follow_joint_trajectory/goal', 'control_msgs/FollowJointTrajectoryActionGoal');

    % Number of movements which is useful for iterating through the list of desired joint angles and executing the trajectory
    numMovements = size(jointAnglesList, 1);
    for i = 1:numMovements % the loop iterates over each movement defined in jointAnglesList
        % Create a new message- this message will be used to specify the trajectory for the robot arm
        msg = rosmessage(pub);

        % Set goal ID and time stamp - this ensures that each movement is identified separately and can be tracked
        msg.GoalId.Id = sprintf('move_%d_%s', i, datestr(now, 'yyyymmddHHMMSSFFF'));
        msg.GoalId.Stamp = rostime('now');
        
        % Set joint names which just indicates which joints of the robot arm will be involved in the trajectory
        msg.Goal.Trajectory.JointNames = jointNames;

        % If this isn't the first movement, get halfway points for smoother movement
        if i > 1

            %calculates the halfway position by averaging the joint angles of the current and previous movements for a smoother trajectory when transitioning between movements
            halfWayAngles = (jointAnglesList(i-1, :) + jointAnglesList(i, :)) / 2;
            % Create a trajectory point for halfway, the calculated halfway angles are set and half of the specified duration is set
            halfWayTrajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            halfWayTrajPoint.Positions = halfWayAngles;
            halfWayTrajPoint.TimeFromStart = rosduration(moveDuration/2);
            
            % Append the halfway trajectory point to the message
            msg.Goal.Trajectory.Points = halfWayTrajPoint;
        end
        
        % Create a trajectory point for final position, which includes the full joint angle list and the total time
        trajPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        trajPoint.Positions = jointAnglesList(i, :);
        trajPoint.TimeFromStart = rosduration(moveDuration);  % Use full duration
        
        % Append the final trajectory point to the message
        msg.Goal.Trajectory.Points(end+1) = trajPoint;
        % Publish the trajectory goal through the ROS publisher which instructs the robot arm to move
        send(pub, msg);
        % Pause for the given duration plus a little buffer.
        pause(moveDuration + 5.0);  % Add more buffer to be sure
    end
end

% this is a debugging function used to find errors in joint positions by displaying them 
function angles = getJointStates(debug)
    pause(5) % allows for ROS nodes and topics to initalise
    jointStateSubscriber = rossubscriber('/joint_states'); %topic typically publishes information about the joint states of a robot
    % Receive data from the topic
    jointStateData = receive(jointStateSubscriber,10); % 10 seconds timeout for data to arrive 
    
    % Extract joint names and positions
    jointNames = jointStateData.Name;
    jointPositions = jointStateData.Position;
    if (debug)
        % This iterates through the joint names and positions, displaying them which provides a convenient way to inspect joint states for monitoring
        for i = 1:length(jointNames)
            disp([jointNames{i} ': ' num2str(jointPositions(i))]);
        end
    end
    
    % The function returns these joint positions as output, which can be used for further processing 
    angles = jointPositions;

end


% this function gets the transformation of the camera relative to the base
function transformedPoint = applyTransform(pt)
    % Shuts down any existing ROS nodes then initialises ROS for the function to begin with a clean environment 
    rosshutdown;
    rosinit;
        
    % for transformations in ROS
    tftree = rostf; 

    % Pause to allow for ROS initialisation
    pause(2); 
    
    try
        % time is given to ensures the lastest transform is obtained
        waitForTransform(tftree, 'base_link', 'head_camera_depth_optical_frame', 10);
        %this actually retrieves thes the transformation of the camera relative to the base
        transform = getTransform(tftree, 'base_link', 'head_camera_depth_optical_frame');

        % Extract rotation and translation components from the transformation (camera frame of reference coordinates to the base)
        quat = [transform.Transform.Rotation.W, transform.Transform.Rotation.X, ...
                transform.Transform.Rotation.Y, transform.Transform.Rotation.Z];
        translation = [transform.Transform.Translation.X, transform.Transform.Translation.Y, ...
                       transform.Transform.Translation.Z];
    
        % Convert quaternion to rotation matrix 
        rotm = quat2rotm(quat);
    
        % Apply rotation and translation to the input point 
        transformedPoint = (rotm * pt')' + translation;
    catch ME
        % Handle exceptions in case the transform is not available or times out
        disp('Transform not available or timed out.');
        transformedPoint = 0;
        return;
    end
    
end

