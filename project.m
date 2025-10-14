function UR5_PickPlace_GUI()
    % Initialize Robot
    robot = loadrobot("universalUR5","DataFormat","row","Gravity",[0 0 -9.81]);
    
    % Create Control GUI with dark theme
    fig = figure('Name', 'UR5 Pick and Place Control Panel', 'Position', [100 100 520 900], ...
        'MenuBar', 'none', 'NumberTitle', 'off', 'Color', [0.15 0.15 0.18], ...
        'Resize', 'off');
    
    % Create control panel
    controlPanel = uipanel('Parent', fig, 'Position', [0.04 0.02 0.92 0.96], ...
        'Title', 'Control Panel', 'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.2 0.24], 'ForegroundColor', [0.9 0.9 0.9]);
    
    yPos = 0.93;
    spacing = 0.055;
    
    % Title
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', 'UR5 Pick and Place Configuration', ...
        'FontSize', 14, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'ForegroundColor', [0.9 0.9 0.95], ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.04], ...
        'HorizontalAlignment', 'center');
    yPos = yPos - 0.07;
    
    % Number of Objects
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', 'Number of Objects:', ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'ForegroundColor', [0.85 0.85 0.9], ...
        'FontSize', 11, ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.5 0.04], ...
        'HorizontalAlignment', 'left');
    numObjEdit = uicontrol('Parent', controlPanel, 'Style', 'edit', ...
        'String', '3', ...
        'BackgroundColor', [0.25 0.25 0.3], ...
        'ForegroundColor', [0.9 0.9 0.9], ...
        'FontSize', 11, ...
        'Units', 'normalized', 'Position', [0.6 yPos 0.35 0.04]);
    yPos = yPos - spacing;
    
    % Speed Control with Slider
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', 'Animation Speed:', ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'ForegroundColor', [0.85 0.85 0.9], ...
        'FontSize', 11, ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.5 0.035], ...
        'HorizontalAlignment', 'left');
    speedValueLabel = uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', '5x', ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'ForegroundColor', [0.4 0.8 1], ...
        'FontSize', 11, 'FontWeight', 'bold', ...
        'Units', 'normalized', 'Position', [0.6 yPos 0.35 0.035], ...
        'HorizontalAlignment', 'center');
    yPos = yPos - 0.045;
    
    speedSlider = uicontrol('Parent', controlPanel, 'Style', 'slider', ...
        'Min', 1, 'Max', 10, 'Value', 5, ...
        'BackgroundColor', [0.25 0.25 0.3], ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.035], ...
        'Callback', @(src,evt) set(speedValueLabel, 'String', sprintf('%.1fx', get(src, 'Value'))));
    yPos = yPos - spacing;
    
    % Object Configuration Table
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', 'Objects Configuration:', ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'ForegroundColor', [0.85 0.85 0.9], ...
        'FontSize', 11, ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.035], ...
        'HorizontalAlignment', 'left');
    yPos = yPos - 0.045;
    
    % Create table with dark theme
    columnNames = {'Pick X', 'Pick Y', 'Pick Z', 'Place X', 'Place Y', 'Place Z'};
    initialData = [0.4, 0.3, 0.1, -0.4, 0.3, 0.1; ...
                   0.5, -0.2, 0.05, -0.3, -0.4, 0.05; ...
                   0.3, 0.4, 0.08, -0.5, 0.2, 0.08];
    
    objectTable = uitable('Parent', controlPanel, ...
        'ColumnName', columnNames, ...
        'ColumnEditable', true, ...
        'Data', initialData, ...
        'BackgroundColor', [0.25 0.25 0.3; 0.22 0.22 0.27], ...
        'ForegroundColor', [0.9 0.9 0.9], ...
        'Units', 'normalized', ...
        'Position', [0.05 yPos-0.24 0.9 0.28]);
    yPos = yPos - 0.26;
    
    % Add/Remove Buttons
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Add Object', ...
        'BackgroundColor', [0.3 0.5 0.7], ...
        'ForegroundColor', [1 1 1], ...
        'FontWeight', 'bold', ...
        'FontSize', 10, ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.43 0.045], ...
        'Callback', @addObject);
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Remove Object', ...
        'BackgroundColor', [0.7 0.3 0.3], ...
        'ForegroundColor', [1 1 1], ...
        'FontWeight', 'bold', ...
        'FontSize', 10, ...
        'Units', 'normalized', 'Position', [0.52 yPos 0.43 0.045], ...
        'Callback', @removeObject);
    yPos = yPos - spacing;
    
    % Preset Buttons
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', 'Quick Presets:', ...
        'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'ForegroundColor', [0.85 0.85 0.9], ...
        'FontSize', 11, ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.035], ...
        'HorizontalAlignment', 'left');
    yPos = yPos - 0.045;
    
    btnColor = [0.35 0.35 0.42];
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Line Pattern', ...
        'BackgroundColor', btnColor, ...
        'ForegroundColor', [0.9 0.9 0.9], ...
        'FontSize', 10, ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.43 0.045], ...
        'Callback', @(src,evt) loadPreset('line'));
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Circle Pattern', ...
        'BackgroundColor', btnColor, ...
        'ForegroundColor', [0.9 0.9 0.9], ...
        'FontSize', 10, ...
        'Units', 'normalized', 'Position', [0.52 yPos 0.43 0.045], ...
        'Callback', @(src,evt) loadPreset('circle'));
    yPos = yPos - 0.05;
    
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Grid Pattern', ...
        'BackgroundColor', btnColor, ...
        'ForegroundColor', [0.9 0.9 0.9], ...
        'FontSize', 10, ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.43 0.045], ...
        'Callback', @(src,evt) loadPreset('grid'));
    uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'Random Pattern', ...
        'BackgroundColor', btnColor, ...
        'ForegroundColor', [0.9 0.9 0.9], ...
        'FontSize', 10, ...
        'Units', 'normalized', 'Position', [0.52 yPos 0.43 0.045], ...
        'Callback', @(src,evt) loadPreset('random'));
    yPos = yPos - spacing;
    
    % Status Label
    statusLabel = uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', 'Ready to start simulation', ...
        'ForegroundColor', [0.3 0.9 0.5], ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'FontSize', 11, ...
        'FontWeight', 'bold', ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.045], ...
        'HorizontalAlignment', 'center');
    yPos = yPos - 0.055;
    
    % Start Button
    startBtn = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'START SIMULATION', ...
        'FontSize', 13, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.6 0.3], ...
        'ForegroundColor', [1 1 1], ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.06], ...
        'Callback', @startSimulation);
    yPos = yPos - 0.07;
    
    % Reset Button
    resetBtn = uicontrol('Parent', controlPanel, 'Style', 'pushbutton', ...
        'String', 'RESET SIMULATION', ...
        'FontSize', 13, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.7 0.4 0.2], ...
        'ForegroundColor', [1 1 1], ...
        'Units', 'normalized', 'Position', [0.05 yPos 0.9 0.06], ...
        'Callback', @resetSimulation, ...
        'Enable', 'off');
    yPos = yPos - 0.08;
    
    % Footer
    uicontrol('Parent', controlPanel, 'Style', 'text', ...
        'String', {'This micro-project was developed as part of the MDM course:', ...
                   'Fundamentals of Robotics, instructed by Prof. Salaba Jacob.', ...
                   'Developed by Anvay Mayekar (SYECS1 - 26)', ...
                   }, ...
        'FontSize', 9, ...
        'BackgroundColor', [0.2 0.2 0.24], ...
        'ForegroundColor', [0.65 0.65 0.7], ...
        'Units', 'normalized', 'Position', [0.05 0 0.9 0.06], ...
        'HorizontalAlignment', 'center');
    
    % Simulation window handle
    simFig = [];
    simAx = [];
    positionText = [];
    isSimulationRunning = false;
    shouldStop = false;
    
    %% Callback Functions
    function addObject(~, ~)
        currentData = objectTable.Data;
        newRow = [0.4, 0.3, 0.1, -0.4, 0.3, 0.1];
        objectTable.Data = [currentData; newRow];
        numObjEdit.String = num2str(size(objectTable.Data, 1));
    end
    
    function removeObject(~, ~)
        currentData = objectTable.Data;
        if size(currentData, 1) > 1
            objectTable.Data = currentData(1:end-1, :);
            numObjEdit.String = num2str(size(objectTable.Data, 1));
        end
    end
    
    function loadPreset(pattern)
        n = str2double(numObjEdit.String);
        if isnan(n) || n < 1
            n = 3;
            numObjEdit.String = '3';
        end
        
        switch pattern
            case 'line'
                pickPos = [(0.3:0.15:0.3+0.15*(n-1))', repmat([0.3, 0.1], n, 1)];
                placePos = [(-0.3:-0.15:-0.3-0.15*(n-1))', repmat([0.3, 0.1], n, 1)];
            case 'circle'
                theta = linspace(0, 2*pi*(n-1)/n, n)';
                r = 0.35;
                pickPos = [r*cos(theta), r*sin(theta), repmat(0.1, n, 1)];
                placePos = [-r*cos(theta), -r*sin(theta), repmat(0.1, n, 1)];
            case 'grid'
                cols = ceil(sqrt(n));
                [X, Y] = meshgrid(linspace(0.3, 0.5, cols), linspace(0.2, 0.4, cols));
                pickPos = [X(1:n)', Y(1:n)', repmat(0.1, n, 1)];
                [X2, Y2] = meshgrid(linspace(-0.5, -0.3, cols), linspace(-0.4, -0.2, cols));
                placePos = [X2(1:n)', Y2(1:n)', repmat(0.1, n, 1)];
            case 'random'
                pickPos = [0.3 + 0.2*rand(n,1), 0.2 + 0.2*rand(n,1), 0.05 + 0.1*rand(n,1)];
                placePos = [-0.3 - 0.2*rand(n,1), -0.2 - 0.2*rand(n,1), 0.05 + 0.1*rand(n,1)];
        end
        objectTable.Data = [pickPos, placePos];
    end
    
    function resetSimulation(~, ~)
        shouldStop = true;
        if ishandle(simFig)
            close(simFig);
        end
        simFig = [];
        simAx = [];
        positionText = [];
        isSimulationRunning = false;
        shouldStop = false;
        statusLabel.String = 'Simulation reset - Ready to start';
        statusLabel.ForegroundColor = [0.3 0.9 0.5];
        resetBtn.Enable = 'off';
        startBtn.Enable = 'on';
    end
    
    function startSimulation(~, ~)
        if isSimulationRunning
            return;
        end
        
        isSimulationRunning = true;
        shouldStop = false;
        resetBtn.Enable = 'on';
        startBtn.Enable = 'off';
        
        % Get parameters
        data = objectTable.Data;
        numObjects = size(data, 1);
        pickPositions = data(:, 1:3);
        placePositions = data(:, 4:6);
        
        % Get speed from slider
        speedVal = get(speedSlider, 'Value');
        animSpeed = 0.02 / (speedVal^1.5);
        numSteps = max(20, round(80 / speedVal));
        
        statusLabel.String = 'Opening simulation window...';
        statusLabel.ForegroundColor = [1 0.7 0.3];
        drawnow;
        
        % Create simulation window
        simFig = figure('Name', 'UR5 Multi-Object Pick and Place', ...
            'Position', [650 100 900 900]);
        simAx = show(robot);
        title('Robotic Arm Simulation for Pick-and-Place Tasks');
        hold on;
        view(135, 20);
        grid on;
        
        % FIXED: Set LARGE cubic workspace FIRST before anything else
        % UR5 robot dimensions: base at origin, max reach ~0.85m, height ~0.89m
        cubeSize = 2.0;  % 2 meter cube - large enough for everything
        xlim(simAx, [-cubeSize/2, cubeSize/2]);
        ylim(simAx, [-cubeSize/2, cubeSize/2]);
        zlim(simAx, [-cubeSize/2, cubeSize/2]);
        
        % CRITICAL: Force EQUAL aspect ratio - this is the key!
        pbaspect(simAx, [1 1 1]);  % Plot box aspect ratio
        daspect(simAx, [1 1 1]);   % Data aspect ratio
        axis(simAx, 'vis3d');      % Freeze aspect ratio during rotation
        
        % Enable full 3D interaction
        rotate3d(simAx, 'on');
        zoom on;
        pan on;
        
        % Create position display text
        positionText = annotation('textbox', [0.02, 0.88, 0.18, 0.1], ...
            'String', sprintf('End-Effector Position:\nX: 0.000 m\nY: 0.000 m\nZ: 0.000 m'), ...
            'FontSize', 10, 'FontWeight', 'bold', ...
            'Color', [1 1 1], ...
            'BackgroundColor', [0.15 0.15 0.15], ...
            'EdgeColor', [0.5 0.5 0.5], ...
            'LineWidth', 1.5, ...
            'FitBoxToText', 'off');
        
        statusLabel.String = 'Simulation in progress...';
        drawnow;
        
        % Setup
        pathColors = lines(numObjects);
        ik = inverseKinematics('RigidBodyTree', robot);
        weights = [0.25 0.25 0.25 1 1 1];
        endEffector = robot.BodyNames{end};
        homeConfig = robot.homeConfiguration;
        hoverHeight = 0.15;
        
        % Main simulation loop with smooth transitions
        for obj = 1:numObjects
            if shouldStop
                break;
            end
            
            pickPos = pickPositions(obj, :);
            placePos = placePositions(obj, :);
            
            % Show markers
            plot3(simAx, pickPos(1), pickPos(2), pickPos(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
            plot3(simAx, placePos(1), placePos(2), placePos(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
            
            % Create cube
            cubeSize = 0.05;
            [v, f] = createCube(cubeSize);
            v = v + pickPos;
            cubePatch = patch('Parent', simAx, 'Vertices', v, 'Faces', f, ...
                'FaceColor', pathColors(obj,:), 'EdgeColor', 'k');
            
            % Define waypoints WITH transition from previous place position
            pickPose = trvec2tform(pickPos);
            placePose = trvec2tform(placePos);
            abovePick = trvec2tform(pickPos + [0 0 hoverHeight]);
            abovePlace = trvec2tform(placePos + [0 0 hoverHeight]);
            
            % Build waypoints - include transition if not first object
            if obj == 1
                % First object: start from above pick
                cartWaypoints = cat(3, abovePick, pickPose, abovePick, abovePlace, placePose, abovePlace);
            else
                % Subsequent objects: transition from previous place position to new pick
                prevPlacePos = placePositions(obj-1, :);
                abovePrevPlace = trvec2tform(prevPlacePos + [0 0 hoverHeight]);
                cartWaypoints = cat(3, abovePrevPlace, abovePick, pickPose, abovePick, abovePlace, placePose, abovePlace);
            end
            
            % Solve IK
            configs = zeros(size(cartWaypoints, 3), numel(homeConfig));
            for i = 1:size(cartWaypoints, 3)
                [config, ~] = ik(endEffector, cartWaypoints(:,:,i), weights, homeConfig);
                configs(i,:) = config;
                homeConfig = config;
            end
            
            % Generate trajectory
            qTraj = [];
            for i = 1:size(configs, 1)-1
                qSegment = trapveltraj([configs(i,:); configs(i+1,:)]', numSteps)';
                qTraj = [qTraj; qSegment];
            end
            
            % Animate
            attached = false;
            cubeTrailX = []; cubeTrailY = []; cubeTrailZ = [];
            
            % Calculate actual segment indices from trajectory
            totalSegments = size(cartWaypoints, 3) - 1;
            segmentSize = round(size(qTraj, 1) / totalSegments);
            
            % Adjust segment indices based on number of waypoints
            if obj == 1
                % First object: 6 waypoints = 5 segments
                % Segments: 0->1, 1->2(pick), 2->3, 3->4, 4->5(place)
                pickIdx = 1 * segmentSize;      % After reaching pick position
                placeIdx = 4 * segmentSize;     % After reaching place position
            else
                % Subsequent objects: 7 waypoints = 6 segments
                % Segments: 0->1, 1->2, 2->3(pick), 3->4, 4->5, 5->6(place)
                pickIdx = 2 * segmentSize;      % After reaching pick position
                placeIdx = 5 * segmentSize;     % After reaching place position
            end
            
            pickDone = false;
            placeDone = false;
            
            frameCount = 0;
            
            for step = 1:size(qTraj, 1)
                if shouldStop
                    break;
                end
                
                config = qTraj(step, :);
                show(robot, config, "PreservePlot", false, "Frames", "off", "Parent", simAx);
                
                eePose = getTransform(robot, config, endEffector);
                eePos = tform2trvec(eePose);
                
                % Update position display every few frames
                frameCount = frameCount + 1;
                if mod(frameCount, 5) == 0 && ishandle(positionText)
                    positionText.String = sprintf('End-Effector Position:\nX: %.3f m\nY: %.3f m\nZ: %.3f m', ...
                        eePos(1), eePos(2), eePos(3));
                end
                
                % Attach/detach logic - use flags to trigger only once
                if step >= pickIdx && ~pickDone && ~attached
                    attached = true;
                    pickDone = true;
                    statusLabel.String = sprintf('Object %d/%d picked', obj, numObjects);
                    drawnow;
                end
                if step >= placeIdx && ~placeDone && attached
                    attached = false;
                    placeDone = true;
                    statusLabel.String = sprintf('Object %d/%d placed', obj, numObjects);
                    drawnow;
                end
                
                % Update cube position
                if attached
                    vNew = v + (eePos - pickPos);
                    set(cubePatch, 'Vertices', vNew);
                    cubeTrailX(end+1) = mean(vNew(:,1));
                    cubeTrailY(end+1) = mean(vNew(:,2));
                    cubeTrailZ(end+1) = mean(vNew(:,3));
                elseif step >= placeIdx
                    vNew = v + (placePos - pickPos);
                    set(cubePatch, 'Vertices', vNew);
                end
                
                % Plot trail only periodically
                if ~isempty(cubeTrailX) && mod(step, 3) == 0
                    plot3(simAx, cubeTrailX, cubeTrailY, cubeTrailZ, '-', ...
                        'Color', pathColors(obj,:), 'LineWidth', 2);
                end
                
                % Real-time speed control
                currentSpeedVal = get(speedSlider, 'Value');
                currentAnimSpeed = 0.02 / (currentSpeedVal^1.5);
                
                drawnow limitrate;
                if currentAnimSpeed > 0.001
                    pause(currentAnimSpeed);
                end
            end
            
            if shouldStop
                break;
            end
        end
        
        % Completion
        if ~shouldStop
            statusLabel.String = 'All tasks completed successfully!';
            statusLabel.ForegroundColor = [0.3 1 0.5];
        else
            statusLabel.String = 'Simulation stopped';
            statusLabel.ForegroundColor = [1 0.7 0.3];
        end
        
        isSimulationRunning = false;
        startBtn.Enable = 'on';
    end
    
    function [vertices, faces] = createCube(sz)
        vertices = sz/2 * [...
            -1 -1 -1; 1 -1 -1; 1  1 -1; -1  1 -1;
            -1 -1  1; 1 -1  1; 1  1  1; -1  1  1];
        faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    end
end
