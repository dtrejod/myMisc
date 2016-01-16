% Personal Project
% Image Acquisition Testing
% By: Devin Trejo
% Source1: http://www.mathworks.com/help/vision/examples/face-detection-and-tracking-using-camshift.html
% Source2: http://www.mathworks.com/help/supportpkg/usbwebcams/ug/webcam.html


%%
clear; close all; clc;

% Create a cascade detector object.
faceDetector = vision.CascadeObjectDetector();

% Create a histogram based tracker object.
tracker = vision.HistogramBasedTracker;

% Create connection to webcam
mycam = webcam; 
while 1
    % Read a video frame and run the detector.
    videoFrame = snapshot(mycam);
    bbox = step(faceDetector, videoFrame);
    
    % We didn't find a face
    %
    if (isempty(bbox) == 1)
        figure(1), imshow(videoFrame), title('No-Face Detected face');
        
    % We found a face in the frame!
    %
    else 
        % Draw the returned bounding box around the detected face.
        videoOut = insertObjectAnnotation(videoFrame,'rectangle',...
            bbox,'Face');
        
        figure(1), imshow(videoOut), title('Potential-Detected face');
        
        % Get the skin tone information by extracting the Hue from the 
        % video frame converted to the HSV color space.
        [hueChannel,~,~] = rgb2hsv(videoFrame);

        % Display the Hue Channel data and draw the bounding box around the 
        % face.
        %figure(3), imshow(hueChannel), title('Hue channel data');
        %rectangle('Position',bbox(1,:),'LineWidth',2,'EdgeColor',[1 1 0])
        
        % Detect the nose within the face region. The nose provides a more
        % accurate measure of the skin tone because it does not contain 
        % any background pixels.
        noseDetector = vision.CascadeObjectDetector('Nose', ...
            'UseROI', true);
        noseBBox = step(noseDetector, videoFrame, bbox(1,:));

        % What we detected was a false positive. We continue looking for a
        % face where we can detect a nose
        %
        if (isempty(noseBBox) == 1) 
            continue
        end
        
        % Initialize the tracker histogram using the Hue channel pixels 
        % from the
        % nose.
        initializeObject(tracker, hueChannel, noseBBox(1,:));
        
        % Start timer
        tic;
        
        % Track the face over successive video frames
        while 1
            % Extract the next video frame from webcam
            videoFrame = snapshot(mycam);

            % RGB -> HSV
            [hueChannel1,~,~] = rgb2hsv(videoFrame);
            
            % Track using the Hue channel data
            bbox = step(tracker, hueChannel);
            points = detectMinEigenFeatures(rgb2gray(videoFrame), ... 
                'ROI', bbox);
            
            % Every second look for a nose
            %
            if (toc <= 5)
                noseBBox = step(noseDetector, videoFrame, bbox(1,:));
                tic;
            end
            
            % If any point in box is at the edge of the screen 
            % recalculate face
            %
            if (any(bbox < 10) || points.Count < 10 || ...
                    isempty(noseBBox) == 1)
                release(tracker);
                break
            end
            
            % Insert a bounding box around the object being tracked
            videoOut = insertObjectAnnotation(videoFrame,'rectangle',...
                bbox,'Face');
            figure(1), imshow(videoOut), title('Tracking Face');
            hold on; plot(points); hold off; 
        end
    end
end

%% Offical Method
% Source: http://www.mathworks.com/help/vision/examples/face-detection-and-tracking-using-live-video-acquisition.html
clear; clc; close all;

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
numPts = 0;
frameCount = 0;

while runLoop %&& frameCount < 400

    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);

        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Save a copy of the points.
            oldPoints = xyPoints;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);

        if numPts >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
        end

    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);


%% Combine two approaches
clear; clc; close all;

%%
clear; close all; clc;

% Create a cascade detector object.
faceDetector = vision.CascadeObjectDetector();

% Create a histogram based tracker object.
tracker = vision.HistogramBasedTracker;

% Create connection to webcam
mycam = webcam; 
videoFrame = snapshot(mycam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', ...
    [100 100 [frameSize(2), frameSize(1)]+30]);

while 1
    % Read a video frame and run the detector.
    videoFrame = snapshot(mycam);
    bbox = step(faceDetector, videoFrame);
    
    % We didn't find a face
    %
    if (isempty(bbox) == 1)
        % Display the annotated video frame using the video player object.
        step(videoPlayer, videoFrame);
        
    % We found a face in the frame!
    %
    else 
        % Draw the returned bounding box around the detected face.
        videoOut = insertObjectAnnotation(videoFrame,'rectangle',...
            bbox,'Pot-Face');
        
        step(videoPlayer, videoOut);
        
        % Get the skin tone information by extracting the Hue from the 
        % video frame converted to the HSV color space.
        [hueChannel,~,~] = rgb2hsv(videoFrame);

        % Display the Hue Channel data and draw the bounding box around the 
        % face.
        %figure(3), imshow(hueChannel), title('Hue channel data');
        %rectangle('Position',bbox(1,:),'LineWidth',2,'EdgeColor',[1 1 0])
        
        % Detect the nose within the face region. The nose provides a more
        % accurate measure of the skin tone because it does not contain 
        % any background pixels.
        noseDetector = vision.CascadeObjectDetector('Nose', ...
            'UseROI', true);
        noseBBox = step(noseDetector, videoFrame, bbox(1,:));

        % What we detected was a false positive. We continue looking for a
        % face where we can detect a nose
        %
        if (isempty(noseBBox) == 1) 
            continue
        end
        
        % Initialize the tracker histogram using the Hue channel pixels 
        % from the
        % nose.
        initializeObject(tracker, hueChannel, noseBBox(1,:));
        
        % Start timer
        tic;
        
        % Track the face over successive video frames
        while 1
            % Extract the next video frame from webcam
            videoFrame = snapshot(mycam);

            % RGB -> HSV
            [hueChannel1,~,~] = rgb2hsv(videoFrame);
            
            % Track using the Hue channel data
            bbox = step(tracker, hueChannel);
            points = detectMinEigenFeatures(rgb2gray(videoFrame), ... 
                'ROI', bbox);
            
            % Every second look for a nose
            %
            if (toc <= 5)
                noseBBox = step(noseDetector, videoFrame, bbox(1,:));
                tic;
            end
            
            % If any point in box is at the edge of the screen 
            % recalculate face
            %
            if (any(bbox < 10) || points.Count < 10 || ...
                    isempty(noseBBox) == 1)
                release(tracker);
                break
            end
            
            % Insert a bounding box around the object being tracked
            videoOut = insertObjectAnnotation(videoFrame,'rectangle',...
                bbox,'Face');
            
            %figure(1), imshow(videoOut), title('Tracking Face');
            %hold on; plot(points); hold off; 
            % Display the annotated video frame using the video player object.
            step(videoPlayer, videoOut);
        end
    end
end