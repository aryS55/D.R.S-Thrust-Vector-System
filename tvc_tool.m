%% TVC Rocket - All-in-One Tool (Live Logger + PID Analyzer + 3D Orientation)
% One file, two modes - set MODE below and run.
%
% ---------------------------------------------------------------------
% WHAT DATA IS AVAILABLE (every column tvc_rocket.ino outputs, both
% over Serial live and to the SD card as LOGnnnn.CSV - same columns
% either way, so everything below applies to both MODE = "live" and
% MODE = "analyze")
% ---------------------------------------------------------------------
%   time_ms        - timestamp, ms
%   angle_x        - estimated tilt about X (toward FIN2), deg.
%                    Currently gyro-only (no accel correction, see
%                    tvc_rocket.ino) so it WILL drift slowly even when
%                    the rig sits still - that's expected right now,
%                    not a bug or a PID problem.
%   angle_y        - estimated tilt about Y (toward FIN1), deg. Same
%                    gyro-only caveat as angle_x.
%   rate_z         - spin rate about Z (vertical/roll), deg/s
%   gx, gy, gz     - bias-corrected gyro rates, deg/s
%   ax, ay, az     - raw accelerometer, g. Logged for debugging but not
%                    currently used in the angle estimate.
%   angle_x_sp     - setpoint for angle_x, deg (0 for hover-straight-up)
%   angle_y_sp     - setpoint for angle_y, deg
%   err_x, err_y   - tracking error (setpoint - estimate), deg
%   pid_x, pid_y   - total PID controller output, deg-equivalent
%   pid_z          - roll/spin PID output
%   servo1..servo4 - commanded fin deflection, deg (FIN1..FIN4)
%
% ---------------------------------------------------------------------
% WHAT YOU CAN DO WITH THIS FILE
% ---------------------------------------------------------------------
% MODE = "live":
%   Connects to the board over Serial, shows a live-updating view with
%   attitude, PID output, and servo command plots, PLUS a rotating
%   labeled 3D block (TOP/BOTTOM/FIN1-4 faces) that mirrors angle_x/
%   angle_y in real time - tilt the rig toward a fin and watch the
%   model rotate toward that same face. Fast way to catch a mismounted
%   IMU or a sign error before it becomes a "why are my fins backwards"
%   argument. Saves everything to a timestamped CSV when you close it.
%   (The SD card gets an independent copy too, so nothing is lost if
%   this live session gets interrupted.)
%
% MODE = "analyze":
%   Loads a saved CSV (your own live-save, or a LOGnnnn.CSV pulled off
%   the SD card) and gives you INAV-style configurable overlay plots
%   for PID tuning - edit PLOT_GROUPS to decide what overlays with
%   what (e.g. angle_x vs angle_x_sp on the same axes), and
%   GROUPS_TO_PLOT to choose which of those you actually want to see
%   this run. Also prints basic step-response metrics (peak, settling
%   time, oscillation period) and gives the SAME 3D orientation model
%   as a scrubbable slider-driven replay so you can step through a
%   saved run and visually re-check the tilt-vs-fin-direction mapping
%   after the fact.
%
% Usage:
%   1. Set MODE below to "live" or "analyze"
%   2. Fill in the matching config block (PORT for live, FILE for
%      analyze)
%   3. Run the script

clear; clc; close all;

%% ==================== MODE SELECT ====================
MODE = "analyze";   % <-- "live" or "analyze"

%% ==================== LIVE-MODE CONFIG ====================
PORT = "COM5";          % <-- your board's serial port (Windows: 'COM5', Mac/Linux: '/dev/tty.usbserial-XXXX')
BAUD = 115200;
LOG_DURATION_S = 60;    % how long to log (seconds); set inf to run until you close the figure

%% ==================== ANALYZE-MODE CONFIG ====================
FILE = "LOG0001.CSV";   % <-- CSV to analyze (SD card log or a live-mode save)
REPLAY_ORIENTATION = true; % show the scrubbable 3D orientation replay?

% Edit these to choose what overlays with what (INAV-style). Each
% group becomes one figure; every channel inside a group is drawn on
% the SAME axes (that's the "overlay"). Put something alone in its own
% group to view it separately instead. GROUPS_TO_PLOT below picks
% which of these actually get drawn.
G = struct('title', {}, 'ylabel', {}, 'channels', {});

G(end+1) = struct('title', 'Attitude vs Setpoint', 'ylabel', 'deg', ...
    'channels', {{ ...
        {'angle_x',    'Angle X',    'r-'}, ...
        {'angle_x_sp', 'Angle X SP', 'r--'}, ...
        {'angle_y',    'Angle Y',    'b-'}, ...
        {'angle_y_sp', 'Angle Y SP', 'b--'} }});

G(end+1) = struct('title', 'Tracking Error', 'ylabel', 'deg', ...
    'channels', {{ ...
        {'err_x', 'Angle X error', 'r-'}, ...
        {'err_y', 'Angle Y error', 'b-'} }});

G(end+1) = struct('title', 'PID Output (all axes)', 'ylabel', 'command (deg)', ...
    'channels', {{ ...
        {'pid_x', 'PID X (FIN2/4)',    'r-'}, ...
        {'pid_y', 'PID Y (FIN1/3)',    'b-'}, ...
        {'pid_z', 'PID Z (roll/all4)', 'g-'} }});

G(end+1) = struct('title', 'Servo / Fin Commands', 'ylabel', 'fin deflection (deg)', ...
    'channels', {{ ...
        {'servo1', 'Servo1 (FIN1,+Y)', '-'}, ...
        {'servo2', 'Servo2 (FIN2,+X)', '-'}, ...
        {'servo3', 'Servo3 (FIN3,-Y)', '-'}, ...
        {'servo4', 'Servo4 (FIN4,-X)', '-'} }});

G(end+1) = struct('title', 'Raw Gyro', 'ylabel', 'deg/s', ...
    'channels', {{ {'gx','gx','-'}, {'gy','gy','-'}, {'gz','gz','-'} }});

G(end+1) = struct('title', 'Raw Accel', 'ylabel', 'g', ...
    'channels', {{ {'ax','ax','-'}, {'ay','ay','-'}, {'az','az','-'} }});

GROUPS_TO_PLOT = { ...
    'Attitude vs Setpoint', ...
    'Tracking Error', ...
    'PID Output (all axes)', ...
    'Servo / Fin Commands', ...
    'Raw Gyro' ...
    };

%% ==================== COMMON ====================
COLUMN_NAMES = ["time_ms","angle_x","angle_y","rate_z","gx","gy","gz", ...
    "ax","ay","az","angle_x_sp","angle_y_sp","err_x","err_y", ...
    "pid_x","pid_y","pid_z","servo1","servo2","servo3","servo4"];

%% ==================== DISPATCH ====================
if MODE == "live"
    runLiveMode(PORT, BAUD, LOG_DURATION_S, COLUMN_NAMES);
elseif MODE == "analyze"
    runAnalyzeMode(FILE, G, GROUPS_TO_PLOT, REPLAY_ORIENTATION);
else
    error("MODE must be 'live' or 'analyze', got '%s'", MODE);
end

%% ==================== LOCAL FUNCTIONS ====================

function runLiveMode(PORT, BAUD, LOG_DURATION_S, COLUMN_NAMES)
    OUTFILE = sprintf("tvc_log_%s.csv", datestr(now, 'yyyymmdd_HHMMSS'));

    s = serialport(PORT, BAUD);
    configureTerminator(s, "LF");
    flush(s);

    fprintf("Connecting... waiting for CSV header from board\n");
    header_found = false;
    tic;
    while ~header_found && toc < 15
        line = readline(s);
        if contains(line, "time_ms")
            header_found = true;
        else
            fprintf("Board: %s\n", line);
        end
    end
    if ~header_found
        error("Never received CSV header from board. Check wiring/port/baud.");
    end
    fprintf("Header received. Logging started.\n");

    maxPoints = 5000;
    data = nan(maxPoints, numel(COLUMN_NAMES));
    idx = 0;

    fig = figure('Name', 'TVC Live Tuning View', 'NumberTitle', 'off', ...
        'Position', [80 80 1050 700]);

    subplot(3,2,1);
    h_x = animatedline('Color', 'r', 'DisplayName', 'Angle X (FIN2/4)');
    h_y = animatedline('Color', 'b', 'DisplayName', 'Angle Y (FIN1/3)');
    legend show; ylabel('deg'); title('Attitude'); grid on;

    subplot(3,2,3);
    h_pidx = animatedline('Color', 'r', 'DisplayName', 'PID X');
    h_pidy = animatedline('Color', 'b', 'DisplayName', 'PID Y');
    legend show; ylabel('cmd (deg)'); title('PID Output'); grid on;

    subplot(3,2,5);
    h_s1 = animatedline('Color', [0.8 0 0], 'DisplayName', 'Servo1 (FIN1,+Y)');
    h_s2 = animatedline('Color', [0 0.6 0], 'DisplayName', 'Servo2 (FIN2,+X)');
    h_s3 = animatedline('Color', [0 0 0.8], 'DisplayName', 'Servo3 (FIN3,-Y)');
    h_s4 = animatedline('Color', [0.6 0 0.6], 'DisplayName', 'Servo4 (FIN4,-X)');
    legend show; ylabel('fin deg'); xlabel('time (s)'); title('Servo Commands'); grid on;

    ax3d = subplot(3,2,[2 4 6]);
    orientationBlock = buildOrientationBlock(ax3d);
    title(ax3d, 'Live Orientation (matches angle\_x / angle\_y)');

    t0 = tic;
    while ishandle(fig) && toc(t0) < LOG_DURATION_S
        if s.NumBytesAvailable > 0
            line = readline(s);
            vals = str2double(split(line, ","));
            if numel(vals) == numel(COLUMN_NAMES) && all(~isnan(vals))
                idx = idx + 1;
                if idx > maxPoints
                    data(1:end-1, :) = data(2:end, :);
                    idx = maxPoints;
                end
                data(idx, :) = vals;

                t_s = vals(1) / 1000;
                angle_x = vals(2);
                angle_y = vals(3);

                addpoints(h_x, t_s, angle_x);
                addpoints(h_y, t_s, angle_y);
                addpoints(h_pidx, t_s, vals(15));
                addpoints(h_pidy, t_s, vals(16));
                addpoints(h_s1, t_s, vals(18));
                addpoints(h_s2, t_s, vals(19));
                addpoints(h_s3, t_s, vals(20));
                addpoints(h_s4, t_s, vals(21));

                updateOrientationBlock(orientationBlock, angle_x, angle_y);

                if mod(idx, 5) == 0
                    drawnow limitrate;
                end
            end
        end
    end

    clear s;
    validRows = ~any(isnan(data), 2);
    T = array2table(data(validRows, :), 'VariableNames', COLUMN_NAMES);
    writetable(T, OUTFILE);
    fprintf("Saved %d rows to %s\n", height(T), OUTFILE);
    fprintf("(The SD card also has an independent copy as LOGnnnn.CSV -\n");
    fprintf("run this same file with MODE=""analyze"" on either copy.)\n");
end

function runAnalyzeMode(FILE, G, GROUPS_TO_PLOT, REPLAY_ORIENTATION)
    T = readtable(FILE);
    cols = T.Properties.VariableNames;
    have = @(name) ismember(name, cols);

    t = T.time_ms / 1000;
    t = t - t(1);

    for gi = 1:numel(GROUPS_TO_PLOT)
        gname = GROUPS_TO_PLOT{gi};
        gidx = find(strcmp({G.title}, gname), 1);
        if isempty(gidx)
            warning("Group '%s' not found in G - skipping.", gname);
            continue;
        end
        plotGroup(T, t, G(gidx), have);
    end

    axisName = "angle_y";
    err = T.err_y;
    resp = T.angle_y;

    [peakVal, peakIdx] = max(abs(resp));
    fprintf("--- %s response metrics ---\n", axisName);
    fprintf("Peak |%s| = %.2f deg at t = %.2f s\n", axisName, peakVal, t(peakIdx));

    tol = 0.02 * max(abs(err));
    settleIdx = find(abs(err) > tol, 1, 'last');
    if ~isempty(settleIdx)
        fprintf("Settling time (2%% band): %.2f s\n", t(settleIdx));
    end

    zc = find(diff(sign(err)) ~= 0);
    if numel(zc) > 2
        periods = diff(t(zc)) * 2;
        fprintf("Approx oscillation period: %.3f s (%.2f Hz)\n", ...
            mean(periods), 1/mean(periods));
    end

    fprintf("\nTuning tips:\n");
    fprintf(" - Overshoot too high -> increase Kd or decrease Kp\n");
    fprintf(" - Slow to respond / mushy -> increase Kp\n");
    fprintf(" - Steady-state offset that never fully corrects -> increase Ki slightly\n");
    fprintf(" - Oscillates and never settles -> decrease Kp and/or increase Kd\n");
    fprintf(" - PID output plot pinned at limits (clipping) -> reduce Kp or raise MAX_FIN_DEG\n");
    fprintf(" - angle_x/angle_y drifting even when the rig sits still -> expected right now,\n");
    fprintf("   it's gyro-only (no accel correction) until the BMI323 accel config is\n");
    fprintf("   sorted out - don't chase this with PID gains, it's not a tuning problem\n");

    if REPLAY_ORIENTATION
        replayOrientation(t, T.angle_x, T.angle_y);
    end
end

function plotGroup(T, t, group, have)
    figure('Name', group.title);
    hold on;
    legendEntries = {};
    for i = 1:numel(group.channels)
        ch = group.channels{i};
        colName = ch{1};
        dispName = ch{2};
        lineSpec = ch{3};
        scaleFactor = 1;
        if numel(ch) >= 4
            scaleFactor = ch{4};
        end
        if ~have(colName)
            fprintf("  (skipping '%s' - not present in this log file)\n", colName);
            continue;
        end
        y = T.(colName) * scaleFactor;
        plot(t, y, lineSpec, 'DisplayName', dispName, 'LineWidth', 1.1);
        legendEntries{end+1} = dispName; %#ok<AGROW>
    end
    if ~isempty(legendEntries)
        legend show;
    end
    xlabel('time (s)');
    ylabel(group.ylabel);
    title(group.title);
    grid on;
    hold off;
end

function replayOrientation(t, angle_x, angle_y)
% Labeled 3D block (+X toward FIN2, +Y toward FIN1, +Z up - matches
% tvc_rocket.ino exactly) with a slider to scrub through the log.
    fig = figure('Name', 'Orientation Replay', 'Position', [150 150 520 560]);
    ax3d = axes('Parent', fig, 'Position', [0.08 0.18 0.86 0.76]);
    block = buildOrientationBlock(ax3d);
    title(ax3d, 'Orientation Replay (drag slider below)');

    n = numel(t);
    slider = uicontrol('Parent', fig, 'Style', 'slider', 'Min', 1, 'Max', n, ...
        'Value', 1, 'SliderStep', [1/max(n-1,1), 10/max(n-1,1)], ...
        'Units', 'normalized', 'Position', [0.08 0.05 0.86 0.05]);

    updateOrientationBlock(block, angle_x(1), angle_y(1));

    addlistener(slider, 'ContinuousValueChange', @(src, ~) ...
        updateOrientationBlock(block, angle_x(round(src.Value)), angle_y(round(src.Value))));
end

function block = buildOrientationBlock(ax3d)
% Builds a labeled 3D block sitting at the origin: +X toward FIN2, +Y
% toward FIN1, +Z up (matches tvc_rocket.ino's axis convention
% exactly). Returns a struct with the patch handle and base
% (unrotated) vertex list so updateOrientationBlock can just re-rotate
% and redraw each frame instead of rebuilding it. Used by both live
% mode (rotates in real time) and analyze mode (rotates via slider).

    axes(ax3d); %#ok<LAXES>
    cla(ax3d);
    hold(ax3d, 'on');

    hw = 1.0; hh = 0.4;
    V = [ -hw -hw -hh;  % 1
           hw -hw -hh;  % 2
           hw  hw -hh;  % 3
          -hw  hw -hh;  % 4
          -hw -hw  hh;  % 5
           hw -hw  hh;  % 6
           hw  hw  hh;  % 7
          -hw  hw  hh]; % 8

    F = [1 2 3 4;   % bottom
         5 6 7 8;   % top
         1 2 6 5;   % -Y face (toward FIN3)
         3 4 8 7;   % +Y face (toward FIN1)
         2 3 7 6;   % +X face (toward FIN2)
         4 1 5 8];  % -X face (toward FIN4)

    faceColors = [
        0.55 0.55 0.55;  % bottom - gray
        0.85 0.85 0.85;  % top - light gray
        0.20 0.55 0.85;  % -Y (FIN3) - blue
        0.85 0.20 0.20;  % +Y (FIN1) - red
        0.20 0.75 0.35;  % +X (FIN2) - green
        0.75 0.35 0.85;  % -X (FIN4) - purple
        ];

    h = patch('Vertices', V, 'Faces', F, 'FaceVertexCData', faceColors, ...
        'FaceColor', 'flat', 'EdgeColor', 'k', 'Parent', ax3d);

    axLim = 1.8;
    axis(ax3d, 'equal');
    xlim(ax3d, [-axLim axLim]); ylim(ax3d, [-axLim axLim]); zlim(ax3d, [-axLim axLim]);
    xlabel(ax3d, 'X (toward FIN2)'); ylabel(ax3d, 'Y (toward FIN1)'); zlabel(ax3d, 'Z (up)');
    view(ax3d, 35, 25);
    grid(ax3d, 'on');
    camlight(ax3d, 'headlight'); lighting(ax3d, 'gouraud');

    labelBaseXYZ = [ ...
        0    0   -hh-0.35;  % BOTTOM
        0    0    hh+0.35;  % TOP
        0   -hw-0.35  0;    % FIN3 (-Y)
        0    hw+0.35  0;    % FIN1 (+Y)
        hw+0.35 0    0;     % FIN2 (+X)
       -hw-0.35 0    0];    % FIN4 (-X)
    labelText = ["BOTTOM","TOP","FIN3 (-Y)","FIN1 (+Y)","FIN2 (+X)","FIN4 (-X)"];
    labelHandles = gobjects(numel(labelText),1);
    for i = 1:numel(labelText)
        labelHandles(i) = text(ax3d, labelBaseXYZ(i,1), labelBaseXYZ(i,2), labelBaseXYZ(i,3), ...
            labelText(i), 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 9);
    end

    block.patchHandle = h;
    block.baseVertices = V;
    block.labelHandles = labelHandles;
    block.labelBaseXYZ = labelBaseXYZ;
    block.angleText = text(ax3d, -axLim*0.95, axLim*0.95, axLim*0.95, '', ...
        'FontSize', 9, 'VerticalAlignment', 'top');
end

function updateOrientationBlock(block, angle_x_deg, angle_y_deg)
% Rotates the block to match angle_x (rotation about X, corrected by
% FIN2/FIN4) and angle_y (rotation about Y, corrected by FIN1/FIN3),
% matching tvc_rocket.ino's convention exactly.
    rx = deg2rad(angle_x_deg);
    ry = deg2rad(angle_y_deg);

    Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
    Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
    R = Ry * Rx;

    Vrot = (R * block.baseVertices')';
    set(block.patchHandle, 'Vertices', Vrot);

    for i = 1:numel(block.labelHandles)
        p = (R * block.labelBaseXYZ(i,:)')';
        set(block.labelHandles(i), 'Position', p);
    end

    set(block.angleText, 'String', ...
        sprintf('angle\\_x = %.1f deg\nangle\\_y = %.1f deg', angle_x_deg, angle_y_deg));
end
