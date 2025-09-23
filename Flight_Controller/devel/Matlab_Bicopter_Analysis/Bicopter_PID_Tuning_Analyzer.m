% ========================================================================
% QUICK PID HEALTH CHECK
% Fast analysis - just tells you what's wrong and how to fix it
% ========================================================================

clear; clc;

fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║   QUICK PID HEALTH CHECK           ║\n');
fprintf('╔════════════════════════════════════╗\n\n');

%% Load data
[file, path] = uigetfile('*.csv', 'Select Flight CSV');
if file == 0, error('No file selected'); end

data = readtable(fullfile(path, file));
time = (data.Time_ms - data.Time_ms(1)) / 1000;

% Get only armed data
armed = data(data.Armed == 1, :);
if isempty(armed), error('No flight data found!'); end

fprintf('Flight: %.1f seconds of armed data\n\n', max(armed.Time_ms - min(armed.Time_ms))/1000);

%% Quick diagnostics
fprintf('═══ DIAGNOSTICS ═══\n\n');

problems = {};
fixes = {};

% Check 1: Roll oscillations
roll_std = std(armed.RateRoll);
fprintf('Roll Rate StdDev: %.1f deg/s ... ', roll_std);
if roll_std > 60
    fprintf('❌ BAD\n');
    problems{end+1} = 'Roll oscillations';
    fixes{end+1} = 'PRateRoll = PRateRoll * 0.7  % Reduce by 30%%';
elseif roll_std > 40
    fprintf('⚠️  MARGINAL\n');
    problems{end+1} = 'Roll slightly oscillatory';
    fixes{end+1} = 'PRateRoll = PRateRoll * 0.85  % Reduce by 15%%';
else
    fprintf('✓ GOOD\n');
end

% Check 2: Pitch oscillations
pitch_std = std(armed.RatePitch);
fprintf('Pitch Rate StdDev: %.1f deg/s ... ', pitch_std);
if pitch_std > 60
    fprintf('❌ BAD\n');
    problems{end+1} = 'Pitch oscillations';
    fixes{end+1} = 'PRatePitch = PRatePitch * 0.7  % Reduce by 30%%';
elseif pitch_std > 40
    fprintf('⚠️  MARGINAL\n');
    problems{end+1} = 'Pitch slightly oscillatory';
    fixes{end+1} = 'PRatePitch = PRatePitch * 0.85  % Reduce by 15%%';
else
    fprintf('✓ GOOD\n');
end

% Check 3: Roll tracking
roll_error = sqrt(mean((armed.DesRoll - armed.ActRoll).^2));
fprintf('Roll Tracking RMSE: %.1f deg ... ', roll_error);
if roll_error > 7
    fprintf('❌ BAD\n');
    problems{end+1} = 'Poor roll tracking';
    fixes{end+1} = 'PAngleRoll = PAngleRoll * 1.3  % Increase by 30%%';
elseif roll_error > 4
    fprintf('⚠️  MARGINAL\n');
    problems{end+1} = 'Roll tracking could be better';
    fixes{end+1} = 'PAngleRoll = PAngleRoll * 1.15  % Increase by 15%%';
else
    fprintf('✓ GOOD\n');
end

% Check 4: Pitch tracking
pitch_error = sqrt(mean((armed.DesPitch - armed.ActPitch).^2));
fprintf('Pitch Tracking RMSE: %.1f deg ... ', pitch_error);
if pitch_error > 7
    fprintf('❌ BAD\n');
    problems{end+1} = 'Poor pitch tracking';
    fixes{end+1} = 'PAnglePitch = PAnglePitch * 1.3  % Increase by 30%%';
elseif pitch_error > 4
    fprintf('⚠️  MARGINAL\n');
    problems{end+1} = 'Pitch tracking could be better';
    fixes{end+1} = 'PAnglePitch = PAnglePitch * 1.15  % Increase by 15%%';
else
    fprintf('✓ GOOD\n');
end

% Check 5: Servo smoothness
servo_jitter = std(diff(armed.ServoL)) + std(diff(armed.ServoR));
fprintf('Servo Smoothness: %.1f ... ', servo_jitter);
if servo_jitter > 30
    fprintf('❌ JITTERY\n');
    problems{end+1} = 'Servo jitter';
    fixes{end+1} = 'SERVO_ALPHA = 0.4  % Increase filtering (currently 0.6)';
elseif servo_jitter > 15
    fprintf('⚠️  MARGINAL\n');
else
    fprintf('✓ GOOD\n');
end

%% Summary
fprintf('\n═══ SUMMARY ═══\n\n');

if isempty(problems)
    fprintf('🎉 ALL SYSTEMS GOOD!\n');
    fprintf('Your PID tuning is solid. Fine-tune if desired.\n');
else
    fprintf('Found %d issue(s):\n\n', length(problems));
    
    for i = 1:length(problems)
        fprintf('%d. %s\n', i, problems{i});
        fprintf('   FIX: %s\n\n', fixes{i});
    end
    
    fprintf('═══ QUICK FIX CODE ═══\n');
    fprintf('Copy these lines to your Arduino code:\n\n');
    for i = 1:length(fixes)
        fprintf('// %s\n%s;\n', problems{i}, fixes{i});
    end
end

%% Quick plot
figure('Name', 'Quick Overview', 'Position', [100 100 1200 400]);

subplot(1,3,1);
plot(armed.Time_ms/1000, armed.DesRoll, 'r--'); hold on;
plot(armed.Time_ms/1000, armed.ActRoll, 'b-');
title('Roll'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Des', 'Act');

subplot(1,3,2);
plot(armed.Time_ms/1000, armed.DesPitch, 'r--'); hold on;
plot(armed.Time_ms/1000, armed.ActPitch, 'b-');
title('Pitch'); xlabel('Time (s)'); ylabel('Angle (deg)');

subplot(1,3,3);
plot(armed.Time_ms/1000, armed.RateRoll, 'b-'); hold on;
plot(armed.Time_ms/1000, armed.RatePitch, 'r-');
title('Rates'); xlabel('Time (s)'); ylabel('deg/s');
legend('Roll', 'Pitch');

fprintf('\n✓ Done!\n');
