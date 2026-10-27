function [w, v, name] = gait_reference_signal(gyro, axisChoice, invert, lpHz, fs)
%GAIT_REFERENCE_SIGNAL  Reference angular velocity for heel-strike detection.
%   [w, v, name] = gait_reference_signal(gyro, axisChoice, invert, lpHz, fs)
%     gyro        [n x 3] shoe angular rate [rad/s], IMU frame
%     axisChoice  1/2/3 = gyro x/y/z, 4 = principal axis of the gyro (PCA,
%                 sign fixed so that its largest component is positive)
%     invert      true flips the sign
%     lpHz        zero-phase low-pass cut-off [Hz] (0 = no filter)
%     fs          sample rate [Hz]
%   w = gyro * v, low-passed.  Convention expected by gait_hs_detect:
%   mid-swing peak positive, toe-off and heel-strike troughs negative.

names = {'gyro x', 'gyro y', 'gyro z', 'principal axis'};
gyro = double(gyro);
gyro(~isfinite(gyro)) = 0;
if axisChoice <= 3
    v = zeros(3, 1);
    v(axisChoice) = 1;
else
    [V, Dg] = eig(cov(gyro));
    [~, im] = max(diag(Dg));
    v = V(:, im);
    [~, j] = max(abs(v));
    v = v * sign(v(j));
end
if invert, v = -v; end
w = gyro * v;
if lpHz > 0
    w = vn_lp_zerophase(w, lpHz, fs);
end
name = names{axisChoice};
if invert, name = ['-' name]; end
end
