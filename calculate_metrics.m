function metrics = calculate_metrics(t, y, y_ref, step_time)
    % calculate_metrics Calculates Settling Time, Overshoot, and Steady State Error
    %   metrics = calculate_metrics(t, y, y_ref, step_time)
    %
    %   Inputs:
    %       t: Time vector
    %       y: Output vector (response)
    %       y_ref: Reference value (scalar)
    %       step_time: Time when the step/disturbance occurred (usually 0 for step response)
    %
    %   Outputs:
    %       metrics: Struct containing calculated metrics

    % 1. Steady State Error
    % Assume the last 10% of data represents steady state
    num_samples = length(y);
    ss_indices = floor(0.9 * num_samples):num_samples;
    y_ss = mean(y(ss_indices));
    metrics.SteadyStateError = abs(y_ref - y_ss);
    metrics.FinalValue = y_ss;

    % 2. Overshoot
    % Calculate percentage overshoot relative to the step size (or reference if step starts at 0)
    % Note: For stabilization (ref=0) from an initial condition, "overshoot" is usually 
    % defined relative to the initial deviation or just the peak deviation in the opposite direction.
    % Here we assume standard step response definition if y_ref != 0.
    % If y_ref == 0 (stabilization), we look for the peak deviation from 0.
    
    if y_ref ~= 0
        [y_max, ~] = max(y);
        metrics.Overshoot = (y_max - y_ref) / y_ref * 100;
    else
        % For stabilization to 0, overshoot is often undefined in the classic sense,
        % or it refers to the maximum excursion on the OTHER side of zero.
        % Let's define it as the max absolute value after the first zero crossing?
        % Or simply the maximum absolute deviation if we want to capture "peak".
        % Let's stick to Peak Amplitude for stabilization.
        metrics.PeakAmplitude = max(abs(y));
        metrics.Overshoot = NaN; % Not a standard step response
    end

    % 3. Settling Time (2% criterion)
    % Time it takes to stay within 2% of the final value
    threshold = 0.02 * abs(y_ref);
    if y_ref == 0
        % If ref is 0, use 2% of the maximum initial deviation or peak
        threshold = 0.02 * max(abs(y)); 
    end
    
    % Find the last time the signal was OUTSIDE the threshold
    is_outside = abs(y - y_ref) > threshold;
    last_outside_idx = find(is_outside, 1, 'last');
    
    if isempty(last_outside_idx)
        metrics.SettlingTime = 0; % Already settled
    else
        metrics.SettlingTime = t(last_outside_idx) - step_time;
    end
    
    % Ensure settling time is non-negative
    metrics.SettlingTime = max(0, metrics.SettlingTime);

end
