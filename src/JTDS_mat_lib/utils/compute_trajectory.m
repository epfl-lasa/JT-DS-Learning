function Qd = compute_trajectory(motion_generator, Q_true, Xt, orientation_flag)
Qd = zeros(size(Q_true));
if orientation_flag==1
    for i = 1:size(Q_true, 2)
        Qd(:, i) = motion_generator.get_next_motion_orientation(Q_true(:, i), Xt);
    end
else
    for i = 1:size(Q_true, 2)
        Qd(:, i) = motion_generator.get_next_motion(Q_true(:, i), Xt);
    end
end
end

