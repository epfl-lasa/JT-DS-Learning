function trajectory_RMSE = trajectory_error(motion_generator, Q_true, Qd_true, Xt,orientation_flag)
Qd = zeros(size(Qd_true));
if orientation_flag==1
    for i = 1:size(Q_true, 2)
        Qd(:, i) = motion_generator.get_next_motion_orientation(Q_true(:, i), Xt(:, i));
    end
else
    for i = 1:size(Q_true, 2)
        Qd(:, i) = motion_generator.get_next_motion(Q_true(:, i), Xt(:, i));
    end
end
trajectory_RMSE = sqrt(mean((Qd - Qd_true).^2, 2));
end

