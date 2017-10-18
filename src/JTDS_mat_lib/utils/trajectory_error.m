function trajectory_RMSE = trajectory_error(motion_generator, Q_true, Qd_true, Xt)
    Qd = zeros(size(Qd_true));
    for i = 1:size(Q_true, 2)
        Qd(:, i) = motion_generator.get_next_motion(Q_true(:, i), Xt(:, i));
    end
    trajectory_RMSE = sqrt(mean((Qd - Qd_true).^2, 2));
end

