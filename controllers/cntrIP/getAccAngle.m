function angle = getAccAngle(acc_x_y_z)
    balance_vector = [acc_x_y_z(1), -acc_x_y_z(3)];
    
    if balance_vector(1) > 0
        angle = +(balance_vector(1)/norm(balance_vector));
    elseif balance_vector(1) < 0
        angle = +(balance_vector(1)/norm(balance_vector));
    else
        angle = 0;
    end
end