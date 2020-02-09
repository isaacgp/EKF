function [X_bar, P_bar] = motion_model(X_hat_tm1,P_tm1, u)
    
    global r_r r_l e Q_wheel
    
    %--------------------- Estimation of X_bar --------------------------
    X_bar = X_hat_tm1 + [ u(1) * cos(X_hat_tm1(3)) ;
                          u(1) * sin(X_hat_tm1(3)) ;
                          u(2)];
                      
    %---------------------- Estimation of P_bar --------------------------
    %Jacobians of Motion Model
    Ak = [1, 0, -u(1) * sin(X_hat_tm1(3));
          0, 1, u(1) * cos(X_hat_tm1(3));
          0, 0, 1];
    
    Bk = [cos(X_hat_tm1(3)), 0;
          sin(X_hat_tm1(3)), 0;
          0, 1];
      
    %Covariance of control signal (odometry)  
    K = [r_r/2, r_l/2;
         r_r/e, -r_l/e];
    P_u = K * Q_wheel * K';

    % Covariance of estimated robot pose
    P_bar = Ak * P_tm1 * Ak' + Bk * P_u * Bk';

end

