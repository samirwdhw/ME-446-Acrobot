function plotControlTrajectory(Xout,uout, p)
% This function plots the motor operating region and the trajectory 
% taken by the bot to confirm that it stays in the operating region

% Motor parameters
NH = p.params(24);
tau_stall = p.params(29);
omega_NL = p.params(30);
tau_Imax = p.params(31);

% Plotting the motor operating region

% Writing motor constraints as inline functions
constraint1 = @(N, tau, dq)tau/N <= tau_Imax;
constraint2 = @(N, tau, dq)tau/N >= -tau_Imax;
constraint3 = @(N, tau, dq)tau/N <= -tau_stall*dq*N/omega_NL + tau_stall;
constraint4 = @(N, tau, dq)tau/N >= -tau_stall*dq*N/omega_NL - tau_stall;

% Constraint to output torque at the extreme value of every constraint
constr_tau_val = @(N,dq)N*[tau_Imax;
                           -tau_Imax;...
                           (-tau_stall*dq*N/omega_NL + tau_stall);...
                           (-tau_stall*dq*N/omega_NL - tau_stall)];

% Function to check if all constraints are valid
constraint_valid = @(N,tau,dq)constraint1(N,tau,dq) && ...
                              constraint2(N,tau,dq) && ...
                              constraint3(N,tau,dq) && ...
                              constraint4(N,tau,dq);   

omega = [-100:0.001:100];
counter_H = 1;
points_omegaH = zeros(1,2*length(omega));
points_tauH = zeros(1,2*length(omega));

for i = 1:length(omega)
    
    tau_H = constr_tau_val(NH, omega(i));
    
    for j = 1:4
        % Checking if the points are valid for hip motor
        if constraint_valid(NH, tau_H(j),omega(i))
            points_tauH(counter_H) = tau_H(j);
            points_omegaH(counter_H) = omega(i);
            counter_H = counter_H + 1;
        end
        
    end
    
end

figure
scatter(points_omegaH(1:counter_H-1), points_tauH(1:counter_H-1),1); hold on;
scatter(Xout(:,4), uout);
legend('Operating Region', 'Simulation');

title('Operating Region for Actuator', 'FontSize', 15);
xlabel('\omega_H (rad/s)', 'FontSize', 15);
ylabel('\tau_H (Nm)');
xlim([-60,60]);

end

