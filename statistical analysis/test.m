% In this script I collect the data used to perform a statistical analysis
% between two different implementation of an exploration algorithm
%
% My implementation (https://github.com/AuroraD-Hub/RT1_I-assignment.git)
%  requires less avarage time to finish a lap of the circuit with respect
%  to this other implementation: https://github.com/CarmineD8/python_simulator/tree/rt2
%    H0 : μ = μ_ref
%    Ha : μ < μ_ref
% Other test parameters:
%  - significance level (alpha): 5%
%  - test distribution: t
%  - test type: one-tailed t-Student

data = [4.04 4.02 4.09 3.59 4.00 3.52 3.53 3.57 4.01 4.00 ...
        3.59 3.57 4.00 4.01 3.55 3.59 4.02 3.56 3.59 3.57 ...
        4.02 4.01 4.05 3.55 4.09 4.00 4.02 3.55 4.05 3.58]; % time needed for every execution
data_ref = [3.00 2.57 3.01 2.59 2.55 2.55 3.02 2.58 3.01 3.00 ...
            3.00 3.05 3.06 3.01 3.02 2.58 3.01 3.02 2.59 2.57 ...
            3.00 3.07 3.02 3.02 2.56 3.00 3.00 3.05 3.03 3.01]; % reference time for every execution
N = length(data); % numbers of executions
N_ref = length(data_ref); % reference number of executions
%% Define mean and std values of samples:
m = mean(data);
m_ref = mean(data_ref);

s = std(data);
s_ref = std(data_ref);
%% Define estimated std of the difference of means:
s_pooled_2 = ((N-1)*s.^2+(N_ref-1)*s_ref.^2)/(N+N_ref-2);
sigma = sqrt(s_pooled_2*(1/N+1/N_ref));
%% Define t value and perform the test:
t = -(m-m_ref)/sigma;
t_table = -1.671; % table value with DoF=60 and 1-alpha=95%

if t<t_table % H0 is rejected
    H0 = 1;
    Ha = 0; 
else %H0 is accepted
    H0 = 0;
    Ha = 1;
end
%% Apply directly the native ttest2 to see whether the result is right:
[h, p] = ttest2(data,data_ref);
%% Paired t-test:
d = zeros(1,N);
for i=1:N
   d(:,i) = data_ref(:,i)-data(:,i); 
end

d_mean = mean(d);
d_std = std(d);
SE = d_std/sqrt(N+N_ref);

t_paired = d_mean/SE;
if t_paired<t_table % H0 is rejected
    H0_paired = 1;
    Ha_paired = 0; 
else %H0 is accepted
    H0_paired = 0;
    Ha_paired = 1;
end