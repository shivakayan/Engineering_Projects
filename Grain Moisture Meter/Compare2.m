% Load data from Excel file
data = xlsread('Training.xlsx');  % Load all data from the file

% Extract values from respective columns
voltage = data(:, 1);   % Assuming Voltage is in the first column
moisture = data(:, 2);  % Assuming Moisture is in the second column

% Plotting only the input data
figure;
scatter(voltage, moisture, 'filled');
xlabel('Voltage');
ylabel('Moisture');
title('Input Data (Voltage vs. Moisture)');
grid on;

%% Fit a curve using different methods and plot curves

figure;

% 1. Using fitlm for Linear Regression
subplot(2, 3, 1);
mdl_fitlm = fitlm(voltage, moisture, 'linear');
plot(mdl_fitlm);
title('fitlm - Linear Regression');
disp('Fitted curve equation (fitlm):');
disp(mdl_fitlm.Coefficients);

% 2. Using fitnlm for Nonlinear Regression
subplot(2, 3, 2);
model = @(b, x) b(1) * exp(b(2) * x); % Example: Exponential model
beta0 = [1 1]; % Initial guess for parameters
mdl_fitnlm = fitnlm(voltage, moisture, model, beta0);
plot(voltage, mdl_fitnlm.Fitted, 'r', 'LineWidth', 2);
title('fitnlm - Nonlinear Regression');
disp('Fitted curve equation (fitnlm):');
disp(mdl_fitnlm.Coefficients.Estimate');

% 3. Using glmfit for Generalized Linear Models (GLMs)
subplot(2, 3, 3);
X = [ones(size(voltage)) voltage];
b_ridge = ridge(moisture, X, 3); % lambda is the regularization parameter
fittedValues_ridge = X * b_ridge;
plot(voltage, fittedValues_ridge, 'c', 'LineWidth', 2);
title('glmfit - Ridge Regression');
disp('Fitted curve equation (glmfit):');
disp(['Intercept: ' num2str(b_ridge(1)) ', Slope: ' num2str(b_ridge(2))]);

% 4. Using polyfit for Polynomial Regression
subplot(2, 3, 4);
degree = 2; % Change the degree as needed
p = polyfit(voltage, moisture, degree);
fittedValues_polyfit = polyval(p, voltage);
plot(voltage, fittedValues_polyfit, 'm', 'LineWidth', 2);
title('polyfit - Polynomial Regression');
disp('Fitted curve equation (polyfit):');
disp(['Coefficients: ' num2str(p)]);

% 5. Using regress for Multivariate Linear Regression
subplot(2, 3, 5);
X_regress = [ones(size(voltage)) voltage]; % Assuming simple linear regression
b_regress = regress(moisture, X_regress);
fittedValues_regress = X_regress * b_regress;
plot(voltage, fittedValues_regress, 'b', 'LineWidth', 2);
title('regress - Multivariate Linear Regression');
disp('Fitted curve equation (regress):');
disp(['Intercept: ' num2str(b_regress(1)) ', Slope: ' num2str(b_regress(2))]);

% Create a new figure displaying only the input data
figure;
scatter(voltage, moisture, 'filled');
xlabel('Voltage');
ylabel('Moisture');
title('Input Data (Voltage vs. Moisture)');
grid on;
