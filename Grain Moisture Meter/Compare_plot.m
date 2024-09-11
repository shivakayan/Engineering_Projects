% Load data from Excel file
data = xlsread('Training.xlsx');  % Load all data from the file

% Extract values from respective columns
voltage = data(:, 1);   % Assuming Voltage is in the first column
moisture = data(:, 2);  % Assuming Moisture is in the second column

% Plotting the scatter plot
figure;
scatter(voltage, moisture, 'filled');
xlabel('Voltage');
ylabel('Moisture');
title('Voltage vs. Moisture');
grid on;

%% Fit a curve using different methods

% 1. Using fitlm for Linear Regression
mdl_fitlm = fitlm(voltage, moisture, 'linear');
subplot(2, 3, 1);
plot(mdl_fitlm);
title('fitlm - Linear Regression');
hold on;
residuals_fitlm = mdl_fitlm.Residuals.Raw;
plot(voltage, residuals_fitlm, 'r.', 'MarkerSize', 8);
hold off;

% 2. Using fitnlm for Nonlinear Regression
model = @(b, x) b(1) * exp(b(2) * x); % Example: Exponential model
beta0 = [1 1]; % Initial guess for parameters
mdl_fitnlm = fitnlm(voltage, moisture, model, beta0);
subplot(2, 3, 2);
plot(voltage, mdl_fitnlm.Fitted, 'r', 'LineWidth', 2);
title('fitnlm - Nonlinear Regression');
hold on;
residuals_fitnlm = mdl_fitnlm.Residuals.Raw;
plot(voltage, residuals_fitnlm, 'r.', 'MarkerSize', 8);
hold off;

% 3. Using glmfit for Generalized Linear Models (GLMs)
X = [ones(size(voltage)) voltage];
b_ridge = ridge(moisture, X, 3); % lambda is the regularization parameter
fittedValues_ridge = X * b_ridge;
subplot(2, 3, 3);
plot(voltage, fittedValues_ridge, 'c', 'LineWidth', 2);
title('glmfit - Ridge Regression');
hold on;
residuals_ridge = moisture - fittedValues_ridge;
plot(voltage, residuals_ridge, 'r.', 'MarkerSize', 8);
hold off;

% 4. Using polyfit for Polynomial Regression
degree = 10; % Change the degree as needed
p = polyfit(voltage, moisture, degree);
fittedValues_polyfit = polyval(p, voltage);
subplot(2, 3, 4);
plot(voltage, fittedValues_polyfit, 'm', 'LineWidth', 2);
title('polyfit - Polynomial Regression');
hold on;
residuals_polyfit = moisture - fittedValues_polyfit;
plot(voltage, residuals_polyfit, 'r.', 'MarkerSize', 8);
hold off;

% 5. Using regress for Multivariate Linear Regression
X_regress = [ones(size(voltage)) voltage]; % Assuming simple linear regression
b_regress = regress(moisture, X_regress);
fittedValues_regress = X_regress * b_regress;
subplot(2, 3, 5);
plot(voltage, fittedValues_regress, 'b', 'LineWidth', 2);
title('regress - Multivariate Linear Regression');
hold on;
residuals_regress = moisture - fittedValues_regress;
plot(voltage, residuals_regress, 'r.', 'MarkerSize', 8);
hold off;

% Error plots
subplot(2, 3, 6);
plot(voltage, residuals_fitlm, 'b.', 'MarkerSize', 8);
hold on;
plot(voltage, residuals_fitnlm, 'g.', 'MarkerSize', 8);
plot(voltage, residuals_ridge, 'c.', 'MarkerSize', 8);
plot(voltage, residuals_polyfit, 'm.', 'MarkerSize', 8);
plot(voltage, residuals_regress, 'y.', 'MarkerSize', 8);
xlabel('Voltage');
ylabel('Residuals');
title('Residuals of Different Methods');
legend('fitlm', 'fitnlm', 'glmfit (ridge)', 'polyfit', 'regress');
hold off;

% Create a new figure displaying only the input data
figure;
scatter(voltage, moisture, 'filled');
xlabel('Voltage');
ylabel('Moisture');
title('Input Data (Voltage vs. Moisture)');
grid on;
