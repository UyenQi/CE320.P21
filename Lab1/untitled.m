x = 0:0.1:50; % Miền giá trị nhiệt độ (0 đến 50°C)
cold = trimf(x, [0, 0, 20]); % Lạnh (Cold)
normal = trimf(x, [10, 25, 40]); % Bình thường (Normal)
hot = trimf(x, [30, 50, 50]); % Nóng (Hot)
plot(x, cold, 'b', 'LineWidth', 2);
hold on;
plot(x, normal, 'g', 'LineWidth', 2);
plot(x, hot, 'r', 'LineWidth', 2);
hold off;
xlabel('Nhiệt độ (°C)');
ylabel('Mức độ thành viên');
title('Hàm thành viên tam giác');
legend('Lạnh', 'Bình thường', 'Nóng');
grid on; 
