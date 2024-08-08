% live plot
figure(5050);
x_rear = x_hist(end) + x_bar*cos(theta_hist(end));
y_rear = y_hist(end) + x_bar*sin(theta_hist(end));
delta = Delta_hist_deg(end)*pi/180;

Coupler_estim_loc = Coupler_estim_loc_hist (1:3,end);

drawVehicle(x_rear, y_rear, theta_hist(end), delta, w, l, L, x_bar);

drawTrailer_angle(0, 0,trailer_angle, w, l, x_bar);hold on;
scatter(Noisy_coupler_location_hist(1,end), Noisy_coupler_location_hist(2,end),55,"filled",'o','MarkerFaceColor','m');
if Use_Kalman
    scatter(Coupler_estim_loc(1), Coupler_estim_loc(2),99,'g+','LineWidth',1.8);
end

hold on; plot(x_hist(1:end),y_hist(1:end),'r-.');

if Stanley_Approach==1
    Stanley_x_path = Coupler_estim_loc(1) + (0:100:15000) * cos(trailer_angle);
    Stanley_y_path = Coupler_estim_loc(2) + (0:100:15000) * sin(trailer_angle);
    plot(Stanley_x_path,Stanley_y_path, '-k','Color',[0,0,0,.2]);
end

% Plot future traj based on current steering
[traj_x,traj_y] = get_future_traj(x_animation,y_animation,theta_animation, v, delta, L, a,x_bar,Ts);
plot(traj_x,traj_y,':k');

delete(findall(gcf,'type','annotation'));

if delta>RWA_max || delta <-RWA_max
    rwa_color = 'red';
else
    rwa_color = 'black';
end

if Show_RWA_on_video==1
    A1 = annotation('textbox', [0.2, 0.73, 0.1, 0.1],  'String', {['rwa = ', num2str(floor(delta*180/pi*10)/10),' deg']}, ...
        'FontSize', 12,  'FontWeight', 'bold',  'Color', rwa_color, 'BackgroundColor', 'white',  'EdgeColor', 'black', ...
        'HorizontalAlignment', 'center',  'VerticalAlignment', 'middle');
    set(A1, 'horizontalAlignment', 'left');
end

if i==2
    axis auto;
    Axis = axis;
end
axis(Axis);
hold off;

drawnow;