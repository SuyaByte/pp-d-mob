function[]=proj_1(mobility_model_parameter, velocity_parameter, pausing_time_parameter, k_parameter, number_of_lines_parameter)

network_size = 1000;
pausing_time = pausing_time_parameter;
mobility_model = mobility_model_parameter;
velocity = velocity_parameter;
k = k_parameter;
num_lines = number_of_lines_parameter+1;


start_points = [0, 0];
 
x = zeros(num_lines, 2);
y = zeros(num_lines, 2);
%z = zeros(num_lines, 2);
 
x(1,1) = start_points(1);
y(1,1) = start_points(2);
%z(1,1) = start_points(3);
 
for i = 2:num_lines
    x(i,1) = x(i-1,2);
    y(i,1) = y(i-1,2);
    %z(i,1) = z(i-1,2);    
     
    x(i,2) = rand() * network_size;
    y(i,2) = rand() * network_size;
    %z(i,2) = rand() * network_size;
end

if mobility_model == 0
    for i = 1:num_lines
        plot(x(i,:), y(i,:));
        hold on
    end
 
    %plot([x(1,1), x(num_lines,2)], [y(1,1), y(num_lines,2)], 'k-', 'LineWidth', 2);
 
    point_pos = [x(1,1) y(1,1)];
    plot(point_pos(1), point_pos(2), 'r*');

    time_step = 1;  

    total_distance = 0;  

    flying_time = 0;
    for i = 1:num_lines     
        dist = norm([x(i,2)-x(i,1) y(i,2)-y(i,1)]);
        total_distance = total_distance + dist;  
    
        time_needed = dist / velocity;
    
        num_steps = ceil(time_needed / time_step); %taking time_step 1 would cause the drone to move for every 1 sec
     
        step_vector = [x(i,2)-x(i,1) y(i,2)-y(i,1)] / num_steps;
    
        time_taken = time_needed + pausing_time;

        flying_time = flying_time + time_taken;
        for j = 1:num_steps     
            point_pos = point_pos + step_vector;
        
            plot(point_pos(1), point_pos(2), 'r*');
         
            pause(0.1);
        end
    end
    fprintf('Total distance covered: %f meter\n', total_distance);
    fprintf('Total flying time: %f sec\n', flying_time);
 
    xlabel('X-axis');
    ylabel('Y-axis');
    title('A mobility model of Drone');
elseif mobility_model == 1
    time_step = 1;  

    total_distance = 0;  

    flying_time = 0;

    dist_array = [0];
    point_pos = [x(1,1) y(1,1)];
    for i = 1:num_lines 
        points = [x(i,1), y(i,1);
          x(i,2), y(i,1);
          x(i,2), y(i,2);
          x(i,1), y(i,2)];
        center_x = (points(1,1) + points(3,1)) / 2;
        center_y = (points(1,2) + points(3,2)) / 2;
        side1_length = abs(points(2,1) - points(1,1));
        side2_length = abs(points(3,2) - points(2,2));
        rectangle('Position', [center_x - side1_length/2, center_y - side2_length/2, side1_length, side2_length], 'EdgeColor', 'r');
        hold on;
        plot([points(1,1), points(3,1)], [points(1,2), points(3,2)], 'r--');
        hold on;
        plot(points(:,1), points(:,2), 'r.', 'MarkerSize', 12);
        hold on;
        random_x = center_x + (2*rand(k,1) - 1) * side1_length/2;
        random_y = center_y + (2*rand(k,1) - 1) * side2_length/2;
        plot(random_x, random_y, 'g.', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
        for j = 1:k
            dist = norm([random_x(j)-x(i,1) random_y(j)-y(i,1)]);
            dist_array = [dist_array; dist];
            dist_array_2 = nonzeros(dist_array);
        end
        min_dist = min(dist_array_2);
        min_index = find(dist_array_2 == min_dist);
        max_dist = max(dist_array_2);
        max_index = find(dist_array_2 == max_dist);
        if ~isempty(dist_array_2) 
            dist_array = [0];
            if k == 2
                plot([x(i,1), random_x(min_index)], [y(i,1), random_y(min_index)]);
                d1 = norm([random_x(min_index)-x(i,1) random_y(min_index)-y(i,1)]);
                plot([random_x(min_index), random_x(max_index)], [random_y(min_index), random_y(max_index)]);
                d2 = norm([random_x(max_index)-random_x(min_index) random_y(max_index)-random_y(min_index)]);
                plot([random_x(max_index), x(i,2)], [random_y(max_index),  y(i,2)]);
                d3 = norm([x(i,2)-random_x(max_index) y(i,2)-random_y(max_index)]);
                dist = d1 + d2 + d3;
                total_distance = total_distance + dist; 
                time_needed = dist / velocity;
    
                num_steps = ceil(time_needed / time_step); %taking time_step 1 would cause the drone to move for every 1 sec
     
                step_vector = [x(i,2)-x(i,1) y(i,2)-y(i,1)] / num_steps;
    
                time_taken = time_needed + pausing_time;

                flying_time = flying_time + time_taken;
                
                pause(pausing_time);
            elseif k == 3
                temp = [1:3];
                temp(temp == min_index) = [];
                temp(temp == max_index) = [];
                plot([x(i,1), random_x(min_index)], [y(i,1), random_y(min_index)]);
                d1 = norm([random_x(min_index)-x(i,1) random_y(min_index)-y(i,1)]);
                plot([random_x(min_index), random_x(temp)], [random_y(min_index), random_y(temp)]);    
                d2 = norm([random_x(temp)-random_x(min_index) random_y(temp)-random_y(min_index)]);
                plot([random_x(temp), random_x(max_index)], [random_y(temp), random_y(max_index)]);  
                d3 = norm([random_x(max_index)-random_x(temp) random_y(max_index)-random_y(temp)]);
                plot([random_x(max_index), x(i,2)], [random_y(max_index), y(i,2)]); 
                d4 = norm([x(i,2)-random_x(max_index) y(i,2)-random_y(max_index)]);
                dist = d1 + d2 + d3 + d4;
                total_distance = total_distance + dist; 
                time_needed = dist / velocity;
    
                num_steps = ceil(time_needed / time_step); %taking time_step 1 would cause the drone to move for every 1 sec
     
                step_vector = [x(i,2)-x(i,1) y(i,2)-y(i,1)] / num_steps;
    
                time_taken = time_needed + pausing_time;

                flying_time = flying_time + time_taken;
                
                
                pause(pausing_time);
            end
        end
    end
    fprintf('Total distance covered: %f meter\n', total_distance);
    fprintf('Total flying time: %f sec\n', flying_time);
    xlabel('X-axis');
    ylabel('Y-axis');
    title('A mobility model of Drone');
end

    