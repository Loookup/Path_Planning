%% Initializing

close all;
clc;

%% Initial Setting

y_start = 6;
x_start = 3;
y_end = 3;
x_end = 13;

figure(1)
hold on

%% Plot
 
for i = 1: 10
    figure(1)
    scatter(1:14, i, 1, 'blue')
    hold on
 
end

figure(1)
scatter(x_start, y_start, 100,'green','filled')
hold on

figure(1)
scatter(x_end, y_end, 100, 'magenta', 'filled')
hold on

for i = 1:8
     figure(1)
     scatter(10, i, 30, "black", 'filled')
     hold on
 
end

%% Obstacle

Heuristic = zeros(10, 14);

for i = 1:8

     Heuristic(i, 10) = NaN;

end


%% Heuristic

for i = 1:10

    dy = abs(y_end - i);

    for j = 1:14

        dx = abs(x_end - j);

        if ~isnan(Heuristic(i, j))

            Heuristic(i, j) = dx + dy;

        end
    end
end


%% G and Heuristic

G_matrix = zeros(10, 14);
G_matrix_TF = zeros(10, 14);

for i = 1:8

     G_matrix_TF(i, 10) = NaN;
end

G_matrix(y_start, x_start) = 1;
G_matrix_TF(y_start, x_start) = NaN;

[G_matrix, G_matrix_TF] = Move_a_Step(y_start, x_start, G_matrix, G_matrix_TF);

F = Heuristic + G_matrix;

while G_matrix_TF(y_end, x_end) == 0

    where = find(G_matrix_TF == 1);

    list = [F(where)];

    [min, max] = bounds(list);

    next = find(list == min);

    index = where(next(1, 1));

    x_next = floor(index / 10) + 1;

    y_next = mod(index, 10);

    [G_matrix, G_matrix_TF] = Move_a_Step(y_next, x_next, G_matrix, G_matrix_TF);

    F = Heuristic + G_matrix;

    figure(1)
    scatter(x_next, y_next, 'red')
    hold on

end

Final_Path(x_end, y_end, x_start, y_start, F, G_matrix_TF)

%% G Matrix Generator

function[mtx, tf_mtx] = Move_a_Step(cur_y, cur_x, mtx, tf_mtx)

    tf_mtx(cur_y, cur_x) = NaN;
    y_start = 6;
    x_start = 3;

    for i = 1:3
       
        for j = 1:3

            if mod(i+j, 2) ~= 0

                if cur_y-2+i > 0 && cur_y-2+i < 11 && cur_x-2+j > 0 && cur_x-2+j < 15

                    if tf_mtx(cur_y-2+i, cur_x-2+j) == 0

                        dy = abs(cur_y-2+i - y_start);
                        dx = abs(cur_x-2+j - x_start);

                        mtx(cur_y-2+i, cur_x-2+j) = mtx(cur_y-2+i, cur_x-2+j) + dx + dy;
                        tf_mtx(cur_y-2+i, cur_x-2+j) = 1;

                    end   
                end
            end
        end
    end
end


%% Final Path Planning

function[] = Final_Path(x_end, y_end, x_start, y_start, mtx, tf)

    mtx_tf = tf;
    
    for i = 1:8
     
          mtx_tf(i, 10) = 0;
    end

    cur_x= x_end;
    cur_y = y_end;
    
    while x_end ~= x_start  || y_end ~= y_start
        
            num = 1;
            adh = 0;
            idx = 0;
        
            if isnan(mtx_tf(cur_y, cur_x-1))
    
                adh = 1;
                idx = idx+1;        
                cri = mtx(cur_y, cur_x-1);   
                num = 1;

            end
        
            if isnan(mtx_tf(cur_y-1, cur_x))

                adh = 2;
                idx = idx+1;
        
                if cri > mtx(cur_y-1, cur_x)
          
                    cri = mtx(cur_y-1, cur_x);
                    num = 2;
        
                end
            end
        
            if isnan(mtx_tf(cur_y, cur_x+1))

                adh = 3;
                idx = idx+1;
        
                if cri > mtx(cur_y, cur_x+1)
    
                    cri = mtx(cur_y, cur_x+1);
                    num = 3;
        
                end
            end
        
            if isnan(mtx_tf(cur_y+1, cur_x))

                adh = 4;
                idx = idx+1;
        
                if cri > mtx(cur_y+1, cur_x)
        
                    cri = mtx(cur_y+1, cur_x);       
                    num = 4;
        
                end
            end

            if idx == 1

                num = adh;

            end
   
            mtx_tf(cur_y, cur_x) = 0;
        
            switch num
        
                case 1
                    cur_x = cur_x - 1;
        
                case 2
                    cur_y = cur_y - 1;
        
                case 3
                    cur_x = cur_x + 1;
        
                 case 4
                    cur_y = cur_y + 1;
        
                otherwise
                    disp("Error")       
            end

            figure(1)
            scatter(cur_x, cur_y, 100,'red','filled')
            hold on
        
            if cur_x == x_start+1 && cur_y == y_start

                disp("End of A star Algorithm")
        
                break
            end       
    end
end