function BayPlacementGRASP()
    % --- 1. CONFIGURACIÓN ---
    res = 200;              
    max_iterations = 5000;  % El límite real será el tiempo
    rcl_size = 3;           
    
    % --- 2. CARGA DE DATOS ---
    warehouse_coords = readmatrix('warehouse.csv');
    obstacles = readmatrix('obstacles.csv');
    ceiling = readmatrix('ceiling.csv');
    bays = readmatrix('types_of_bays.csv');
    ceiling = sortrows(ceiling, 1);
    
    % --- 3. PREPARACIÓN DEL GRID ---
    min_x = min(warehouse_coords(:,1)); max_x = max(warehouse_coords(:,1));
    min_y = min(warehouse_coords(:,2)); max_y = max(warehouse_coords(:,2));
    cols = ceil((max_x - min_x) / res);
    rows = ceil((max_y - min_y) / res);
    [X_grid, Y_grid] = meshgrid(min_x:res:(max_x-res), min_y:res:(max_y-res));
    
    in_wh = inpolygon(X_grid, Y_grid, warehouse_coords(:,1), warehouse_coords(:,2));
    base_grid = ~in_wh; 
    
    if ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            ox = obstacles(i,1); oy = obstacles(i,2); ow = obstacles(i,3); od = obstacles(i,4);
            obs_mask = (X_grid >= ox) & (X_grid < ox + ow) & (Y_grid >= oy) & (Y_grid < oy + od);
            base_grid = base_grid | obs_mask;
        end
    end
    total_free_cells = sum(~base_grid, 'all');
    
    % --- 4. HEURÍSTICA ---
    bay_efficiency = bays(:,6) ./ bays(:,7); 
    bay_area = bays(:,2) .* bays(:,3);       
    [~, sort_idx] = sort((bay_efficiency .* 1000 + bay_area ./ 10000), 'descend');
    sorted_bays = bays(sort_idx, :);
    
    % --- 5. BUCLE GRASP CON TIEMPO (29s) ---
    best_quality = 0; % Inicializamos en 0 para medir mejoras
    best_solution = [];
    best_area_pct = 0;
    
    fprintf('Optimizando... Límite: 29 segundos.\n');
    t_start = tic; 
    
    for iter = 1:max_iterations
        % Control de tiempo estricto
        if toc(t_start) >= 29
            fprintf('\n--- LÍMITE DE 29s ALCANZADO ---');
            break; 
        end
        
        grid_map = base_grid;
        solution = [];
        sum_price = 0; sum_loads = 0; used_cells = 0;
        placed_something = true;
        
        while placed_something
            placed_something = false;
            for c = 1:cols
                real_x = min_x + (c-1)*res;
                ceil_h = interp1(ceiling(:,1), ceiling(:,2), real_x, 'previous', 'extrap');
                for r_row = 1:rows
                    if grid_map(r_row, c) == 0 
                        real_y = min_y + (r_row-1)*res;
                        valid_bays_idx = find((sorted_bays(:,4) + sorted_bays(:,5)) <= ceil_h);
                        if isempty(valid_bays_idx), continue; end
                        
                        p_lim = min(rcl_size, length(valid_bays_idx));
                        pool = valid_bays_idx(1:p_lim);
                        pool = pool(randperm(length(pool)));
                        
                        for p_idx = 1:length(pool)
                            b = pool(p_idx);
                            id = sorted_bays(b,1); w = sorted_bays(b,2); d = sorted_bays(b,3); 
                            nL = sorted_bays(b,6); pr = sorted_bays(b,7);
                            cw = ceil(w / res); cd = ceil(d / res);
                            
                            rots = [0, cw, cd; 90, cd, cw]; 
                            r_ord = randperm(2);
                            
                            for ri = 1:2
                                r_type = rots(r_ord(ri), 1); wc = rots(r_ord(ri), 2); dc = rots(r_ord(ri), 3);
                                if (c+wc-1 > cols) || (r_row+dc-1 > rows), continue; end
                                if sum(grid_map(r_row:r_row+dc-1, c:c+wc-1), 'all') == 0
                                    solution(end+1, :) = [id, real_x, real_y, r_type];
                                    grid_map(r_row:r_row+dc-1, c:c+wc-1) = 1; 
                                    sum_loads = sum_loads + nL;
                                    sum_price = sum_price + pr;
                                    used_cells = used_cells + (wc * dc);
                                    placed_something = true;
                                    break;
                                end
                            end
                            if placed_something, break; end
                        end
                    end
                    if placed_something, break; end
                end
                if placed_something, break; end
            end
        end
        
        % Cálculo de mejora
        if sum_loads > 0
            pct_area = used_cells / total_free_cells;
            quality = (sum_price / sum_loads) ^ pct_area; 
            
            if quality > best_quality
                improvement = quality - best_quality;
                best_quality = quality;
                best_solution = solution;
                best_area_pct = pct_area * 100;
                
                fprintf('\nIter %d: ¡Récord! Calidad: %.4f (Mejora: +%.4f) | Área: %.2f%% | Tiempo: %.1fs', ...
                        iter, best_quality, improvement, best_area_pct, toc(t_start));
            end
        end
    end
    
    % --- 6. RESULTADOS ---
    fprintf('\n\n--- RESUMEN ---');
    fprintf('\nTiempo: %.2f s | Área: %.2f%% | Calidad Final: %.4f\n', toc(t_start), best_area_pct, best_quality);
    
    if ~isempty(best_solution)
        T = array2table(best_solution, 'VariableNames', {'Id', 'X', 'Y', 'Rotation'});
        writetable(T, 'expected_output.csv');
        plot_warehouse_solution(warehouse_coords, obstacles, bays, best_solution, best_area_pct, best_quality);
    end
end

function plot_warehouse_solution(warehouse_coords, obstacles, bays, solution, best_area_pct, best_quality)
    figure('Name', 'Visualización Exacta (Sin Gaps)', 'Color', 'w');
    hold on; axis equal;
    
    % Almacén
    plot(polyshape(warehouse_coords(:,1), warehouse_coords(:,2)), 'FaceColor', [0.98 0.98 0.98]);
    
    % Bahías (Coordenadas Reales)
    cmap = lines(max(bays(:,1)) + 1);
    for s = 1:size(solution, 1)
        id = solution(s, 1); bx = solution(s, 2); by = solution(s, 3); rot = solution(s, 4);
        b_idx = find(bays(:,1) == id, 1);
        bw = bays(b_idx, 2); bd = bays(b_idx, 3);
        if rot == 90, t = bw; bw = bd; bd = t; end
        
        % Dibujamos sin GAP visual
        rectangle('Position', [bx, by, bw, bd], 'FaceColor', cmap(id+1,:), 'EdgeColor', 'k');
        text(bx+bw/2, by+bd/2, num2str(id), 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', 'k');
    end
    
    % Obstáculos (Borde rojo grueso para ver colisiones)
    if ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            rectangle('Position', [obstacles(i,1), obstacles(i,2), obstacles(i,3), obstacles(i,4)], ...
                      'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'r');
        end
    end
    title(sprintf('Acomodo Exacto - Área: %.2f%%', best_area_pct));
    grid on;
end