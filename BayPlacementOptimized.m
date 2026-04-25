function [best_solution, best_cost, best_area_pct] = BayPlacementOptimized(warehouse_coords, obstacles, ceiling, bays)
    % --- 1. CONFIGURACIÓN ---
    % res = 0 indica al algoritmo que calcule la mejor resolución dinámicamente
    res = 0;                
    time_limit = 29;        % Límite estricto para la ejecución
    
    area_base = 2.0;        
    area_weight = 1.0;      
    
    % --- 2. VALIDACIÓN DE DATOS ---
    if isempty(warehouse_coords) || size(warehouse_coords, 1) < 3
        fprintf('\nRESULTADO: Quantity = Infinito\nMOTIVO: No hay almacén.\n');
        best_solution = []; best_cost = Inf; best_area_pct = 0;
        return;
    end
    if isempty(bays)
        fprintf('\nRESULTADO: Quantity = Infinito\nMOTIVO: No hay bays.\n');
        best_solution = []; best_cost = Inf; best_area_pct = 0;
        return;
    end
    
    if isempty(obstacles)
        obstacles = zeros(0, 4);
    end
    
    ceiling = sortrows(ceiling, 1);
    
    % Inferir el res perfecto si está en 0
    if res <= 0
        res = infer_optimal_resolution(warehouse_coords);
    end
    
    % --- 3. PREPARACIÓN DEL GRID ---
    min_x = min(warehouse_coords(:,1)); max_x = max(warehouse_coords(:,1));
    min_y = min(warehouse_coords(:,2)); max_y = max(warehouse_coords(:,2));
    
    cols = ceil((max_x - min_x) / res);
    rows = ceil((max_y - min_y) / res);
    
    x_vec = min_x + (0:cols-1) * res;
    y_vec = min_y + (0:rows-1) * res;
    [X_grid, Y_grid] = meshgrid(x_vec, y_vec);
    
    base_grid = build_base_grid(X_grid, Y_grid, res, warehouse_coords, obstacles);
    total_free_cells = sum(base_grid == 0, 'all');
    
    if total_free_cells < 4 
        fprintf('\nRESULTADO: Quantity = Infinito\nMOTIVO: Almacén muy pequeño.\n');
        best_solution = []; best_cost = Inf; best_area_pct = 0;
        return;
    end
    
    ceil_grid = zeros(1, cols);
    for c = 1:cols
        real_x = min_x + (c-1)*res;
        ceil_grid(c) = interp1(ceiling(:,1), ceiling(:,2), real_x, 'previous', 'extrap');
    end
    
    % --- 4. PREPARACIÓN DE BAYS ---
    cost_ratio = bays(:,7) ./ bays(:,6); 
    [~, sort_idx] = sort(cost_ratio, 'ascend');
    sorted_bays = bays(sort_idx, :);
    
    % --- 5. BUCLE PRINCIPAL METAHEURÍSTICO ---
    best_cost = Inf; 
    best_solution = [];
    best_area_pct = 0;
    
    fprintf('Optimizando Costes (Res Dinámico = %d)... Límite: %d s.\n', res, time_limit);
    t_start = tic; 
    iter = 0;
    
    while toc(t_start) < time_limit
        iter = iter + 1;
        
        % Reinicio Estocástico: 20% de probabilidad de retomar la mejor solución
        if rand() < 0.20 && ~isempty(best_solution)
            current_solution = best_solution;
            grid_map = rebuild_map_from_solution(current_solution, base_grid);
        else
            current_solution = [];
            grid_map = base_grid; 
        end
        
        [current_solution, grid_map] = fill_pass(current_solution, grid_map, base_grid, cols, rows, res, min_x, min_y, ceil_grid, sorted_bays);
        [best_cost, best_solution, best_area_pct] = evaluate_and_save(current_solution, best_cost, best_solution, best_area_pct, total_free_cells, iter, 0, t_start, area_base, area_weight);
        
        for ls = 1:4
            if toc(t_start) >= time_limit, break; end
            
            [current_solution, grid_map] = compact_pass(current_solution, base_grid, cols, rows, res, min_x, min_y, ceil_grid);
            [current_solution, grid_map] = fill_pass(current_solution, grid_map, base_grid, cols, rows, res, min_x, min_y, ceil_grid, sorted_bays);
            [best_cost, best_solution, best_area_pct] = evaluate_and_save(current_solution, best_cost, best_solution, best_area_pct, total_free_cells, iter, ls, t_start, area_base, area_weight);
            
            [current_solution, grid_map] = ruin_pass(current_solution, base_grid);
        end
    end
    
    % --- 6. RESULTADOS ---
    if isempty(best_solution)
        fprintf('\nRESULTADO: Quantity = Infinito\nMOTIVO: No se encontró solución.\n');
    else
        fprintf('\n--- RESUMEN FINAL ---\n');
        fprintf('Tiempo: %.2f s | Coste: %.4f | Área: %.2f%% | Racks: %d\n', ...
                toc(t_start), best_cost, best_area_pct, size(best_solution, 1));
        plot_warehouse_solution(warehouse_coords, obstacles, bays, best_solution, best_area_pct, best_cost);
    end
end

% =========================================================================
%                       FUNCIONES AUXILIARES
% =========================================================================

function res = infer_optimal_resolution(warehouse_coords)
    min_x = min(warehouse_coords(:,1)); max_x = max(warehouse_coords(:,1));
    min_y = min(warehouse_coords(:,2)); max_y = max(warehouse_coords(:,2));
    bbox_area = (max_x - min_x) * (max_y - min_y);
    target_cells = 12500; 
    ideal_res = sqrt(bbox_area / target_cells);
    res = round(ideal_res / 50) * 50;
    res = max(50, min(400, res));
end

function map = rebuild_map_from_solution(sol, base_map)
    map = base_map;
    if isempty(sol), return; end
    for i = 1:size(sol, 1)
        r_type = sol(i, 4); c = sol(i, 8); r_row = sol(i, 9);
        wc = sol(i, 10); dc = sol(i, 11); cg = sol(i, 13);
        map = mark_grid(map, r_row, c, wc, dc, cg, r_type);
    end
end

function [sol, map] = fill_pass(sol, map, base_map, cols, rows, res, min_x, min_y, ceil_grid, sorted_bays)
    scan_c = 1:cols; if rand() > 0.5, scan_c = cols:-1:1; end
    scan_r = 1:rows; if rand() > 0.5, scan_r = rows:-1:1; end
    
    free_cells_left = sum(map(:) == 0);
    total_free_cells = sum(base_map(:) == 0);
    pool_base = 4;
    if free_cells_left < total_free_cells * 0.25
        pool_base = 10;
    end
    
    for c = scan_c
        valid_bays = sorted_bays(sorted_bays(:,4) <= ceil_grid(c), :);
        if isempty(valid_bays), continue; end
        pool_size = min(pool_base, size(valid_bays, 1));
        
        for r_row = scan_r
            if map(r_row, c) == 0 || map(r_row, c) == 2 
                pool_idx = randperm(pool_size); 
                for p = 1:pool_size
                    bay = valid_bays(pool_idx(p), :);
                    id = bay(1); w = bay(2); d = bay(3);
                    bay_h = bay(4); gap_val = bay(5); 
                    cw = ceil(w/res); cd = ceil(d/res); cg = ceil(gap_val/res); 
                    rots = [0, cw, cd; 90, cd, cw; 180, cw, cd; 270, cd, cw]; 
                    
                    best_overlap = -1;
                    best_r_type = -1; best_wc = 0; best_dc = 0;
                    
                    for ri = 1:4
                        r_type = rots(ri, 1); wc = rots(ri, 2); dc = rots(ri, 3);
                        if r_row + dc - 1 > rows || c + wc - 1 > cols, continue; end
                        span_ceil = min(ceil_grid(max(1, c) : min(cols, c+wc-1))); 
                        if bay_h > span_ceil, continue; end
                        
                        if can_place_grid(map, r_row, c, wc, dc, cg, r_type)
                            [grmin, grmax, gcmin, gcmax] = get_gap_bounds(r_row, c, wc, dc, cg, r_type);
                            gap_region = map(grmin:grmax, gcmin:gcmax);
                            overlap_score = sum(gap_region(:) == 2); 
                            if overlap_score > 0
                                if gcmin == 1 || gcmax == cols || grmin == 1 || grmax == rows
                                    overlap_score = overlap_score + 2;
                                end
                                overlap_score = overlap_score * 1.5; 
                            end
                            if overlap_score > best_overlap
                                best_overlap = overlap_score;
                                best_r_type = r_type; best_wc = wc; best_dc = dc;
                            end
                        end
                    end
                    
                    if best_overlap >= 0
                        map = mark_grid(map, r_row, c, best_wc, best_dc, cg, best_r_type);
                        real_x = min_x + (c-1)*res; real_y = min_y + (r_row-1)*res;
                        sol(end+1, :) = [id, real_x, real_y, best_r_type, bay(6), bay(7), best_wc*best_dc, c, r_row, best_wc, best_dc, bay_h, cg];
                        break;
                    end
                end
            end
        end
    end
end

function [sol, map] = compact_pass(sol, base_map, cols, rows, res, min_x, min_y, ceil_grid)
    map = base_map;
    if isempty(sol), return; end
    [~, sort_idx] = sortrows(sol, [8, 9]); 
    sol = sol(sort_idx, :);
    for i = 1:size(sol, 1)
        r_type = sol(i, 4); c = sol(i, 8); r_row = sol(i, 9);
        wc = sol(i, 10); dc = sol(i, 11); bay_h = sol(i, 12); cg = sol(i, 13);
        moved = true;
        while moved
            moved = false;
            if c > 1 && can_place_grid(map, r_row, c-1, wc, dc, cg, r_type)
                if bay_h <= min(ceil_grid(max(1, c-1) : min(cols, c+wc-2))) 
                    c = c - 1; moved = true;
                end
            end
            if r_row > 1 && can_place_grid(map, r_row-1, c, wc, dc, cg, r_type)
                r_row = r_row - 1; moved = true;
            end
        end
        sol(i, 8) = c; sol(i, 9) = r_row;
        sol(i, 2) = min_x + (c-1)*res; sol(i, 3) = min_y + (r_row-1)*res;
        if can_place_grid(map, r_row, c, wc, dc, cg, r_type)
            map = mark_grid(map, r_row, c, wc, dc, cg, r_type);
        else
            sol(i, :) = nan;
        end
    end
    sol = sol(~isnan(sol(:, 1)), :);
end

function [sol, map] = ruin_pass(sol, base_map)
    num_items = size(sol, 1);
    if num_items == 0, map = base_map; return; end
    keep_count = max(1, floor(num_items * 0.80));
    keep_idx = randperm(num_items, keep_count);
    sol = sol(keep_idx, :);
    map = base_map;
    for i = 1:size(sol, 1)
        r_type = sol(i, 4); c = sol(i, 8); r_row = sol(i, 9);
        wc = sol(i, 10); dc = sol(i, 11); cg = sol(i, 13);
        if can_place_grid(map, r_row, c, wc, dc, cg, r_type)
            map = mark_grid(map, r_row, c, wc, dc, cg, r_type);
        else
            sol(i, :) = nan;
        end
    end
    sol = sol(~isnan(sol(:, 1)), :);
end

function [gap_rmin, gap_rmax, gap_cmin, gap_cmax] = get_gap_bounds(r_row, c, wc, dc, cg, r_type)
    if r_type == 0
        gap_rmin = r_row + dc; gap_rmax = r_row + dc + cg - 1;
        gap_cmin = c; gap_cmax = c + wc - 1;
    elseif r_type == 90
        gap_rmin = r_row; gap_rmax = r_row + dc - 1;
        gap_cmin = c - cg; gap_cmax = c - 1;
    elseif r_type == 180
        gap_rmin = r_row - cg; gap_rmax = r_row - 1;
        gap_cmin = c; gap_cmax = c + wc - 1;
    elseif r_type == 270
        gap_rmin = r_row; gap_rmax = r_row + dc - 1;
        gap_cmin = c + wc; gap_cmax = c + wc + cg - 1;
    else
        gap_rmin = r_row; gap_rmax = r_row; gap_cmin = c; gap_cmax = c;
    end
end

function ok = can_place_grid(map, r_row, c, wc, dc, cg, r_type)
    [rows, cols] = size(map);
    if r_row < 1 || c < 1 || r_row + dc - 1 > rows || c + wc - 1 > cols
        ok = false; return;
    end
    [gap_rmin, gap_rmax, gap_cmin, gap_cmax] = get_gap_bounds(r_row, c, wc, dc, cg, r_type);
    if gap_rmin < 1 || gap_cmin < 1 || gap_rmax > rows || gap_cmax > cols
        ok = false; return;
    end
    if any(reshape(map(r_row:r_row+dc-1, c:c+wc-1), [], 1) ~= 0), ok = false; return; end
    if any(reshape(map(gap_rmin:gap_rmax, gap_cmin:gap_cmax), [], 1) == 1), ok = false; return; end
    ok = true;
end

function map = mark_grid(map, r_row, c, wc, dc, cg, r_type)
    [gap_rmin, gap_rmax, gap_cmin, gap_cmax] = get_gap_bounds(r_row, c, wc, dc, cg, r_type);
    map(gap_rmin:gap_rmax, gap_cmin:gap_cmax) = max(map(gap_rmin:gap_rmax, gap_cmin:gap_cmax), 2);
    map(r_row:r_row+dc-1, c:c+wc-1) = 1;
end

function base_grid = build_base_grid(X_grid, Y_grid, res, warehouse_coords, obstacles)
    [rows, cols] = size(X_grid);
    base_grid = zeros(rows, cols);
    for r = 1:rows
        for c = 1:cols
            x1 = X_grid(r, c); y1 = Y_grid(r, c); x2 = x1 + res; y2 = y1 + res;
            if ~rect_inside_warehouse(x1, y1, x2, y2, warehouse_coords) || rect_hits_obstacle(x1, y1, x2, y2, obstacles)
                base_grid(r, c) = 1;
            end
        end
    end
end

function ok = rect_inside_warehouse(x1, y1, x2, y2, warehouse_coords)
    corners = [x1, y1; x2, y1; x2, y2; x1, y2];
    [in, on] = inpolygon(corners(:,1), corners(:,2), warehouse_coords(:,1), warehouse_coords(:,2));
    if ~all(in | on), ok = false; return; end
    ok = true;
    rect_edges = [x1, y1, x2, y1; x2, y1, x2, y2; x2, y2, x1, y2; x1, y2, x1, y1];
    for e = 1:4
        a = rect_edges(e, 1:2); b = rect_edges(e, 3:4);
        for w = 1:size(warehouse_coords, 1)
            p = warehouse_coords(w, :); q = warehouse_coords(mod(w, size(warehouse_coords, 1)) + 1, :);
            v1 = b-a; v2 = p-a; v3 = q-a;
            o1 = v1(1)*v2(2) - v1(2)*v2(1); o2 = v1(1)*v3(2) - v1(2)*v3(1);
            v4 = q-p; v5 = a-p; v6 = b-p;
            o3 = v4(1)*v5(2) - v4(2)*v5(1); o4 = v4(1)*v6(2) - v4(2)*v6(1);
            if (o1*o2 < 0) && (o3*o4 < 0), ok = false; return; end
        end
    end
end

function hit = rect_hits_obstacle(x1, y1, x2, y2, obstacles)
    hit = false;
    for i = 1:size(obstacles, 1)
        if x1 < obstacles(i,1)+obstacles(i,3) && x2 > obstacles(i,1) && y1 < obstacles(i,2)+obstacles(i,4) && y2 > obstacles(i,2)
            hit = true; return;
        end
    end
end

function [best_c, best_s, best_a] = evaluate_and_save(sol, best_c, best_s, best_a, total_free, iter, ls, t_start, area_base, area_weight)
    if isempty(sol), return; end
    sum_loads = sum(sol(:, 5)); sum_price = sum(sol(:, 6)); used_cells = sum(sol(:, 7)); 
    pct_area = used_cells / total_free; 
    current_cost = (sum_price / sum_loads) ^ (area_base - (pct_area * area_weight));
    if current_cost < best_c
        best_c = current_cost; best_s = sol; best_a = pct_area * 100;
        if ls == 0, fprintf('Iter %d (Llenado Base)   : Coste: %.4f\n', iter, best_c);
        else, fprintf('Iter %d (Compactación %d): Coste: %.4f\n', iter, ls, best_c); end
    end
end

function plot_warehouse_solution(warehouse_coords, obstacles, bays, solution, best_area_pct, best_cost)
    figure('Name', 'Solución de Almacén', 'Color', 'w'); hold on; axis equal;
    plot(polyshape(warehouse_coords(:,1), warehouse_coords(:,2)), 'FaceColor', [0.98 0.98 0.98]);
    cmap = lines(max(bays(:,1)) + 1);
    for s = 1:size(solution, 1)
        id = solution(s, 1); bx = solution(s, 2); by = solution(s, 3); rot = solution(s, 4);
        b_idx = find(bays(:,1) == id, 1); bw = bays(b_idx, 2); bd = bays(b_idx, 3); gv = bays(b_idx, 5); 
        if rot == 90, rectangle('Position', [bx-gv, by, gv, bw], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--');
            rectangle('Position', [bx, by, bd, bw], 'FaceColor', cmap(id+1,:), 'EdgeColor', 'k');
            tx = bx + bd/2; ty = by + bw/2;
        elseif rot == 180, rectangle('Position', [bx, by-gv, bw, gv], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--');
            rectangle('Position', [bx, by, bw, bd], 'FaceColor', cmap(id+1,:), 'EdgeColor', 'k');
            tx = bx + bw/2; ty = by + bd/2;
        elseif rot == 270, rectangle('Position', [bx+bd, by, gv, bw], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--');
            rectangle('Position', [bx, by, bd, bw], 'FaceColor', cmap(id+1,:), 'EdgeColor', 'k');
            tx = bx + bd/2; ty = by + bw/2;
        else, rectangle('Position', [bx, by+bd, bw, gv], 'EdgeColor', [0.6 0.6 0.6], 'LineStyle', '--');
            rectangle('Position', [bx, by, bw, bd], 'FaceColor', cmap(id+1,:), 'EdgeColor', 'k');
            tx = bx + bw/2; ty = by + bd/2;
        end
        text(tx, ty, num2str(id), 'HorizontalAlignment', 'center', 'FontSize', 8);
    end
    for i = 1:size(obstacles, 1), rectangle('Position', [obstacles(i,1), obstacles(i,2), obstacles(i,3), obstacles(i,4)], 'FaceColor', [0.3 0.3 0.3]); end
    title(sprintf('Coste: %.2f | Área Útil: %.2f%%', best_cost, best_area_pct)); grid on;
end
