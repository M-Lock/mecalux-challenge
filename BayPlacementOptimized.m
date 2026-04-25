    function BayPlacementOptimized()
        % --- 1. CONFIGURACIÓN ---
        res = 200;            % Tamano fijo de celda; bays adyacentes pueden tocarse
        time_limit = 29;        % Límite estricto para la demo
        
        % Variables de la heurística de coste para modificar a placer:
        area_base = 2.0;        % Representa el "2" en (2 - AreaUsed)
        area_weight = 1.0;      % Multiplicador para darle más o menos peso al AreaUsed (1.0 = normal)
        
        % --- 2. CARGA DE DATOS ---
        warehouse_coords = readmatrix('warehouse.csv');
        obstacles = readmatrix('obstacles.csv');
        ceiling = readmatrix('ceiling.csv');
        bays = readmatrix('types_of_bays.csv');
        ceiling = sortrows(ceiling, 1);

        if isempty(obstacles)
            obstacles = zeros(0, 4);
        end

        if res <= 0
            res = infer_grid_resolution(warehouse_coords, obstacles, bays);
            fprintf('Resolucion automatica de rejilla: %.0f\n', res);
        end
        
        % --- 3. PREPARACIÓN DEL GRID ---
        min_x = min(warehouse_coords(:,1)); max_x = max(warehouse_coords(:,1));
        min_y = min(warehouse_coords(:,2)); max_y = max(warehouse_coords(:,2));
        cols = ceil((max_x - min_x) / res);
        rows = ceil((max_y - min_y) / res);
        [X_grid, Y_grid] = meshgrid(min_x:res:(max_x-res), min_y:res:(max_y-res));
        
        base_grid = build_base_grid(X_grid, Y_grid, res, warehouse_coords, obstacles);
        total_free_cells = sum(~base_grid, 'all');
        
        ceil_grid = zeros(1, cols);
        for c = 1:cols
            real_x = min_x + (c-1)*res;
            ceil_grid(c) = interp1(ceiling(:,1), ceiling(:,2), real_x, 'previous', 'extrap');
        end
        
        % --- 4. PREPARACIÓN DE BAYS (Orientado a MINIMIZAR COSTE) ---
        % Calculamos Precio / Carga (menor es mejor)
        cost_ratio = bays(:,7) ./ bays(:,6); 
        % Ordenamos de menor a mayor coste para que el GRASP use los más eficientes primero
        [~, sort_idx] = sort(cost_ratio, 'ascend');
        sorted_bays = bays(sort_idx, :);
        
        % --- 5. BUCLE PRINCIPAL METAHEURÍSTICO ---
        best_cost = Inf; % Ahora buscamos MINIMIZAR, empezamos en Infinito
        best_solution = [];
        best_area_pct = 0;
        
        fprintf('Optimizando Costes (GRASP + Local Search)... Límite: %d segundos.\n', time_limit);
        fprintf('Fórmula: (Suma Precio / Suma Carga) ^ (%.1f - (AreaUsed * %.1f))\n', area_base, area_weight);
        t_start = tic; 
        iter = 0;
        
        while toc(t_start) < time_limit
            iter = iter + 1;
            current_solution = [];
            grid_map = base_grid;
            
            % Fase 1: Construcción
            [current_solution, grid_map] = fill_pass(current_solution, grid_map, base_grid, cols, rows, res, min_x, min_y, ceil_grid, sorted_bays);
            [best_cost, best_solution, best_area_pct] = evaluate_and_save(current_solution, best_cost, best_solution, best_area_pct, total_free_cells, iter, 0, t_start, area_base, area_weight);
            
            % Fase 2: Búsqueda Local
            for ls = 1:4
                if toc(t_start) >= time_limit, break; end
                
                [current_solution, grid_map] = compact_pass(current_solution, base_grid, cols, rows, res, min_x, min_y, ceil_grid);
                [current_solution, grid_map] = fill_pass(current_solution, grid_map, base_grid, cols, rows, res, min_x, min_y, ceil_grid, sorted_bays);
                [best_cost, best_solution, best_area_pct] = evaluate_and_save(current_solution, best_cost, best_solution, best_area_pct, total_free_cells, iter, ls, t_start, area_base, area_weight);
                
                [current_solution, grid_map] = ruin_pass(current_solution, base_grid);
            end
        end
        
        % --- 6. RESULTADOS ---
        fprintf('\n\n--- RESUMEN FINAL ---\n');
        fprintf('Tiempo: %.2f s | Coste Final Mínimo: %.4f | Área: %.2f%% | Racks Totales: %d\n', ...
                toc(t_start), best_cost, best_area_pct, size(best_solution, 1));
        
        if ~isempty(best_solution)
            T = array2table(best_solution, 'VariableNames', {'Id', 'X', 'Y', 'Rotation'});
            writetable(T, 'expected_output.csv');
            plot_warehouse_solution(warehouse_coords, obstacles, bays, best_solution, best_area_pct, best_cost);
        end
    end
    
    % =========================================================================
    %                       FUNCIONES AUXILIARES
    % =========================================================================
    
    function [sol, map] = fill_pass(sol, map, base_map, cols, rows, res, min_x, min_y, ceil_grid, sorted_bays)
        for c = 1:cols
            valid_bays = sorted_bays((sorted_bays(:,4) + sorted_bays(:,5)) <= ceil_grid(c), :);
            if isempty(valid_bays), continue; end
            
            pool_size = min(4, size(valid_bays, 1));
            
            for r_row = 1:rows
                if map(r_row, c) == 0
                    pool_idx = randperm(pool_size); 
                    placed = false;
                    
                    for p = 1:pool_size
                        bay = valid_bays(pool_idx(p), :);
                        id = bay(1); w = bay(2); d = bay(3);
                        bay_h = bay(4) + bay(5);
                        
                        cw = ceil(w/res); cd = ceil(d/res);
                        rots = [0, cw, cd; 90, cd, cw]; 
                        r_ord = randperm(2);
                        
                        for ri = 1:2
                            r_type = rots(r_ord(ri), 1);
                            wc = rots(r_ord(ri), 2);
                            dc = rots(r_ord(ri), 3);
                            
                            if (c + wc - 1 <= cols) && (r_row + dc - 1 <= rows)
                                span_ceil = min(ceil_grid(c : c+wc-1)); 
                                if bay_h > span_ceil
                                    continue; 
                                end
                                
                                if can_place_grid(map, r_row, c, wc, dc)
                                    map = mark_grid(map, r_row, c, wc, dc);
                                    real_x = min_x + (c-1)*res;
                                    real_y = min_y + (r_row-1)*res;
                                    sol(end+1, :) = [id, real_x, real_y, r_type, bay(6), bay(7), wc*dc, c, r_row, wc, dc, bay_h];
                                    placed = true;
                                    break;
                                end
                            end
                        end
                        if placed, break; end
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
            c = sol(i, 8); r_row = sol(i, 9);
            wc = sol(i, 10); dc = sol(i, 11); bay_h = sol(i, 12);
            
            moved = true;
            while moved
                moved = false;
                if c > 1 && can_place_grid(map, r_row, c-1, wc, dc)
                    if bay_h <= min(ceil_grid(c-1:c+wc-2)) 
                        c = c - 1; moved = true;
                    end
                end
                if r_row > 1 && can_place_grid(map, r_row-1, c, wc, dc)
                    r_row = r_row - 1; moved = true;
                end
            end
            
            sol(i, 8) = c; sol(i, 9) = r_row;
            sol(i, 2) = min_x + (c-1)*res; sol(i, 3) = min_y + (r_row-1)*res;
            if can_place_grid(map, r_row, c, wc, dc)
                map = mark_grid(map, r_row, c, wc, dc);
            else
                sol(i, :) = nan;
            end
        end
        sol = sol(~isnan(sol(:, 1)), :);
    end
    
    function [sol, map] = ruin_pass(sol, base_map)
        num_items = size(sol, 1);
        if num_items == 0
            map = base_map; return;
        end
        
        keep_count = max(1, floor(num_items * 0.80));
        keep_idx = randperm(num_items, keep_count);
        sol = sol(keep_idx, :);
        
        map = base_map;
        for i = 1:size(sol, 1)
            c = sol(i, 8); r_row = sol(i, 9);
            wc = sol(i, 10); dc = sol(i, 11);
            if can_place_grid(map, r_row, c, wc, dc)
                map = mark_grid(map, r_row, c, wc, dc);
            else
                sol(i, :) = nan;
            end
        end
        sol = sol(~isnan(sol(:, 1)), :);
    end

    function ok = can_place_grid(map, r_row, c, wc, dc)
        [rows, cols] = size(map);
        if r_row < 1 || c < 1 || r_row + dc - 1 > rows || c + wc - 1 > cols
            ok = false;
            return;
        end

        ok = sum(map(r_row:r_row+dc-1, c:c+wc-1), 'all') == 0;
    end

    function map = mark_grid(map, r_row, c, wc, dc)
        map(r_row:r_row+dc-1, c:c+wc-1) = 1;
    end

    function res = infer_grid_resolution(warehouse_coords, obstacles, bays)
        values = [warehouse_coords(:); bays(:,2); bays(:,3)];

        if ~isempty(obstacles)
            values = [values; obstacles(:)];
        end

        values = abs(values(~isnan(values) & isfinite(values) & values > 0));
        values = round(values);

        if isempty(values)
            res = 200;
            return;
        end

        res = values(1);
        for i = 2:numel(values)
            res = gcd(res, values(i));
        end

        res = max(50, min(200, res));
    end

    function base_grid = build_base_grid(X_grid, Y_grid, res, warehouse_coords, obstacles)
        [rows, cols] = size(X_grid);
        base_grid = false(rows, cols);

        for r = 1:rows
            for c = 1:cols
                x1 = X_grid(r, c);
                y1 = Y_grid(r, c);
                x2 = x1 + res;
                y2 = y1 + res;

                if ~rect_inside_warehouse(x1, y1, x2, y2, warehouse_coords)
                    base_grid(r, c) = true;
                    continue;
                end

                if rect_hits_obstacle(x1, y1, x2, y2, obstacles)
                    base_grid(r, c) = true;
                end
            end
        end
    end

    function ok = rect_inside_warehouse(x1, y1, x2, y2, warehouse_coords)
        corners = [x1, y1; x2, y1; x2, y2; x1, y2];
        [in, on] = inpolygon(corners(:,1), corners(:,2), warehouse_coords(:,1), warehouse_coords(:,2));

        if ~all(in | on)
            ok = false;
            return;
        end

        ok = true;
        rect_edges = [x1, y1, x2, y1; x2, y1, x2, y2; x2, y2, x1, y2; x1, y2, x1, y1];
        for e = 1:size(rect_edges, 1)
            a = rect_edges(e, 1:2);
            b = rect_edges(e, 3:4);

            for w = 1:size(warehouse_coords, 1)
                p = warehouse_coords(w, :);
                q = warehouse_coords(mod(w, size(warehouse_coords, 1)) + 1, :);
                if segments_cross_strict(a, b, p, q)
                    ok = false;
                    return;
                end
            end
        end
    end

    function hit = rect_hits_obstacle(x1, y1, x2, y2, obstacles)
        hit = false;
        if isempty(obstacles), return; end

        for i = 1:size(obstacles, 1)
            ox1 = obstacles(i,1);
            oy1 = obstacles(i,2);
            ox2 = ox1 + obstacles(i,3);
            oy2 = oy1 + obstacles(i,4);

            if x1 < ox2 && x2 > ox1 && y1 < oy2 && y2 > oy1
                hit = true;
                return;
            end
        end
    end

    function cross = segments_cross_strict(a, b, c, d)
        o1 = cross2d(b - a, c - a);
        o2 = cross2d(b - a, d - a);
        o3 = cross2d(d - c, a - c);
        o4 = cross2d(d - c, b - c);
        cross = (o1 * o2 < 0) && (o3 * o4 < 0);
    end

    function z = cross2d(a, b)
        z = a(1) * b(2) - a(2) * b(1);
    end
    
    function [best_c, best_s, best_a] = evaluate_and_save(sol, best_c, best_s, best_a, total_free, iter, ls, t_start, area_base, area_weight)
        if isempty(sol), return; end
        sum_loads = sum(sol(:, 5));
        if sum_loads <= 0, return; end
        
        sum_price = sum(sol(:, 6));
        used_cells = sum(sol(:, 7));
        pct_area = used_cells / total_free; % Fracción entre 0.0 y 1.0
        
        % FÓRMULA DE COSTE A MINIMIZAR
        current_cost = (sum_price / sum_loads) ^ (area_base - (pct_area * area_weight));
        
        % Si el coste es estrictamente MENOR que el récord actual, lo guardamos
        if current_cost < best_c
            impr = best_c - current_cost; 
            best_c = current_cost;
            best_s = sol(:, 1:4); 
            best_a = pct_area * 100;
            num_racks = size(sol, 1);
            
            fase_str = "Const."; 
            if ls > 0
                fase_str = sprintf("LS %d", ls); 
            end
            
            if isinf(impr)
                impr_str = "Inicial"; 
            else
                impr_str = sprintf("-%.2f", impr); 
            end
            
            fprintf('\nIter %d (%s): ¡Récord! Coste: %.4f (%s) | Área: %.2f%% | Racks: %d | %.1fs', ...
                    iter, fase_str, best_c, impr_str, best_a, num_racks, toc(t_start));
        end
    end
    
    function plot_warehouse_solution(warehouse_coords, obstacles, bays, solution, best_area_pct, best_cost)
        figure('Name', 'Visualización de Coste Optimizado', 'Color', 'w');
        hold on; axis equal;
        
        plot(polyshape(warehouse_coords(:,1), warehouse_coords(:,2)), 'FaceColor', [0.98 0.98 0.98]);
        
        cmap = lines(max(bays(:,1)) + 1);
        for s = 1:size(solution, 1)
            id = solution(s, 1); bx = solution(s, 2); by = solution(s, 3); rot = solution(s, 4);
            b_idx = find(bays(:,1) == id, 1);
            bw = bays(b_idx, 2); bd = bays(b_idx, 3);
            if rot == 90, t = bw; bw = bd; bd = t; end
            
            rectangle('Position', [bx, by, bw, bd], 'FaceColor', cmap(id+1,:), 'EdgeColor', 'k');
            text(bx+bw/2, by+bd/2, num2str(id), 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', 'k');
        end
        
        if ~isempty(obstacles)
            for i = 1:size(obstacles, 1)
                rectangle('Position', [obstacles(i,1), obstacles(i,2), obstacles(i,3), obstacles(i,4)], ...
                          'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'r');
            end
        end
        title(sprintf('Acomodo Optimizado - Coste: %.2f | Área: %.2f%% | Racks: %d', best_cost, best_area_pct, size(solution, 1)));
        grid on;
    end
