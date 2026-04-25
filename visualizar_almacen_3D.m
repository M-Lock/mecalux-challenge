function visualizar_almacen_3D(warehouse_coords, obstacles, bays, ceiling, solucion)
    % 1. Configurar la Figura 3D
    fig = figure('Name', 'Simulación 3D del Almacén', 'Color', 'w', 'Position', [100, 100, 900, 700]);
    hold on; grid on; axis equal;
    view(3); % Activar la vista en 3 dimensiones
    
    camlight('headlight');
    lighting gouraud;

    % 2. Dibujar el suelo del Almacén (Z = 0)
    patch('XData', warehouse_coords(:,1), 'YData', warehouse_coords(:,2), ...
          'ZData', zeros(size(warehouse_coords(:,1))), ...
          'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'k', 'FaceAlpha', 0.5);

    % 3. Dibujar los Obstáculos en Rojo
    if ~isempty(obstacles)
        for i = 1:size(obstacles, 1)
            ox = obstacles(i,1); oy = obstacles(i,2);
            ow = obstacles(i,3); od = obstacles(i,4);
            % Dibujamos el obstáculo como un bloque de altura máxima (ej. 6000mm)
            dibujar_caja_3d(ox, oy, 0, ow, od, 6000, [0.8 0.2 0.2]); 
        end
    end

    % 4. Dibujar los Racks (Estanterías)
    if ~isempty(solucion)
        cmap = lines(max(bays(:,1)) + 1); 

        for i = 1:size(solucion, 1)
            id = solucion(i, 1);
            bx = solucion(i, 2);
            by = solucion(i, 3);
            rot = solucion(i, 4);
            
            % Buscar las dimensiones del rack en el catálogo de bays
            b_idx = find(bays(:,1) == id, 1);
            
            % Protección de seguridad por si el ID no existe
            if isempty(b_idx)
                continue; 
            end
            
            bw = bays(b_idx, 2); % Ancho
            bd = bays(b_idx, 3); % Profundidad
            bh = bays(b_idx, 4); % Altura real de la estantería
            
            % Aplicar rotación
            if rot == 90
                temp = bw; bw = bd; bd = temp;
            end
            
            color = cmap(id+1, :);
            
            % Llamar a la función auxiliar para dibujar la caja
            dibujar_caja_3d(bx, by, 0, bw, bd, bh, color);
        end
    end

    % 5. Dibujar el Techo (Ceiling) transparente
    if ~isempty(ceiling)
        min_x = min(warehouse_coords(:,1)); max_x = max(warehouse_coords(:,1));
        min_y = min(warehouse_coords(:,2)); max_y = max(warehouse_coords(:,2));
        
        [Xq, Yq] = meshgrid(linspace(min_x, max_x, 50), linspace(min_y, max_y, 50));
        Zq = interp1(ceiling(:,1), ceiling(:,2), Xq, 'previous', 'extrap');
        
        surf(Xq, Yq, Zq, 'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
    end

    % 6. Ajustes finales de la cámara
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm) - Altura');
    title(sprintf('Visualización 3D - Total Racks: %d', size(solucion, 1)));
    hold off;
end

% =========================================================================
% Función auxiliar para construir un bloque 3D
% =========================================================================
function dibujar_caja_3d(x, y, z, w, d, h, color)
    V = [x, y, z; x+w, y, z; x+w, y+d, z; x, y+d, z; x, y, z+h; x+w, y, z+h; x+w, y+d, z+h; x, y+d, z+h];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', V, 'Faces', F, 'FaceColor', color, 'EdgeColor', [0.2 0.2 0.2], 'FaceAlpha', 0.9);
end