function WarehouseOptimizerGUI()
    % --- 1. CONFIGURACIÓN DE LA VENTANA PRINCIPAL ---
    fig = uifigure('Name', 'Optimizador de Almacén - Carga de Datos', ...
                   'Position', [500, 300, 450, 350], ...
                   'Color', [0.95 0.95 0.95]);

    % Estructura para almacenar las matrices temporalmente
    appData = struct('warehouse', [], 'obstacles', [], 'ceiling', [], 'bays', []);

    % Título de la aplicación
    uilabel(fig, 'Position', [20, 300, 400, 30], ...
            'Text', 'Configuración del Proyecto', ...
            'FontSize', 18, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center');

    % --- 2. BOTONES Y ETIQUETAS ---
    % 2.1 Almacén (Warehouse)
    btnWH = uibutton(fig, 'Position', [30, 240, 150, 30], 'Text', '1. Cargar Almacén', ...
                     'ButtonPushedFcn', @(btn,event) loadCSV('warehouse', 'warehouse.csv'));
    lblWH = uilabel(fig, 'Position', [200, 240, 220, 30], 'Text', 'Falta archivo...', 'FontColor', [0.8 0 0]);

    % 2.2 Obstáculos (Obstacles)
    btnObs = uibutton(fig, 'Position', [30, 190, 150, 30], 'Text', '2. Cargar Obstáculos', ...
                      'ButtonPushedFcn', @(btn,event) loadCSV('obstacles', 'obstacles.csv'));
    lblObs = uilabel(fig, 'Position', [200, 190, 220, 30], 'Text', 'Falta archivo...', 'FontColor', [0.8 0 0]);

    % 2.3 Techo (Ceiling)
    btnCeil = uibutton(fig, 'Position', [30, 140, 150, 30], 'Text', '3. Cargar Techo', ...
                       'ButtonPushedFcn', @(btn,event) loadCSV('ceiling', 'ceiling.csv'));
    lblCeil = uilabel(fig, 'Position', [200, 140, 220, 30], 'Text', 'Falta archivo...', 'FontColor', [0.8 0 0]);

    % 2.4 Bahías (Types of Bays)
    btnBays = uibutton(fig, 'Position', [30, 90, 150, 30], 'Text', '4. Cargar Bahías', ...
                       'ButtonPushedFcn', @(btn,event) loadCSV('bays', 'types_of_bays.csv'));
    lblBays = uilabel(fig, 'Position', [200, 90, 220, 30], 'Text', 'Falta archivo...', 'FontColor', [0.8 0 0]);

    % --- 3. BOTÓN DE EJECUCIÓN ---
    btnRun = uibutton(fig, 'Position', [125, 20, 200, 40], ...
                      'Text', 'INICIAR OPTIMIZACIÓN', ...
                      'FontSize', 14, 'FontWeight', 'bold', ...
                      'BackgroundColor', [0 0.45 0.74], 'FontColor', 'w', ...
                      'ButtonPushedFcn', @(btn,event) runAlgorithm());

    % --- 4. FUNCIONES CALLBACK ---
    
    % Función genérica para abrir el explorador y cargar el CSV
    function loadCSV(dataType, expectedName)
        % Mapeo de la etiqueta correspondiente según el tipo
        switch dataType
            case 'warehouse', currentLabel = lblWH;
            case 'obstacles', currentLabel = lblObs;
            case 'ceiling',   currentLabel = lblCeil;
            case 'bays',      currentLabel = lblBays;
        end
        
        % Abrir explorador de archivos
        [file, path] = uigetfile('*.csv', ['Selecciona el archivo: ', expectedName]);
        
        % Si el usuario cancela (cierra la ventana)
        if isequal(file, 0)
            return; 
        end
        
        % Leer el archivo
        try
            fullPath = fullfile(path, file);
            appData.(dataType) = readmatrix(fullPath); % Guardar en memoria
            
            % Actualizar interfaz (Feedback visual en verde)
            currentLabel.Text = ['Cargado: ', file];
            currentLabel.FontColor = [0 0.6 0]; % Verde oscuro
            currentLabel.FontWeight = 'bold';
            
            % Pequeño aviso si el nombre no coincide con el esperado (opcional)
            if ~strcmpi(file, expectedName)
                uialert(fig, ['Has cargado "', file, '". Asegúrate de que contiene el formato correcto para ', expectedName, '.'], 'Aviso de nombre', 'Icon', 'warning');
            end
        catch
            uialert(fig, 'Error al leer el archivo. Asegúrate de que es un CSV válido sin texto mezclado.', 'Error de Lectura', 'Icon', 'error');
        end
    end

    % Función que se ejecuta al darle al botón azul final
    function runAlgorithm()
        % 1. Comprobar que los archivos obligatorios están cargados
        if isempty(appData.warehouse) || isempty(appData.ceiling) || isempty(appData.bays)
            uialert(fig, 'Faltan datos por cargar (Almacén, Techo o Bahías son obligatorios).', 'Faltan datos', 'Icon', 'warning');
            return;
        end
        
        % Si los obstáculos están vacíos, creamos una matriz vacía segura
        if isempty(appData.obstacles)
            appData.obstacles = zeros(0, 4);
        end
        
        % 2. Feedback de que el algoritmo va a empezar
        btnRun.Text = 'CALCULANDO...';
        btnRun.BackgroundColor = [0.85 0.33 0.10]; % Naranja
        drawnow; % Forzar a MATLAB a repintar la ventana inmediatamente
        
       [solucion, coste, area] = BayPlacementOptimized(appData.warehouse, appData.obstacles, appData.ceiling, appData.bays);
                                  visualizar_almacen_3D(appData.warehouse, appData.obstacles, appData.bays, appData.ceiling, solucion);
    end
end