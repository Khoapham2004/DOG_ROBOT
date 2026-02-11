function DogLeg_Display_Corner()
    % ---------------------------------------------------------
    % MÔ PHỎNG CHÂN CHÓ - HIỂN THỊ THÔNG SỐ GÓC TRÊN TRÁI
    % - Slider chỉnh quỹ đạo: bo cung, width, height, center X/Z
    % - Nút in góc: 80 / 40 chẵn / 40 lẻ (format C array)
    % - THÊM: nút Print 20 (từ 40) -> lấy 20 giá trị từ 40 frame swing
    % - Khớp vai: so với trục Z+
    % - Khớp cẳng: GÓC TRONG tại gối (0..180, thẳng=180)
    % - Nút Direction: CW / CCW (đi tiến / đi lùi)
    %   => In ra cũng theo chiều đang chọn
    % Nhấn 'q' để thoát
    % ---------------------------------------------------------
    clc; close all;

    %% 1) CẤU HÌNH CƠ CẤU
    cfg.L_Thigh = 130;
    cfg.L_Shank = 130;

    %% 2) THAM SỐ QUỸ ĐẠO (slider điều khiển)
    params.N_Points = 40;           % 40 swing + 40 stance = 80
    params.L_step   = 160;          % độ dài bước
    params.H_step   = 40;           % chiều cao cung swing
    params.Z_ground = -220;         % baseline ground
    params.curvePow = 1.0;          % bo cung: z = sin(pi*t)^curvePow
    params.centerX  = 0;            % dịch trái/phải
    params.centerZ  = 0;            % dịch lên/xuống

    % Quỹ đạo (update động)
    Full_X = [];
    Full_Z = [];
    Total_Points = 0;

    %% 3) FIGURE / AXES
    f = figure('Name', 'SIMULATION DOG LEG', 'Color', 'w', ...
               'Position', [50 50 1050 650], ...
               'KeyPressFcn', @KeyPressCallback);

    ax = axes('Parent', f);
    axis(ax, 'equal'); grid(ax, 'on'); hold(ax, 'on');
    xlim(ax, [-300 250]); ylim(ax, [-350 80]);
    xlabel(ax, 'X [mm]'); ylabel(ax, 'Z [mm]');

    % --- handles môi trường (update được) ---
    h_Ground = yline(ax, params.Z_ground, 'k--', 'LineWidth', 1);
    h_Path   = plot(ax, 0, 0, 'Color', [0.8 0.8 0.8], 'LineWidth', 2);

    % --- ROBOT ---
    h_Thigh  = plot(ax, [0 0], [0 0], 'r-', 'LineWidth', 4);
    h_Shank  = plot(ax, [0 0], [0 0], 'k-', 'LineWidth', 4);
    h_Joints = plot(ax, 0, 0, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 6);

    % --- TEXT HIỂN THỊ CỐ ĐỊNH (GÓC TRÊN TRÁI) ---
    h_TextInfo = text(ax, -285, 60, '', ...
                      'FontSize', 11, ...
                      'FontName', 'Arial', ...
                      'Interpreter', 'none', ...
                      'BackgroundColor', [1 1 1 0.85], ...
                      'EdgeColor', 'k', ...
                      'Margin', 5);

    %% 4) NÚT IN GÓC + NÚT ĐI LÙI
    uicontrol('Parent', f, 'Style', 'pushbutton', ...
              'String', 'Print 80 frames', ...
              'Position', [880 600 150 30], ...
              'FontSize', 10, ...
              'Callback', @Print80Callback);

    uicontrol('Parent', f, 'Style', 'pushbutton', ...
              'String', 'Print 40 even', ...
              'Position', [880 560 150 30], ...
              'FontSize', 10, ...
              'Callback', @PrintEvenCallback);

    uicontrol('Parent', f, 'Style', 'pushbutton', ...
              'String', 'Print 40 odd', ...
              'Position', [880 520 150 30], ...
              'FontSize', 10, ...
              'Callback', @PrintOddCallback);

    % THÊM NÚT: Print 20 từ 40 (lấy đều 1,3,5,...,39 trong SWING 1..40)
    uicontrol('Parent', f, 'Style', 'pushbutton', ...
              'String', 'Print 20 (from 40)', ...
              'Position', [880 480 150 30], ...
              'FontSize', 10, ...
              'Callback', @Print20From40Callback);

    % Direction toggle: CW / CCW
    dirSign = +1; % +1: tiến, -1: lùi
    uicontrol('Parent', f, 'Style', 'togglebutton', ...
              'String', 'Direction: CW', ...
              'Position', [880 440 150 30], ...
              'FontSize', 10, ...
              'Value', 0, ...
              'Callback', @ToggleDirectionCallback);

    %% 5) SLIDERS (đánh vào quỹ đạo)
    baseX = 860; baseY = 60; w = 180; h = 18; gap = 45;

    makeLabel(baseX, baseY+4*gap+20, 'TRAJECTORY SLIDERS', 10, true);

    makeLabel(baseX, baseY+4*gap, 'Bo cung (Curve)', 9, false);
    sCurve = uicontrol('Parent', f, 'Style', 'slider', ...
        'Min', 0.25, 'Max', 3.0, 'Value', params.curvePow, ...
        'Position', [baseX baseY+4*gap-18 w h], ...
        'Callback', @AnySliderChanged);
    tCurve = makeValueText(baseX+w-5, baseY+4*gap, params.curvePow);

    makeLabel(baseX, baseY+3*gap, 'Do dai/rong (L_step)', 9, false);
    sWidth = uicontrol('Parent', f, 'Style', 'slider', ...
        'Min', 60, 'Max', 260, 'Value', params.L_step, ...
        'Position', [baseX baseY+3*gap-18 w h], ...
        'Callback', @AnySliderChanged);
    tWidth = makeValueText(baseX+w-5, baseY+3*gap, params.L_step);

    makeLabel(baseX, baseY+2*gap, 'Chieu cao cung (H_step)', 9, false);
    sHeight = uicontrol('Parent', f, 'Style', 'slider', ...
        'Min', 0, 'Max', 140, 'Value', params.H_step, ...
        'Position', [baseX baseY+2*gap-18 w h], ...
        'Callback', @AnySliderChanged);
    tHeight = makeValueText(baseX+w-5, baseY+2*gap, params.H_step);

    makeLabel(baseX, baseY+1*gap, 'Tam cung X (Center X)', 9, false);
    sCX = uicontrol('Parent', f, 'Style', 'slider', ...
        'Min', -180, 'Max', 180, 'Value', params.centerX, ...
        'Position', [baseX baseY+1*gap-18 w h], ...
        'Callback', @AnySliderChanged);
    tCX = makeValueText(baseX+w-5, baseY+1*gap, params.centerX);

    makeLabel(baseX, baseY+0*gap, 'Tam cung Z (Center Z)', 9, false);
    sCZ = uicontrol('Parent', f, 'Style', 'slider', ...
        'Min', -180, 'Max', 180, 'Value', params.centerZ, ...
        'Position', [baseX baseY+0*gap-18 w h], ...
        'Callback', @AnySliderChanged);
    tCZ = makeValueText(baseX+w-5, baseY+0*gap, params.centerZ);

    %% 6) TÍNH QUỸ ĐẠO BAN ĐẦU
    RecomputeTrajectoryAndRedraw();

    %% 7) LOOP ANIMATION
    global keep_running; keep_running = true;

    while keep_running
        idxRun = GetRunIndex(Total_Points); % theo dirSign hiện tại

        for i = idxRun
            if ~keep_running, break; end

            Tx = Full_X(i);  Tz = Full_Z(i);

            % 1) GIẢI IK
            [Knee_X, Knee_Z, is_valid] = Solve_DogLeg_Knee(Tx, Tz, cfg);

            if is_valid
                Hip_X = 0; Hip_Z = 0;

                % 2) TÍNH GÓC
                [th2_deg, th3_inner_deg] = ComputeAngles(Hip_X, Hip_Z, Knee_X, Knee_Z, Tx, Tz);

                % 3) UPDATE DRAW
                set(h_Thigh,  'XData', [Hip_X,  Knee_X], 'YData', [Hip_Z,  Knee_Z]);
                set(h_Shank,  'XData', [Knee_X, Tx],     'YData', [Knee_Z, Tz]);
                set(h_Joints, 'XData', [Hip_X, Knee_X, Tx], 'YData', [Hip_Z, Knee_Z, Tz]);

                info_str = {
                    ['Frame: ' num2str(i) '/' num2str(Total_Points)];
                    ['------------------'];
                    ['Khớp vai (vs Z+): ' sprintf('%.1f', th2_deg) '°'];
                    ['Khớp cẳng (IN):   ' sprintf('%.1f', th3_inner_deg) '°'];
                    ['------------------'];
                    ['Dir: ' ternary(dirSign>0,'CW','CCW')];
                    ['Curve: ' sprintf('%.2f', params.curvePow) ...
                     ' | L: ' sprintf('%.0f', params.L_step) ...
                     ' | H: ' sprintf('%.0f', params.H_step)];
                    ['CenterX: ' sprintf('%.0f', params.centerX) ...
                     ' | CenterZ: ' sprintf('%.0f', params.centerZ)];
                };
                set(h_TextInfo, 'String', info_str);
            end

            drawnow;
            pause(0.02);
        end
    end

    if isvalid(f), close(f); end

    % =========================================================
    % =================== CALLBACKS ============================
    % =========================================================
    function ToggleDirectionCallback(src, ~)
        if get(src,'Value') == 1
            dirSign = -1;
            set(src,'String','Direction: CCW');
        else
            dirSign = +1;
            set(src,'String','Direction: CW');
        end
    end

    function AnySliderChanged(~, ~)
        params.curvePow = get(sCurve, 'Value');
        params.L_step   = get(sWidth, 'Value');
        params.H_step   = get(sHeight, 'Value');
        params.centerX  = get(sCX, 'Value');
        params.centerZ  = get(sCZ, 'Value');

        set(tCurve, 'String', sprintf('%.2f', params.curvePow));
        set(tWidth, 'String', sprintf('%.0f', params.L_step));
        set(tHeight,'String', sprintf('%.0f', params.H_step));
        set(tCX,    'String', sprintf('%.0f', params.centerX));
        set(tCZ,    'String', sprintf('%.0f', params.centerZ));

        RecomputeTrajectoryAndRedraw();
    end

    function RecomputeTrajectoryAndRedraw()
        N = params.N_Points;
        t = linspace(0, 1, N);

        Zg = params.Z_ground + params.centerZ;

        P1 = [-params.L_step/2 + params.centerX, Zg];
        P3 = [ params.L_step/2 + params.centerX, Zg];

        Swing_X = (1-t)*P1(1) + t*P3(1);
        s = max(0, sin(pi*t));
        Swing_Z = Zg + params.H_step * (s.^params.curvePow);

        Stance_X = (1-t)*P3(1) + t*P1(1);
        Stance_Z = ones(1, N) * Zg;

        Full_X = [Swing_X, Stance_X];
        Full_Z = [Swing_Z, Stance_Z];
        Total_Points = length(Full_X);

        set(h_Path, 'XData', Full_X, 'YData', Full_Z);
        set(h_Ground, 'Value', Zg);
    end

    % ---- IN THEO CHIỀU HIỆN TẠI ----
    function Print80Callback(~, ~)
        [hipDeg, kneeDeg, validArr] = ComputeAllAngles();
        if ~any(validArr)
            fprintf('\n[Print 80] Không có frame hợp lệ!\n');
            return;
        end
        idxList = GetRunIndex(Total_Points);
        fprintf('\n==== PRINT 80 FRAMES (%s) ====\n', ternary(dirSign>0,'CW','CCW'));
        PrintAngleArray(hipDeg, kneeDeg, idxList);
    end

    function PrintEvenCallback(~, ~)
        [hipDeg, kneeDeg, validArr] = ComputeAllAngles();
        if ~any(validArr)
            fprintf('\n[Print even] Không có frame hợp lệ!\n');
            return;
        end
        if dirSign > 0
            idxList = 2:2:Total_Points;           % CW
        else
            idxList = Total_Points:-2:2;          % CCW: 80,78,...,2
        end
        if ~any(validArr(idxList))
            fprintf('\n[Print even] Không có frame chẵn hợp lệ!\n');
            return;
        end
        fprintf('\n==== PRINT 40 EVEN FRAMES (%s) ====\n', ternary(dirSign>0,'CW','CCW'));
        PrintAngleArray(hipDeg, kneeDeg, idxList);
    end

    function PrintOddCallback(~, ~)
        [hipDeg, kneeDeg, validArr] = ComputeAllAngles();
        if ~any(validArr)
            fprintf('\n[Print odd] Không có frame hợp lệ!\n');
            return;
        end
        if dirSign > 0
            idxList = 1:2:(Total_Points-1);       % CW: 1..79
        else
            idxList = (Total_Points-1):-2:1;      % CCW: 79,77,...,1
        end
        if ~any(validArr(idxList))
            fprintf('\n[Print odd] Không có frame lẻ hợp lệ!\n');
            return;
        end
        fprintf('\n==== PRINT 40 ODD FRAMES (%s) ====\n', ternary(dirSign>0,'CW','CCW'));
        PrintAngleArray(hipDeg, kneeDeg, idxList);
    end

    % ====== THÊM: PRINT 20 GIÁ TRỊ TỪ 40 (SWING) ======
    % Mặc định: lấy 20 điểm đều trong swing 1..40: 1,3,5,...,39
    % Và đảo ngược theo Direction (CCW in 39,37,...,1)
    function Print20From40Callback(~, ~)
        [hipDeg, kneeDeg, validArr] = ComputeAllAngles();
        if ~any(validArr)
            fprintf('\n[Print 20] Không có frame hợp lệ!\n');
            return;
        end

        if Total_Points < 40
            fprintf('\n[Print 20] Total_Points < 40 (đang là %d)!\n', Total_Points);
            return;
        end

        baseIdx = 1:40;           % 40 frame swing
        pick20  = baseIdx(1:2:end); % 1,3,5,...,39 (20 giá trị)

        if dirSign > 0
            idxList = pick20;              % CW
        else
            idxList = fliplr(pick20);      % CCW
        end

        if ~any(validArr(idxList))
            fprintf('\n[Print 20] 20 frame chọn ra không hợp lệ!\n');
            return;
        end

        fprintf('\n==== PRINT 20 (FROM 40 SWING) (%s) ====\n', ternary(dirSign>0,'CW','CCW'));
        PrintAngleArray(hipDeg, kneeDeg, idxList);
    end

    function idx = GetRunIndex(N)
        if dirSign > 0
            idx = 1:N;
        else
            idx = N:-1:1;
        end
    end

    function [hipDeg, kneeDeg, validArr] = ComputeAllAngles()
        hipDeg   = nan(1, Total_Points);
        kneeDeg  = nan(1, Total_Points);
        validArr = false(1, Total_Points);

        Hip_X = 0; Hip_Z = 0;

        for k = 1:Total_Points
            Tx = Full_X(k);  Tz = Full_Z(k);
            [Knee_X, Knee_Z, is_valid] = Solve_DogLeg_Knee(Tx, Tz, cfg);
            if ~is_valid, continue; end

            [th2, th3] = ComputeAngles(Hip_X, Hip_Z, Knee_X, Knee_Z, Tx, Tz);

            hipDeg(k)   = th2;
            kneeDeg(k)  = th3;
            validArr(k) = true;
        end
    end

    function PrintAngleArray(hipDeg, kneeDeg, idxList)
        for k = idxList
            if isnan(hipDeg(k)) || isnan(kneeDeg(k))
                continue;
            end
            fprintf('{ %7.1ff, %8.1ff}, // %d\n', hipDeg(k), kneeDeg(k), k);
        end
    end

    % ---- GÓC (vai vs Z+), (gối = góc trong) ----
    function [hip_deg, knee_inner_deg] = ComputeAngles(Hip_X, Hip_Z, Knee_X, Knee_Z, Tx, Tz)
        vec_thigh_x = Knee_X - Hip_X;
        vec_thigh_z = Knee_Z - Hip_Z;
        hip_deg = rad2deg(atan2(vec_thigh_x, vec_thigh_z));
        hip_deg = Wrap180(hip_deg);

        v1 = [Hip_X - Knee_X, Hip_Z - Knee_Z]; % gối -> hông
        v2 = [Tx    - Knee_X, Tz    - Knee_Z]; % gối -> bàn chân

        n1 = hypot(v1(1), v1(2));
        n2 = hypot(v2(1), v2(2));
        if n1 < 1e-9 || n2 < 1e-9
            knee_inner_deg = 0;
            return;
        end

        c = (v1(1)*v2(1) + v1(2)*v2(2)) / (n1*n2);
        c = max(-1, min(1, c));
        knee_inner_deg = rad2deg(acos(c));
    end

    % ==================== UI HELPERS ==========================
    function makeLabel(x, y, str, fs, bold)
        if bold, fw = 'bold'; else, fw = 'normal'; end
        uicontrol('Parent', f, 'Style', 'text', ...
            'String', str, 'Position', [x y 190 18], ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'left', ...
            'FontSize', fs, 'FontWeight', fw);
    end

    function hTxt = makeValueText(xRight, y, val)
        hTxt = uicontrol('Parent', f, 'Style', 'text', ...
            'String', sprintf('%.2f', val), ...
            'Position', [xRight-55 y 55 18], ...
            'BackgroundColor', 'w', 'HorizontalAlignment', 'right', ...
            'FontSize', 9);
    end

    function s = ternary(cond, a, b)
        if cond, s = a; else, s = b; end
    end

end

% --- HÀM IK ---
function [Kx, Kz, valid] = Solve_DogLeg_Knee(Tx, Tz, cfg)
    L1 = cfg.L_Thigh;
    L2 = cfg.L_Shank;
    r = sqrt(Tx^2 + Tz^2);

    if r > (L1 + L2)
        ratio = L1 / r;
        Kx = Tx * ratio;
        Kz = Tz * ratio;
        valid = true;
        return;
    end

    if r < 1e-9
        Kx = 0; Kz = -L1;
        valid = true;
        return;
    end

    valid = true;

    cos_alpha = (L1^2 + r^2 - L2^2) / (2*L1*r);
    cos_alpha = max(-1, min(1, cos_alpha));
    alpha = acos(cos_alpha);

    base_angle = atan2(Tz, Tx);

    theta_1 = base_angle + alpha;
    theta_2 = base_angle - alpha;

    kx1 = L1 * cos(theta_1); kz1 = L1 * sin(theta_1);
    kx2 = L1 * cos(theta_2); kz2 = L1 * sin(theta_2);

    if kx1 < kx2
        Kx = kx1; Kz = kz1;
    else
        Kx = kx2; Kz = kz2;
    end
end

function KeyPressCallback(~, event)
    global keep_running;
    if strcmp(event.Key, 'q')
        keep_running = false;
    end
end

function a = Wrap180(a)
    while a > 180, a = a - 360; end
    while a < -180, a = a + 360; end
end
