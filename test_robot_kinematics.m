function mechanism_with_A_triangle_GH_drives_B()
    % =====================================================
    % THÔNG SỐ CƠ CẤU (mm)
    % =====================================================
    LAB = 130;   % AB cố định
    LBC = 130;   % BC cố định
    LBD = 37;    % BD cố định
    LED = 115;   % ED cố định

    % Tam giác tại A (cứng, quay quanh A)
    LAE = 43;      % AE cố định
    LAF = 43;      % AF cố định
    LEF = 67.07;   % EF cố định

    % A cố định
    A = [0 0];

    % =====================================================
    % G cố định (offset từ A)
    % =====================================================
    dGx = -20.18;
    dGy = -30.0;

    % =====================================================
    % Thanh GH và HF
    % =====================================================
    LGH = 29.5;
    LHF = 40.0;

    % Góc đỉnh tại A của tam giác AEF
    R = LAE;
    if LEF > 2*R
        error('Sai kích thước: EF không thể lớn hơn 2*AE khi AE=AF.');
    end
    phi = 2*asin(LEF/(2*R));   % rad

    % =====================================================
    % UI
    % =====================================================
    f = figure('Name','GH drives Beta at B (IK closed-loop)', ...
               'NumberTitle','off','Color','w','Position',[100 100 950 720]);
    ax = axes('Parent', f, 'Position',[0.05 0.18 0.92 0.77]);
    axis(ax,'equal'); grid(ax,'on'); hold(ax,'on');

    % =====================================================
       % Slider BAO: 0° là hướng lên (+Y), GIỚI HẠN -200 -> -90
    uicontrol('Style','text','String','Góc BAO (θ) - 0° Up (+Y), chạy [-200..-90]', ...
        'Units','normalized','Position',[0.08 0.11 0.40 0.03],'BackgroundColor','w');
    
    sBAO = uicontrol('Style','slider', ...
        'Min',-200,'Max',-90,'Value',-150, ...
        'Units','normalized','Position',[0.08 0.08 0.35 0.035]);


    % =====================================================
    % Slider GH: ép HGO (đọc theo +Y) chạy 0..180
    % Quan hệ: HGO = (gamma_std - 90). Để HGO ∈ [0..180] => gamma_std ∈ [90..270]
    % =====================================================
    uicontrol('Style','text','String','Góc HGO - 0° Up (+Y), chạy [0..180]', ...
        'Units','normalized','Position',[0.50 0.11 0.40 0.03],'BackgroundColor','w');
    sGH = uicontrol('Style','slider','Min',90,'Max',270,'Value',90, ...
        'Units','normalized','Position',[0.50 0.08 0.35 0.035]);

    % Checkbox lock nhánh
    chkLock = uicontrol('Style','checkbox','String','LOCK nhánh (chống nhảy)', ...
        'Value',1,'Units','normalized','BackgroundColor','w', ...
        'Position',[0.08 0.03 0.25 0.04]);

    txt = uicontrol('Style','text','Units','normalized', ...
        'Position',[0.02 0.95 0.92 0.04], ...
        'BackgroundColor','w','HorizontalAlignment','left','FontSize',11);

    % SỬ DỤNG 3 DÒNG NÀY TRÊN MÁY HOÀNG
    sBAO.Callback = @(~,~) redraw();
    sGH.Callback  = @(~,~) redraw();
    chkLock.Callback = @(~,~) redraw();

    % SỬ DỤNG 3 DÒNG NÀY TRÊN MÁY KHOA
    % sBAO.Callback = @(,) redraw();
    % sGH.Callback  = @(,) redraw();
    % chkLock.Callback = @(,) redraw();

    % =====================================================
    % STATE chống nhảy nghiệm
    % =====================================================
    F_prev = [];
    E_prev = [];
    D_prev = [];
    lockF_sign = [];
    lockE_sign = [];
    lockD_sign = [];

    redraw();

    % =====================================================
    % HÀM VẼ / IK
    % =====================================================
    function redraw()
        cla(ax); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');

        % =========================
        % BAO: slider đang là hệ +Y (0° Up), giới hạn [-180..0]
        % nội bộ đổi sang chuẩn +X để dùng cos/sin
        % =========================
        baoUpDeg = sBAO.Value;          % đọc theo +Y
        theta = deg2rad(baoUpDeg + 90); % đổi sang góc từ +X

        % =========================
        % GH: tính H theo hệ +X (gamma_std)
        % nhưng HGO hiển thị theo +Y: HGO = gamma_std - 90
        % =========================
        gammaStdDeg = sGH.Value;        % đo từ +X
        gamma = deg2rad(gammaStdDeg);   % dùng để tính H
        hgoDeg = gammaStdDeg - 90;      % đọc theo +Y (0° Up) -> [0..180]

        useLock = (chkLock.Value == 1);

        % ---- Tính B từ theta (AB cố định) ----
        B = A + LAB * [cos(theta), sin(theta)];

        % ---- G cố định ----
        G = A + [dGx, dGy];

        % ---- H theo góc GH (tính từ +X) ----
        H = G + LGH * [cos(gamma), sin(gamma)];

        % =====================================================
        % BƯỚC 1: Tìm F = giao (A,AF) và (H,HF)
        % =====================================================
        [F1, F2, okF] = circleCircleIntersect(A, LAF, H, LHF);
        if ~okF
            title(ax,'LỖI: Không tìm thấy F (A-F và H-F không giao)','Color','r');
            drawKnown();
            drawCircleFallback(ax,A,LAF,[0.2 0.2 0.2]);
            drawCircleFallback(ax,H,LHF,[0.8 0.8 0]);
            return;
        end
        F = selectSolution(F1, F2, F_prev, A, (H-A), lockF_sign, useLock);
        if useLock
            lockF_sign = sign2(cross2((H-A), (F-A)));
            if lockF_sign==0, lockF_sign=1; end
        end
        F_prev = F;

        % =====================================================
        % BƯỚC 2: Tìm E từ tam giác cứng AEF (biết F)
        % =====================================================
        angF = atan2(F(2)-A(2), F(1)-A(1));
        E1 = A + LAE * [cos(angF + phi), sin(angF + phi)];
        E2 = A + LAE * [cos(angF - phi), sin(angF - phi)];

        E = selectSolution(E1, E2, E_prev, A, (F-A), lockE_sign, useLock);
        if useLock
            lockE_sign = sign2(cross2((F-A), (E-A)));
            if lockE_sign==0, lockE_sign=1; end
        end
        E_prev = E;

        % =====================================================
        % BƯỚC 3: Tìm D = giao (E,ED) và (B,BD)
        % =====================================================
        [D1, D2, okD] = circleCircleIntersect(E, LED, B, LBD);
        if ~okD
            title(ax,'LỖI: Không tìm thấy D (ED và BD không giao)','Color','r');
            drawKnown();
            drawTriangleAEF(A,E,F);
            drawCircleFallback(ax,E,LED,[0 1 0]);
            drawCircleFallback(ax,B,LBD,[1 0 1]);
            return;
        end
        D = selectSolution(D1, D2, D_prev, B, (E-B), lockD_sign, useLock);
        if useLock
            lockD_sign = sign2(cross2((E-B), (D-B)));
            if lockD_sign==0, lockD_sign=1; end
        end
        D_prev = D;

        % =====================================================
        % BƯỚC 4: Từ B và D suy ra hướng BC, tính C, và tính Beta
        % =====================================================
        uBC = (B - D) / norm(B - D);
        C = B + LBC * uBC;

        angBC = atan2(uBC(2), uBC(1));
        beta = wrapToPiLocal(angBC - theta - pi);
        betaDeg = rad2deg(beta);

        % =====================================================
        % VẼ
        % =====================================================
        drawKnown();

        plot(ax, [A(1) B(1)], [A(2) B(2)], 'b-', 'LineWidth', 3);                 % AB
        plot(ax, [B(1) C(1)], [B(2) C(2)], 'k-', 'LineWidth', 5);                 % BC
        plot(ax, [B(1) D(1)], [B(2) D(2)], 'm-', 'LineWidth', 4);                 % BD
        plot(ax, [E(1) D(1)], [E(2) D(2)], 'g-', 'LineWidth', 3);                 % ED
        plot(ax, [G(1) H(1)], [G(2) H(2)], 'c-', 'LineWidth', 3);                 % GH
        plot(ax, [H(1) F(1)], [H(2) F(2)], '-', 'LineWidth', 3, 'Color',[0.8 0.8 0]); % HF

        drawTriangleAEF(A,E,F);

        % ========= Points =========
        % C: gắn thêm tọa độ
        plot(ax, C(1), C(2), 'ko','MarkerFaceColor','w','MarkerSize',6);
        text(ax, C(1)+3, C(2)+3, sprintf('C (%.1f , %.1f)', C(2), C(1)), ...
            'FontWeight','bold','BackgroundColor','w','Margin',1);

        % Các điểm còn lại
        drawPt(B,'B'); drawPt(D,'D');
        drawPt(E,'E'); drawPt(F,'F'); drawPt(H,'H');

        % =====================================================
        % VẼ CUNG GÓC + HIỂN THỊ SỐ ĐỘ
        % =====================================================
        vRefY = [0 1];        % trục +Y
        rA = 22; rG = 16; rB = 26;

        % BAO tại A: từ +Y -> AB
        drawAngleArc(ax, A, vRefY, (B-A), rA, [1 0 0], sprintf('%.1f°', baoUpDeg));

        % HGO tại G: từ +Y -> GH (G->H)
        drawAngleArc(ax, G, vRefY, (H-G), rG, [1 0 0], sprintf('%.1f°', hgoDeg));

        % Beta tại B: giữa BA và BC
        drawAngleArc(ax, B, (A-B), (C-B), rB, [1 0 0], sprintf('%.2f°', betaDeg));

        % =====================================================
        % Text info
        % =====================================================
        degSym = char(176);
        set(txt,'String',sprintf('θ(BAO)=%.1f%s ([-200..-90])   ∠HGO=%.1f%s ([0..180])   =>  Beta@B = %.2f%s', ...
             baoUpDeg, degSym, hgoDeg, degSym, betaDeg, degSym));

        title(ax,'GH slider drives Beta at B (IK đóng vòng)');

        allP = [A;B;C;D;E;F;G;H];
        pad = 60;
        xlim(ax,[min(allP(:,1))-pad, max(allP(:,1))+pad]);
        ylim(ax,[min(allP(:,2))-pad, max(allP(:,2))+pad]);
    end

    % =====================================================
    % Vẽ phần cố định / khung tham chiếu
    % =====================================================
    function drawKnown()
        G = A + [dGx, dGy];
        P = [G(1), A(2)];
        Q = [A(1), G(2)];
        plot(ax, [A(1) P(1) G(1) Q(1) A(1)], [A(2) P(2) G(2) Q(2) A(2)], ...
            'k--', 'Color',[0.7 0.7 0.7]);

        plot(ax, A(1),A(2),'ko','MarkerFaceColor','k','MarkerSize',10);
        text(ax,A(1),A(2)+5,'  A','FontWeight','bold');

        plot(ax, G(1),G(2),'ko','MarkerFaceColor','k','MarkerSize',8);
        text(ax,G(1),G(2)-5,'  G','FontWeight','bold');
    end

    function drawPt(P, name)
        plot(ax, P(1),P(2), 'ko','MarkerFaceColor','w','MarkerSize',6);
        text(ax, P(1)+3, P(2)+3, name);
    end

    % =====================================================
    % Helper: chọn nghiệm (lock theo dấu cross hoặc bám prev)
    % =====================================================
    function P = selectSolution(P1, P2, P_prev, origin, refVec, lockSign, useLock)
        s1 = sign2(cross2(refVec, (P1 - origin)));
        s2 = sign2(cross2(refVec, (P2 - origin)));

        if useLock && ~isempty(lockSign)
            if s1 == lockSign
                P = P1; return;
            elseif s2 == lockSign
                P = P2; return;
            end
        end
        P = pickClosest(P1, P2, P_prev);
    end

    function P = pickClosest(P1, P2, P_prev)
        if isempty(P_prev)
            if P1(2) >= P2(2), P = P1; else, P = P2; end
        else
            if norm(P1 - P_prev) <= norm(P2 - P_prev), P = P1; else, P = P2; end
        end
    end

    function drawTriangleAEF(Ap, Ep, Fp)
        patch(ax, [Ap(1) Ep(1) Fp(1)], [Ap(2) Ep(2) Fp(2)], ...
            'm','FaceAlpha',0.1,'EdgeColor','k','LineWidth',2);
    end

    % =====================================================
    % Circle-circle intersection
    % =====================================================
    function [P1, P2, ok] = circleCircleIntersect(O1, r1, O2, r2)
        ok = false; P1 = [NaN NaN]; P2 = [NaN NaN];
        d = norm(O2 - O1);
        if d < 1e-9, return; end
        if d > (r1 + r2) + 1e-6, return; end
        if d < abs(r1 - r2) - 1e-6, return; end

        a = (r1^2 - r2^2 + d^2) / (2*d);
        h2 = r1^2 - a^2;
        if h2 < -1e-9, return; end
        h = sqrt(max(0, h2));

        u = (O2 - O1) / d;
        Pm = O1 + a * u;
        perp = [-u(2), u(1)];
        P1 = Pm + h * perp;
        P2 = Pm - h * perp;
        ok = true;
    end

    function drawCircleFallback(axh, O, r, col)
        t = linspace(0, 2*pi, 200);
        x = O(1) + r*cos(t);
        y = O(2) + r*sin(t);
        plot(axh, x, y, '--', 'Color', col, 'LineWidth', 1);
    end

    function a = wrapToPiLocal(a)
        a = mod(a + pi, 2*pi) - pi;
    end

    function z = cross2(a,b), z = a(1)*b(2) - a(2)*b(1); end
    function s = sign2(x)
        eps0 = 1e-9;
        if x > eps0, s = 1;
        elseif x < -eps0, s = -1;
        else, s = 0;
        end
    end

    % =====================================================
    % Vẽ cung góc từ vector v1 -> v2 (chiều ngắn nhất) + label
    % =====================================================
    function drawAngleArc(axh, O, v1, v2, r, col, labelStr)
        if norm(v1) < 1e-12 || norm(v2) < 1e-12
            return;
        end
        u1 = v1 / norm(v1);
        u2 = v2 / norm(v2);

        a1 = atan2(u1(2), u1(1));
        a2 = atan2(u2(2), u2(1));

        da = mod(a2 - a1 + pi, 2*pi) - pi;  % [-pi, pi]

        n = 60;
        ang = a1 + linspace(0, da, n);

        x = O(1) + r*cos(ang);
        y = O(2) + r*sin(ang);
        plot(axh, x, y, '-', 'LineWidth', 2, 'Color', col);

        amid = a1 + da/2;
        rt = r * 1.15;
        tx = O(1) + rt*cos(amid);
        ty = O(2) + rt*sin(amid);

        text(axh, tx, ty, labelStr, 'Color', col, 'FontWeight','bold', ...
            'HorizontalAlignment','center', 'VerticalAlignment','middle', ...
            'BackgroundColor','w', 'Margin', 1);
    end
end
