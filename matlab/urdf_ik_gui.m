function urdf_ik_gui()
% urdf_ik_gui  GUI interaktif IK: geser slider X Y Z, robot dari URDF mengikuti.
% Butuh: Robotics System Toolbox. Tidak butuh toolbox lain.
%
% Fitur:
%   - Slider + kotak angka untuk X, Y, Z (meter, frame base)
%   - IK real-time (warm start dari pose sebelumnya, jadi gerakan mulus)
%   - Indikator tercapai / di luar jangkauan, error posisi
%   - Opsi kunci orientasi (hanya aktif jika robot >= 6 DOF)
%   - Tombol Reset, Acak (target pasti terjangkau), Salin q (derajat)

%% ===== Pengaturan =====
urdfFile = 'robot.urdf';      % nama/path URDF
meshPath = '';                % opsional: SATU folder mesh; kosong = dicari otomatis
eeName   = '';                % kosong = ujung rantai otomatis
tolPos   = 1e-3;              % toleransi error posisi (m)
tolOri   = 1e-2;              % toleransi error orientasi (rad), jika dikunci

%% ===== Load URDF =====
if ~isfile(urdfFile)
    [f, p] = uigetfile('*.urdf', 'Pilih file URDF');
    if isequal(f, 0), return; end
    urdfFile = fullfile(p, f);
end
urdfFile = char(java.io.File(urdfFile).getCanonicalPath());
urdfDir  = fileparts(urdfFile);

if isempty(meshPath)
    cands = {fullfile(urdfDir,'..','..'), fullfile(urdfDir,'..'), urdfDir, ...
             fullfile(urdfDir,'meshes'), fullfile(urdfDir,'..','meshes')};
    cands = cellfun(@(c) char(java.io.File(c).getCanonicalPath()), cands, 'UniformOutput', false);
    cands = unique(cands(cellfun(@isfolder, cands)), 'stable');
else
    cands = {meshPath};
end
robot = [];  best = -1;
for k = 1:numel(cands)
    try
        r = importrobot(urdfFile, 'MeshPath', cands{k});
    catch
        continue
    end
    nv = countVisuals(r);
    if nv > best, robot = r; best = nv; end
    if nv > 0, break; end
end
if isempty(robot), robot = importrobot(urdfFile); end
robot.DataFormat = 'row';

if isempty(eeName), eeName = findEE(robot); end
qHome = homeConfiguration(robot);
nq    = numel(qHome);
T0    = getTransform(robot, qHome, eeName);
R0    = T0(1:3,1:3);
p0    = T0(1:3,4)';

%% ===== Estimasi ruang kerja untuk rentang slider =====
rng(0);
Ns = 1500;  P = zeros(Ns, 3);
for i = 1:Ns
    P(i,:) = tform2trvec(getTransform(robot, randomConfiguration(robot), eeName));
end
lo = min([P; p0; 0 0 0], [], 1) - 0.03;
hi = max([P; p0; 0 0 0], [], 1) + 0.03;

%% ===== State =====
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 300;
qCur = qHome;
busy = false;
hMark = [];
lastOK = true;
canLock = nq >= 6;

%% ===== GUI =====
fig = figure('Name', 'IK Interaktif', 'NumberTitle', 'off', 'Color', 'w', ...
             'Position', [80 60 1150 680]);
ax  = axes('Parent', fig, 'Position', [0.04 0.07 0.60 0.90]);
pnl = uipanel('Parent', fig, 'Position', [0.66 0.05 0.32 0.92], ...
              'Title', 'Target posisi ujung robot (m, frame base)', 'FontSize', 10);

lbl = {'X', 'Y', 'Z'};
ypos = [0.88 0.78 0.68];
hs = gobjects(1,3);  he = gobjects(1,3);
for k = 1:3
    uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', 'Position', [0.03 ypos(k) 0.08 0.06], ...
              'String', lbl{k}, 'FontSize', 13, 'FontWeight', 'bold');
    hs(k) = uicontrol(pnl, 'Style', 'slider', 'Units', 'normalized', ...
              'Position', [0.12 ypos(k)+0.01 0.55 0.05], 'Min', lo(k), 'Max', hi(k), ...
              'Value', min(max(p0(k), lo(k)), hi(k)), 'SliderStep', [0.002 0.05]);
    he(k) = uicontrol(pnl, 'Style', 'edit', 'Units', 'normalized', ...
              'Position', [0.70 ypos(k)+0.005 0.26 0.055], 'FontSize', 11, ...
              'String', sprintf('%.3f', p0(k)));
    set(hs(k), 'Callback', @(~,~) onSlider(k));
    try
        addlistener(hs(k), 'ContinuousValueChange', @(~,~) onSlider(k));
    catch
    end
    set(he(k), 'Callback', @(~,~) onEdit(k));
end

hLock = uicontrol(pnl, 'Style', 'checkbox', 'Units', 'normalized', ...
          'Position', [0.05 0.59 0.90 0.05], 'FontSize', 10, ...
          'String', 'Kunci orientasi (seperti pose home)', 'Value', 0, ...
          'Callback', @(~,~) solveAndDraw());
if ~canLock
    set(hLock, 'Enable', 'off', 'String', 'Kunci orientasi (butuh robot >= 6 DOF)');
end

hStatus = uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', ...
          'Position', [0.03 0.50 0.94 0.07], 'FontSize', 12, 'FontWeight', 'bold', ...
          'String', '');
hErr = uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', ...
          'Position', [0.03 0.44 0.94 0.05], 'FontSize', 10, ...
          'HorizontalAlignment', 'left', 'String', '');
hQ = uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', ...
          'Position', [0.03 0.17 0.94 0.26], 'FontSize', 11, ...
          'HorizontalAlignment', 'left', 'FontName', 'Consolas', 'String', '');

uicontrol(pnl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.03 0.08 0.30 0.07], ...
          'String', 'Reset', 'FontSize', 10, 'Callback', @(~,~) onReset());
uicontrol(pnl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.36 0.08 0.30 0.07], ...
          'String', 'Acak', 'FontSize', 10, 'Callback', @(~,~) onRandom());
uicontrol(pnl, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.69 0.08 0.28 0.07], ...
          'String', 'Salin q', 'FontSize', 10, 'Callback', @(~,~) onCopy());
uicontrol(pnl, 'Style', 'text', 'Units', 'normalized', 'Position', [0.03 0.01 0.94 0.06], ...
          'FontSize', 9, 'ForegroundColor', [0.4 0.4 0.4], 'HorizontalAlignment', 'left', ...
          'String', 'Salin q: sudut joint (derajat) disalin ke clipboard untuk --goal-q di run_experiment.py');

solveAndDraw();

%% ===== Callback =====
    function onSlider(k)
        set(he(k), 'String', sprintf('%.3f', get(hs(k), 'Value')));
        solveAndDraw();
    end

    function onEdit(k)
        v = str2double(get(he(k), 'String'));
        if isnan(v), v = get(hs(k), 'Value'); end
        v = min(max(v, lo(k)), hi(k));
        set(hs(k), 'Value', v);
        set(he(k), 'String', sprintf('%.3f', v));
        solveAndDraw();
    end

    function onReset()
        qCur = qHome;
        setTarget(p0);
        solveAndDraw();
    end

    function onRandom()
        qr = randomConfiguration(robot);
        qCur = qr;
        setTarget(tform2trvec(getTransform(robot, qr, eeName)));
        solveAndDraw();
    end

    function onCopy()
        s = strtrim(sprintf('%.1f ', rad2deg(qCur)));
        try, clipboard('copy', s); catch, end
        fprintf('\nq (derajat) = [%s]\n', s);
        fprintf('python run_experiment.py --dry-run --plot --goal-q %s\n', s);
    end

    function setTarget(p)
        for kk = 1:3
            v = min(max(p(kk), lo(kk)), hi(kk));
            set(hs(kk), 'Value', v);
            set(he(kk), 'String', sprintf('%.3f', v));
        end
    end

%% ===== Inti: IK + gambar =====
    function solveAndDraw()
        if busy, return; end
        busy = true;
        guard = onCleanup(@() setBusyFalse());
        xyz = [get(hs(1),'Value') get(hs(2),'Value') get(hs(3),'Value')];
        lockOri = canLock && get(hLock, 'Value') == 1;

        T = trvec2tform(xyz);
        if lockOri
            T(1:3,1:3) = R0;
            w = [1 1 1 1 1 1];
        else
            w = [0 0 0 1 1 1];
        end

        [qs, okS, eP, eO] = trySolve(T, w, qCur, xyz, lockOri);
        if ~okS && ~isequal(qCur, qHome)          % coba lagi dari home (mungkin cabang solusi lain)
            [qs2, okS2, eP2, eO2] = trySolve(T, w, qHome, xyz, lockOri);
            if okS2, qs = qs2; okS = okS2; eP = eP2; eO = eO2; end
        end

        lastOK = okS;
        if okS, qCur = qs; end

        draw(xyz);
        if okS
            set(hStatus, 'String', 'TERCAPAI', 'ForegroundColor', [0 0.55 0.1]);
        else
            set(hStatus, 'String', 'DI LUAR JANGKAUAN', 'ForegroundColor', [0.8 0 0]);
        end
        if lockOri
            set(hErr, 'String', sprintf('Error posisi: %.2f mm   orientasi: %.2f deg', eP*1000, rad2deg(eO)));
        else
            set(hErr, 'String', sprintf('Error posisi: %.2f mm', eP*1000));
        end
        lines = cell(nq, 1);
        for j = 1:nq
            lines{j} = sprintf('q%d = %8.2f deg', j, rad2deg(qCur(j)));
        end
        set(hQ, 'String', lines);
        drawnow limitrate;
    end

    function setBusyFalse()
        busy = false;
    end

    function [qs, ok, eP, eO] = trySolve(T, w, q0, xyz, lockOri)
        [qs, ~] = ik(eeName, T, w, q0);
        Tr = getTransform(robot, qs, eeName);
        eP = norm(Tr(1:3,4)' - xyz);
        eO = 0;
        if lockOri
            a  = rotm2axang(R0 * Tr(1:3,1:3)');
            eO = abs(a(4));
        end
        ok = eP < tolPos && eO < tolOri;
    end

    function draw(xyz)
        show(robot, qCur, 'Parent', ax, 'PreservePlot', false, ...
             'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off');
        hold(ax, 'on');
        if lastOK, mc = [0 0.7 0.2]; else, mc = [0.9 0 0]; end
        if isempty(hMark) || ~isgraphics(hMark)
            hMark = plot3(ax, xyz(1), xyz(2), xyz(3), 'p', 'MarkerSize', 16, ...
                          'MarkerFaceColor', mc, 'MarkerEdgeColor', 'k');
        else
            set(hMark, 'XData', xyz(1), 'YData', xyz(2), 'ZData', xyz(3), 'MarkerFaceColor', mc);
        end
        xlim(ax, [lo(1) hi(1)]);  ylim(ax, [lo(2) hi(2)]);  zlim(ax, [lo(3) hi(3)]);
        daspect(ax, [1 1 1]);
        grid(ax, 'on');  view(ax, 135, 25);
        xlabel(ax, 'X (m)');  ylabel(ax, 'Y (m)');  zlabel(ax, 'Z (m)');
        title(ax, sprintf('Target: [%.3f  %.3f  %.3f] m', xyz));
    end
end

%% ===== Fungsi bantu =====
function n = countVisuals(r)
    n = 0;
    try
        bodies = [{r.Base}, r.Bodies];
        for i = 1:numel(bodies)
            n = n + numel(bodies{i}.Visuals);
        end
    catch
        n = 1;
    end
end

function ee = findEE(robot)
    names = robot.BodyNames;
    par = cell(size(names));
    for i = 1:numel(names)
        par{i} = getBody(robot, names{i}).Parent.Name;
    end
    leaves = names(~ismember(names, par));
    ee = leaves{end};
    if numel(leaves) > 1
        warning('Ada %d ujung rantai, dipakai: %s (set eeName untuk memilih).', numel(leaves), ee);
    end
end