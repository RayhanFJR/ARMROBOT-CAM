%% urdf_ik.m
% Inverse kinematics (IK) robot dari URDF di MATLAB.
% Metode:
%   (1) inverseKinematics bawaan toolbox (BFGS + random restart) -> semua solusi yang ditemukan
%   (2) Damped Least Squares (DLS) berbasis Jacobian buatan sendiri (untuk pemahaman/pembanding)
%   (3) Tracking lintasan garis lurus (opsional) dengan warm-start
% Butuh: Robotics System Toolbox

clear; clc; close all;

%% 0. Pengaturan
urdfFile = 'robot.urdf';          % nama/path file URDF
meshPath = '';                    % opsional: SATU folder mesh (kosong = tanpa MeshPath)
eeName   = '';                    % body end-effector; kosong = body paling ujung otomatis

% Target pose end-effector
useManual = false;                % false = target dibuat dari konfigurasi acak (pasti terjangkau)
xyzTarget = [0.30 0.10 0.20];     % posisi target (m) pada frame base, dipakai jika useManual = true
eulTarget = [0 0 0];              % orientasi target [yaw pitch roll] rad (urutan ZYX)

nTry     = 30;                    % jumlah tebakan awal acak untuk mencari banyak solusi
tolPose  = 1e-4;                  % toleransi error pose
doTraj   = true;                  % true = tracking lintasan garis lurus dari home ke target
nWp      = 30;                    % jumlah waypoint lintasan

%% 1. Load URDF
if ~isfile(urdfFile)
    [f, p] = uigetfile('*.urdf', 'Pilih file URDF');
    if isequal(f, 0), error('Tidak ada file URDF yang dipilih.'); end
    urdfFile = fullfile(p, f);
end
if isempty(meshPath)
    robot = importrobot(urdfFile);
else
    robot = importrobot(urdfFile, 'MeshPath', meshPath);
end
robot.DataFormat = 'row';

names = robot.BodyNames;
if isempty(eeName)
    par = cellfun(@(b) robot.Bodies{find(strcmp(names, b), 1)}.Parent.Name, names, 'UniformOutput', false);
    leaves = names(~ismember(names, par));
    eeName = leaves{end};
    if numel(leaves) > 1
        warning('Ada %d ujung rantai, dipakai: %s (set eeName untuk memilih).', numel(leaves), eeName);
    end
end

qHome = homeConfiguration(robot);
nq    = numel(qHome);

% Batas joint (urutan sama dengan vektor konfigurasi)
qmin = zeros(1, nq); qmax = zeros(1, nq); c = 0;
for i = 1:robot.NumBodies
    jn = robot.Bodies{i}.Joint;
    if ~strcmp(jn.Type, 'fixed')
        c = c + 1;
        qmin(c) = jn.PositionLimits(1);
        qmax(c) = jn.PositionLimits(2);
    end
end
qmin(~isfinite(qmin)) = -pi;  qmax(~isfinite(qmax)) = pi;

fprintf('Robot: %d DOF, end-effector: %s\n', nq, eeName);

%% 2. Tentukan target pose
if useManual
    Ttarget = trvec2tform(xyzTarget) * eul2tform(eulTarget);   % ZYX
else
    rng(1);
    qGoal   = randomConfiguration(robot);
    Ttarget = getTransform(robot, qGoal, eeName);
    fprintf('Target dibuat dari konfigurasi acak (pasti terjangkau).\n');
end
fprintf('Posisi target (m): [%.4f %.4f %.4f]\n', Ttarget(1:3,4));

% Bobot [Rx Ry Rz Tx Ty Tz]. Robot < 6 DOF: hanya posisi yang dikejar.
if nq >= 6
    w = [1 1 1 1 1 1];
else
    w = [0 0 0 1 1 1];
    fprintf('DOF < 6 -> IK hanya mengejar POSISI (orientasi diabaikan).\n');
end

%% 3. IK bawaan toolbox: banyak tebakan awal -> kumpulkan solusi berbeda
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 1500;

sols = zeros(0, nq);
for t = 1:nTry
    if t == 1, q0 = qHome; else, q0 = randomConfiguration(robot); end
    [qs, info] = ik(eeName, Ttarget, w, q0);
    if info.PoseErrorNorm < tolPose
        if isempty(sols) || all(vecnorm(sols - qs, 2, 2) > 1e-2)
            sols(end+1, :) = qs; %#ok<SAGROW>
        end
    end
end

if isempty(sols)
    error(['Tidak ada solusi IK (error < %g). Target mungkin di luar jangkauan, ', ...
           'atau coba naikkan nTry / longgarkan tolPose.'], tolPose);
end

% Pilih solusi paling dekat dengan home (gerakan joint terkecil)
[~, ib] = min(vecnorm(sols - qHome, 2, 2));
qIK = sols(ib, :);

fprintf('\n=== IK bawaan toolbox: %d solusi berbeda ditemukan ===\n', size(sols, 1));
disp(array2table(sols, 'VariableNames', compose('q%d', 1:nq)));
fprintf('Solusi terpilih (terdekat dengan home), indeks %d:\n', ib); disp(qIK);

%% 4. Verifikasi dengan forward kinematics
Tchk = getTransform(robot, qIK, eeName);
errPos = norm(Tchk(1:3,4) - Ttarget(1:3,4));
axA    = rotm2axang(Ttarget(1:3,1:3) * Tchk(1:3,1:3)');
errOri = abs(axA(4));                                    % sudut error orientasi (rad)
fprintf('Error posisi = %.3e m, error orientasi = %.3e rad\n', errPos, errOri);

%% 5. IK dengan Damped Least Squares (Jacobian) sebagai pembanding
lambda = 0.05;  maxIt = 500;
[qDLS, okDLS, itDLS] = ikDLS(robot, eeName, Ttarget, qHome, w, lambda, maxIt, tolPose, qmin, qmax);
Td = getTransform(robot, qDLS, eeName);
fprintf('\n=== IK DLS (Jacobian) ===\n');
fprintf('Konvergen: %d, iterasi: %d, error posisi = %.3e m\n', okDLS, itDLS, norm(Td(1:3,4) - Ttarget(1:3,4)));
fprintf('q DLS ='); disp(qDLS);

%% 6. Visualisasi: home -> solusi IK dengan frame target
figure('Name', 'IK', 'Color', 'w');
ax = show(robot, qIK, 'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off');
hold(ax, 'on');
plotTransforms(tform2trvec(Ttarget), tform2quat(Ttarget), 'Parent', ax, 'FrameSize', 0.1);
plot3(ax, Ttarget(1,4), Ttarget(2,4), Ttarget(3,4), 'r*', 'MarkerSize', 10);
title('Solusi IK (frame = target)'); axis equal; grid on; view(135, 25);

%% 7. (Opsional) Tracking lintasan garis lurus dari home ke target
if doTraj
    T0 = getTransform(robot, qHome, eeName);
    p0 = T0(1:3,4);  pT = Ttarget(1:3,4);
    wTraj = [0 0 0 1 1 1];                         % lintasan: hanya posisi
    Q = zeros(nWp, nq);  qk = qHome;  P = zeros(nWp, 3);
    ikT = inverseKinematics('RigidBodyTree', robot);
    for k = 1:nWp
        s  = k / nWp;
        Tk = eye(4);  Tk(1:3,4) = (1-s)*p0 + s*pT;
        qk = ikT(eeName, Tk, wTraj, qk);           % warm-start dari titik sebelumnya
        Q(k,:) = qk;  P(k,:) = Tk(1:3,4)';
    end
    figure('Name', 'Tracking lintasan', 'Color', 'w');
    ax2 = show(robot, Q(1,:), 'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off');
    hold(ax2, 'on'); plot3(ax2, P(:,1), P(:,2), P(:,3), 'r-', 'LineWidth', 2);
    axis(ax2, 'equal'); grid(ax2, 'on'); view(ax2, 135, 25);
    axis(ax2, 'manual');
    for k = 1:nWp
        show(robot, Q(k,:), 'Parent', ax2, 'PreservePlot', false, ...
             'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off');
        drawnow;
    end
    title(ax2, 'Tracking garis lurus (merah) dengan IK');

    figure('Name', 'Joint selama lintasan', 'Color', 'w');
    plot(1:nWp, Q, 'LineWidth', 1.5); grid on;
    xlabel('waypoint'); ylabel('q'); title('Konfigurasi joint sepanjang lintasan');
    legend(compose('q%d', 1:nq), 'Location', 'best');
end

%% 8. Simpan hasil
save('hasil_ik.mat', 'Ttarget', 'sols', 'qIK', 'qDLS', 'eeName');
fprintf('\nHasil disimpan ke hasil_ik.mat\n');

%% ===== Fungsi bantu =====
function [q, ok, it] = ikDLS(robot, eeName, Tt, q, w, lambda, maxIt, tol, qmin, qmax)
    % Iterasi: dq = J_w' * (J_w*J_w' + lambda^2 I)^-1 * (w .* e)
    ok = false;  W = diag(w);
    for it = 1:maxIt
        T  = getTransform(robot, q, eeName);
        e  = poseErr(T, Tt);                      % [angular; linear] pada frame base
        if norm(w(:) .* e) < tol, ok = true; return; end
        J  = geometricJacobian(robot, q, eeName); % 6 x nq, baris: [angular; linear]
        Jw = W * J;
        dq = Jw' / (Jw*Jw' + lambda^2 * eye(6)) * (w(:) .* e);
        q  = min(max(q + dq', qmin), qmax);       % jaga batas joint
    end
end

function e = poseErr(T, Tt)
    elin = Tt(1:3,4) - T(1:3,4);
    ax   = rotm2axang(Tt(1:3,1:3) * T(1:3,1:3)');
    eang = ax(1:3)' * ax(4);
    e    = [eang; elin];
end