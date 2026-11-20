%% load_urdf.m
% Load dan visualisasi robot dari file URDF (termasuk mesh STL/DAE)
% Membutuhkan: Robotics System Toolbox

clear; clc; close all;

%% 1. Pilih file URDF
urdfFile = 'robot.urdf';   % ganti dengan nama/path file URDF kamu

if ~isfile(urdfFile)
    [f, p] = uigetfile('*.urdf', 'Pilih file URDF');
    if isequal(f, 0)
        error('Tidak ada file URDF yang dipilih.');
    end
    urdfFile = fullfile(p, f);
end
urdfFile = char(java.io.File(urdfFile).getCanonicalPath());
urdfDir  = fileparts(urdfFile);

%% 2. Daftar mesh yang direferensikan URDF
txt  = fileread(urdfFile);
tok  = regexp(txt, 'filename\s*=\s*"([^"]+)"', 'tokens');
meshRefs = unique(cellfun(@(c) c{1}, tok, 'UniformOutput', false));

fprintf('Mesh yang direferensikan URDF: %d file\n', numel(meshRefs));
for i = 1:numel(meshRefs)
    fprintf('  - %s\n', meshRefs{i});
end

%% 3. Kandidat folder mesh (dicoba SATU per SATU)
% Untuk "package://arm/meshes/x.STL", folder yang benar biasanya
% induk dari folder "arm". Tambahkan path manual di sini bila perlu.
cands = { ...
    fullfile(urdfDir, '..', '..'), ...   % induk dari folder paket "arm"
    fullfile(urdfDir, '..'), ...         % folder paket "arm"
    urdfDir, ...
    fullfile(urdfDir, 'meshes'), ...
    fullfile(urdfDir, '..', 'meshes')};
cands = cellfun(@(c) char(java.io.File(c).getCanonicalPath()), cands, ...
                'UniformOutput', false);
cands = unique(cands(cellfun(@isfolder, cands)), 'stable');

%% 4. Load URDF: coba tiap folder, pilih yang mesh-nya ter-load
robot = [];  bestVis = -1;  usedPath = '';
for k = 1:numel(cands)
    try
        r = importrobot(urdfFile, 'MeshPath', cands{k});
    catch ME
        fprintf('Gagal dengan path %s: %s\n', cands{k}, ME.message);
        continue
    end
    nv = countVisuals(r);
    fprintf('MeshPath = %s  ->  visual ter-load: %d\n', cands{k}, nv);
    if nv > bestVis
        robot = r;  bestVis = nv;  usedPath = cands{k};
    end
    if nv > 0, break; end
end

% Fallback: load tanpa MeshPath
if isempty(robot)
    robot = importrobot(urdfFile);
    usedPath = '(tanpa MeshPath)';
end
robot.DataFormat = 'row';

fprintf('\nRobot berhasil di-load: %s\n', urdfFile);
fprintf('MeshPath dipakai : %s\n', usedPath);
fprintf('Jumlah body      : %d\n', robot.NumBodies);
fprintf('Base             : %s\n', robot.BaseName);
fprintf('DOF (non-fixed)  : %d\n', numel(homeConfiguration(robot)));
showdetails(robot);

if bestVis == 0 && ~isempty(meshRefs)
    warning(['Mesh tidak ter-load. Pastikan file STL ada, lalu tambahkan ', ...
             'folder induk paket "arm" ke daftar cands di bagian 3.']);
end

%% 5. Tampilkan robot dengan mesh
q = homeConfiguration(robot);

figure('Name', 'Visualisasi URDF', 'Color', 'w');
show(robot, q, 'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off');
title('Model Robot dari URDF');
axis equal; grid on; view(135, 25);
camlight('headlight'); lighting gouraud;

%% 6. (Opsional) GUI slider joint
% gui = interactiveRigidBodyTree(robot, 'MeshPath', usedPath);

%% Fungsi bantu
function n = countVisuals(r)
    n = 0;
    try
        bodies = [{r.Base}, r.Bodies];
        for i = 1:numel(bodies)
            n = n + numel(bodies{i}.Visuals);
        end
    catch
        n = 1;   % versi MATLAB tanpa properti Visuals: anggap berhasil
    end
end