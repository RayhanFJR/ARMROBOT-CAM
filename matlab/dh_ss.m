%% urdf_dh_ss.m
% Dari URDF -> parameter DH, matriks transformasi (numerik & simbolik),
% dan model state space (linearisasi) di MATLAB.
% Butuh: Robotics System Toolbox
% Opsional: Symbolic Math Toolbox (matriks simbolik), Control System Toolbox (objek ss)
%
% Catatan: parameter DH tidak unik. Script ini memakai konvensi DH standar:
%   A_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)

clear; clc; close all;

%% 0. Pengaturan
urdfFile = 'robot.urdf';          % nama/path file URDF
meshPath = '';                    % opsional: SATU folder mesh (boleh kosong, mesh tidak perlu utk perhitungan)
eeName   = '';                    % nama body end-effector; kosong = body paling ujung otomatis
gravity  = [0 0 -9.81];           % vektor gravitasi pada frame base
qOp      = [];                    % titik operasi linearisasi (1 x DOF); kosong = semua nol

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
robot.Gravity    = gravity;

%% 2. Cari rantai base -> end-effector dan joint yang bergerak
names    = robot.BodyNames;
baseName = robot.BaseName;

parentOf = containers.Map();
cfgIdx   = containers.Map();      % indeks joint di vektor konfigurasi
nq = 0;
for i = 1:robot.NumBodies
    b = robot.Bodies{i};
    parentOf(b.Name) = b.Parent.Name;
    if ~strcmp(b.Joint.Type, 'fixed')
        nq = nq + 1;
        cfgIdx(b.Name) = nq;
    end
end

if isempty(eeName)
    isLeaf = ~ismember(names, values(parentOf));
    leaves = names(isLeaf);
    eeName = leaves{end};
    if numel(leaves) > 1
        warning('Ada %d ujung rantai, dipakai: %s (set eeName untuk memilih).', numel(leaves), eeName);
    end
end

chain = {eeName};
while ~strcmp(parentOf(chain{1}), baseName)
    chain = [parentOf(chain{1}), chain]; %#ok<AGROW>
end

q0 = homeConfiguration(robot);
mov = struct('name', {}, 'type', {}, 'P', {}, 'z', {}, 'qidx', {});
for k = 1:numel(chain)
    b   = getBody(robot, chain{k});
    jnt = b.Joint;
    if strcmp(jnt.Type, 'fixed'), continue; end
    pName = parentOf(chain{k});
    if strcmp(pName, baseName)
        Tpar = eye(4);
    else
        Tpar = getTransform(robot, q0, pName);
    end
    Tj = Tpar * jnt.JointToParentTransform;           % frame joint pada base
    ax = jnt.JointAxis(:); ax = ax / norm(ax);
    mov(end+1) = struct('name', chain{k}, 'type', jnt.Type, ...
                        'P', Tj(1:3,4), 'z', Tj(1:3,1:3)*ax, ...
                        'qidx', cfgIdx(chain{k})); %#ok<SAGROW>
end
n = numel(mov);
if n == 0, error('Tidak ada joint bergerak pada rantai.'); end
if n ~= nq
    warning('Rantai punya %d joint, robot punya %d DOF (ada cabang). DH hanya untuk rantai ke %s.', n, nq, eeName);
end
isRev = arrayfun(@(m) ~strcmp(m.type, 'prismatic'), mov);

Tee0 = getTransform(robot, q0, eeName);               % pose ee saat q = 0

%% 3. Bangun frame DH F0..Fn (dalam koordinat base)
F = cell(1, n+1);
z1 = mov(1).z;
x0 = [1;0;0] - ([1;0;0]'*z1)*z1;
if norm(x0) < 1e-6, x0 = [0;1;0] - ([0;1;0]'*z1)*z1; end
F{1} = makeFrame(x0/norm(x0), z1, mov(1).P);

for i = 1:n
    Op = F{i}(1:3,4);  zp = F{i}(1:3,3);  xp = F{i}(1:3,1);
    if i < n
        Pn = mov(i+1).P;  zn = mov(i+1).z;
    else
        Pn = Tee0(1:3,4); zn = zp;                    % frame terakhir: z sejajar joint n
    end
    cr = cross(zp, zn);
    if norm(cr) > 1e-6                                 % sumbu tidak sejajar
        w0 = Op - Pn;
        a_ = dot(zp,zp); b_ = dot(zp,zn); c_ = dot(zn,zn);
        d_ = dot(zp,w0); e_ = dot(zn,w0);
        den = a_*c_ - b_^2;
        s = (b_*e_ - c_*d_)/den;
        t = (a_*e_ - b_*d_)/den;
        foot1 = Op + s*zp;
        On = Pn + t*zn;
        v = On - foot1;
        if norm(v) > 1e-9, x = v/norm(v); else, x = cr/norm(cr); end
    else                                               % sumbu sejajar / berimpit
        t  = dot(Op - Pn, zn);
        On = Pn + t*zn;
        v  = On - Op;
        if norm(v) > 1e-9
            x = v/norm(v);
        else
            x = xp - dot(xp,zn)*zn;  x = x/norm(x);
        end
    end
    F{i+1} = makeFrame(x, zn, On);
end
Ttool = F{n+1} \ Tee0;                                 % transform tetap Fn -> end-effector

%% 4. Ekstrak parameter DH
a = zeros(1,n); alpha = a; d = a; th = a;
for i = 1:n
    T = F{i} \ F{i+1};
    alpha(i) = atan2(T(3,2), T(3,3));
    th(i)    = atan2(T(2,1), T(1,1));
    d(i)     = T(3,4);
    a(i)     = T(1,4)*cos(th(i)) + T(2,4)*sin(th(i));
end
cl = @(x) round(x, 9) + 0;                             % bersihkan error numerik (+0 hilangkan -0)
a = cl(a); alpha = cl(alpha); d = cl(d); th = cl(th);

typ = repmat({'R'}, n, 1);  typ(~isRev) = {'P'};
DH = table((1:n)', {mov.name}', typ, a(:), rad2deg(alpha(:)), d(:), rad2deg(th(:)), ...
    'VariableNames', {'Joint','Body','Tipe','a_m','alpha_deg','d_m','theta0_deg'});
fprintf('=== Parameter DH (konfigurasi q = 0) ===\n');
fprintf('Revolute : theta_i = theta0_i + q_i\nPrismatic: d_i = d0_i + q_i\n\n');
disp(DH);

%% 5. Matriks transformasi numerik + verifikasi terhadap URDF
fkDH = @(qv) fkFromDH(F{1}, a, alpha, d, th, isRev, qv, Ttool);

qt = (rand(1, nq) - 0.5) * pi/2;
qd_chain = qt([mov.qidx]);
Tref = getTransform(robot, qt, eeName);
Tdh  = fkDH(qd_chain);
fprintf('Verifikasi FK (DH vs URDF), error Frobenius = %.3e\n', norm(Tdh - Tref, 'fro'));
if norm(Tdh - Tref, 'fro') > 1e-6
    warning('Error besar: cek jenis joint (mis. joint dengan limit/mimic) atau urutan rantai.');
end
fprintf('\nT_base_F0 (offset base ke frame DH 0):\n'); disp(cl(F{1}));
fprintf('T_Fn_ee (offset tool dari frame DH n ke end-effector):\n'); disp(cl(Ttool));

%% 6. Matriks transformasi simbolik (butuh Symbolic Math Toolbox)
if license('test', 'symbolic_toolbox') && ~isempty(ver('symbolic'))
    qs = sym('q', [1 n], 'real');
    T0n = sym(eye(4));
    fprintf('\n=== Matriks transformasi simbolik A_i (frame i-1 ke i) ===\n');
    for i = 1:n
        ai = symLen(a(i)); al = symAng(alpha(i));
        di = symLen(d(i)); ti = symAng(th(i));
        if isRev(i), ti = ti + qs(i); else, di = di + qs(i); end
        Ai = [cos(ti), -sin(ti)*cos(al),  sin(ti)*sin(al), ai*cos(ti);
              sin(ti),  cos(ti)*cos(al), -cos(ti)*sin(al), ai*sin(ti);
              0,        sin(al),          cos(al),         di;
              0,        0,                0,               1];
        Ai = simplify(Ai);
        fprintf('A%d =\n', i); disp(Ai);
        T0n = T0n * Ai;
    end
    T0n = simplify(T0n);
    Tbase_ee = simplify(symLen(F{1}) * T0n * symLen(Ttool));
    fprintf('T_0n (frame DH 0 ke n) =\n'); disp(T0n);
    fprintf('T_base_ee (base URDF ke end-effector) =\n'); disp(Tbase_ee);
    fprintf('Posisi end-effector p(q) =\n'); disp(Tbase_ee(1:3,4));
    fkFun = matlabFunction(Tbase_ee, 'Vars', {qs});    % fkFun([q1 ... qn]) -> 4x4 numerik
else
    fprintf('\n(Symbolic Math Toolbox tidak ada: matriks simbolik dilewati, pakai fkDH(q) untuk numerik.)\n');
end

%% 7. State space (linearisasi)
% Model: M(q)*qdd + C(q,qd)*qd + G(q) = tau,   state x = [q; qd],  input u = tau
if isempty(qOp), qOp = zeros(1, nq); end
qdOp  = zeros(1, nq);
tauOp = gravityTorque(robot, qOp);                     % torsi penahan gravitasi -> titik setimbang
M = massMatrix(robot, qOp);
if rank(M) < nq || min(eig((M+M')/2)) < 1e-9
    warning('Matriks massa singular: data <inertial> di URDF kemungkinan belum lengkap.');
end

dyn = @(q, qd, tau) forwardDynamics(robot, q, qd, tau);   % -> qdd (baris)
h = 1e-6;
dQ = zeros(nq);  dQd = zeros(nq);
for j = 1:nq
    e = zeros(1, nq); e(j) = h;
    dQ(:,j)  = ((dyn(qOp+e, qdOp, tauOp) - dyn(qOp-e, qdOp, tauOp)) / (2*h))';
    dQd(:,j) = ((dyn(qOp, qdOp+e, tauOp) - dyn(qOp, qdOp-e, tauOp)) / (2*h))';
end

A = [zeros(nq), eye(nq); dQ, dQd];
B = [zeros(nq); inv(M)];
C = eye(2*nq);
D = zeros(2*nq, nq);
A(abs(A) < 1e-8) = 0;  B(abs(B) < 1e-10) = 0;

fprintf('\n=== State space: dx = A x + B u,  x = [q; qdot],  u = tau ===\n');
fprintf('Titik operasi q = '); disp(qOp);
fprintf('tau setimbang (gravitasi) = '); disp(tauOp);
fprintf('M(q_op) =\n'); disp(M);
fprintf('A =\n'); disp(A);
fprintf('B =\n'); disp(B);

% Model nonlinear: dx = fx(x,u)   (x, u kolom)
fx = @(x, u) [x(nq+1:end); dyn(x(1:nq).', x(nq+1:end).', u(:).').'];

% Sifat sistem
p = eig(A);
fprintf('Pole sistem (eig A) =\n'); disp(p);

% --- Cek stabilitas (lokal, di sekitar titik operasi qOp) ---
tol   = 1e-6;
maxRe = max(real(p));
fprintf('\n=== Cek stabilitas ===\n');
fprintf('Re(pole) terbesar = %.4g\n', maxRe);
if maxRe > tol
    fprintf('Hasil: TIDAK STABIL (%d pole di setengah bidang kanan).\n', sum(real(p) > tol));
elseif maxRe < -tol
    fprintf('Hasil: STABIL ASIMTOTIK (semua pole di setengah bidang kiri).\n');
else
    % Pole di sumbu imajiner: stabil marginal hanya jika tidak ada blok Jordan
    imp = p(abs(real(p)) <= tol);
    checked = [];  jordanOK = true;
    for k = 1:numel(imp)
        lk = imp(k);
        if any(abs(checked - lk) < 1e-5), continue; end
        checked(end+1) = lk; %#ok<AGROW>
        alg = sum(abs(p - lk) < 1e-5);                         % multiplisitas aljabar
        geo = size(A,1) - rank(A - lk*eye(size(A)), 1e-6);     % multiplisitas geometri
        if geo < alg, jordanOK = false; end
    end
    if jordanOK
        fprintf('Hasil: STABIL MARGINAL (pole di sumbu imajiner, berosilasi terus karena tanpa redaman).\n');
    else
        fprintf('Hasil: TIDAK STABIL (pole berulang di sumbu imajiner dengan blok Jordan, respons drift).\n');
    end
end
fprintf('Catatan: ini stabilitas LOKAL dari model linear di sekitar qOp.\n\n');

figure('Name', 'Peta pole', 'Color', 'w');
plot(real(p), imag(p), 'rx', 'MarkerSize', 10, 'LineWidth', 2); hold on;
xline(0, 'k--'); yline(0, 'k--'); grid on;
xlabel('Real'); ylabel('Imajiner'); title('Pole sistem (eig A)');

Cm = B;  Ak = eye(2*nq);
for k = 1:2*nq-1, Ak = Ak*A; Cm = [Cm, Ak*B]; end %#ok<AGROW>
fprintf('Rank matriks controllability = %d dari %d\n', rank(Cm), 2*nq);

if exist('ss', 'file')
    sn = [compose('q%d', 1:nq), compose('dq%d', 1:nq)];
    sys = ss(A, B, C, D, 'StateName', sn, 'InputName', compose('tau%d', 1:nq), 'OutputName', sn);
end

%% 8. Stabilisasi: state feedback  u = tau_op - K (x - x_op)
% Bobot LQR: makin besar Q -> respons makin cepat; makin besar R -> torsi makin hemat.
Qw = diag([100*ones(1,nq), 10*ones(1,nq)]);    % bobot state [q; qdot]
Rw = 0.1*eye(nq);                              % bobot input (torsi)

if exist('lqr', 'file')
    K = lqr(A, B, Qw, Rw);                     % Control System Toolbox
else
    K = lqrHam(A, B, Qw, Rw);                  % cadangan tanpa toolbox
end
Acl = A - B*K;
pcl = eig(Acl);

fprintf('\n=== Stabilisasi (LQR) ===\n');
fprintf('Gain K =\n'); disp(K);
fprintf('Pole closed-loop (eig(A-BK)) =\n'); disp(pcl);
if max(real(pcl)) < 0
    fprintf('Hasil: closed-loop STABIL ASIMTOTIK (Re terbesar = %.4g).\n', max(real(pcl)));
else
    warning('Closed-loop masih belum stabil, periksa controllability dan bobot Q/R.');
end

% Alternatif pole placement (opsional, butuh Control System Toolbox):
% pDes = -2*(1:2*nq);               % pole yang diinginkan (harus berbeda & bagian real < 0)
% K = place(A, B, pDes);

% Simulasi nonlinear dengan kontroler: mulai dari simpangan dari titik operasi
xOp    = [qOp(:); qdOp(:)];
dq0    = 0.05;                                 % simpangan awal (rad atau m), mulai kecil dulu
x0     = xOp + [dq0*ones(nq,1); zeros(nq,1)];
tauMax = Inf;                                  % batas torsi aktuator, mis. 5; Inf = tanpa batas
ctrl   = @(x) min(max(tauOp(:) - K*(x - xOp), -tauMax), tauMax);
Tsim   = 3;  dt = 0.005;                       % RK4 langkah tetap (lebih cepat & terkontrol)
tl     = 0:dt:Tsim;
N      = numel(tl);

% (a) Simulasi LINEAR closed-loop (instan, sebagai pembanding)
xl = zeros(2*nq, N);
Ex = expm(Acl*dt);
dx = x0 - xOp;
for k = 1:N
    xl(:,k) = xOp + dx;
    dx = Ex*dx;
end

% (b) Simulasi NONLINEAR (forwardDynamics) dengan RK4
xn = nan(2*nq, N);  xn(:,1) = x0;
F_ = @(x) fx(x, ctrl(x));
tl_n = tl;  tic;
for k = 1:N-1
    x  = xn(:,k);
    k1 = F_(x);
    k2 = F_(x + dt/2*k1);
    k3 = F_(x + dt/2*k2);
    k4 = F_(x + dt*k3);
    xn(:,k+1) = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    if any(~isfinite(xn(:,k+1))) || max(abs(xn(1:nq,k+1) - qOp(:))) > 5
        warning('Simulasi nonlinear divergen di t = %.3f s (simpangan awal terlalu besar atau torsi terbatas).', tl(k+1));
        xn = xn(:,1:k+1);  tl_n = tl(1:k+1);
        break
    end
    if mod(k, 100) == 0
        fprintf('  sim t = %.2f / %.1f s  (komputasi %.1f s)\n', tl(k), Tsim, toc);
    end
end

figure('Name', 'Simulasi closed-loop', 'Color', 'w');
subplot(2,1,1);
plot(tl, xl(1:nq,:)', 'LineWidth', 1.5); grid on;
ylabel('q'); title('Linear: respons posisi joint (A - BK)');
legend(compose('q%d', 1:nq), 'Location', 'best');
subplot(2,1,2);
plot(tl_n, xn(1:nq,:)', 'LineWidth', 1.5); grid on;
xlabel('t (s)'); ylabel('q'); title('Nonlinear: respons posisi joint (forwardDynamics)');

figure('Name', 'Torsi', 'Color', 'w');
U = zeros(nq, numel(tl_n));
for k = 1:numel(tl_n), U(:,k) = ctrl(xn(:,k)); end
plot(tl_n, U', 'LineWidth', 1.5); grid on;
xlabel('t (s)'); ylabel('\tau'); title('Torsi kontrol (cek apakah masuk akal untuk aktuator)');

%% 9. Simpan hasil
save('hasil_urdf.mat', 'DH', 'F', 'Ttool', 'A', 'B', 'C', 'D', 'qOp', 'tauOp', 'M', 'K', 'Acl');
fprintf('\nHasil disimpan ke hasil_urdf.mat\n');

%% ===== Fungsi bantu =====
function K = lqrHam(A, B, Q, R)
    % LQR via matriks Hamiltonian (cadangan jika fungsi lqr tidak tersedia)
    nA = size(A, 1);
    H  = [A, -B*(R\B'); -Q, -A'];
    [V, D] = eig(H);
    idx = real(diag(D)) < 0;                   % ambil eigenvector stabil
    V1 = V(1:nA, idx);  V2 = V(nA+1:end, idx);
    P  = real(V2 / V1);
    K  = R \ (B' * P);
end

function Fm = makeFrame(x, z, O)
    y  = cross(z, x);
    Fm = [x, y, z, O; 0 0 0 1];
end

function A = dhA(a, al, d, th)
    A = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
         sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
         0,        sin(al),          cos(al),         d;
         0,        0,                0,               1];
end

function T = fkFromDH(F0, a, alpha, d, th, isRev, qv, Ttool)
    T = F0;
    for i = 1:numel(a)
        if isRev(i)
            T = T * dhA(a(i), alpha(i), d(i), th(i) + qv(i));
        else
            T = T * dhA(a(i), alpha(i), d(i) + qv(i), th(i));
        end
    end
    T = T * Ttool;
end

function s = symLen(x)
    x(abs(x) < 1e-9) = 0;
    s = sym(round(x, 6), 'd');
end

function s = symAng(x)
    r = x / (pi/2);
    if abs(r - round(r)) < 1e-8
        s = round(r) * sym(pi) / 2;
    else
        s = sym(round(x, 6), 'd');
    end
end