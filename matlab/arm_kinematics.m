%% KINEMATIKA LENGAN 4 DOF (sesuai arm.urdf)
% Butuh Symbolic Math Toolbox, MATLAB R2016b atau lebih baru.
% Isi: 1) DH  2) T joint space  3) T task space  4) FK  5) IK  6) Uji
% (FK diturunkan sebelum IK karena IK diturunkan dari persamaan FK.)
clear; clc;

syms q1 q2 q3 q4 real
syms x y z phi real
syms bx by d1 a1 L1 L2 L3 d4 e real
syms sigma1 sigma2 real        % pilihan solusi, bernilai +1 atau -1
hp = sym(pi)/2;

% nilai numerik dari URDF (meter)
par = [bx by d1 a1 L1 L2 L3 d4 e];
val = [-0.00354 0.010685 0.09259 0.020575 0.10384 0.14241 0.06449 0.010295 0.00007];

%% 1. PARAMETER DH (standar): A = Rz(theta)*Tz(d)*Tx(a)*Rx(alpha)
theta = [q1+hp;  q2+hp;  -q3-hp;  q4];
d     = [d1;     sym(0); sym(0);  d4];
a     = [a1;     L1;     L2;      L3];
alpha = [hp;     sym(0); sym(0);  sym(0)];

disp('=== 1. TABEL DH ===')
str = @(v) arrayfun(@char, v, 'UniformOutput', false);
DH = table((1:4).', str(theta), str(d), str(a), str(alpha), ...
    'VariableNames', {'i','theta','d','a','alpha'});
disp(DH)

% transformasi tetap: base -> frame 0, dan frame 4 -> end-effector
T_b0 = sym(eye(4));  T_b0(1,4) = bx;  T_b0(2,4) = by;
T_4e = sym([0 0 -1 0; 0 -1 0 0; -1 0 0 0; 0 0 0 1]);  T_4e(2,4) = -e;
disp('T_b0 ='); disp(T_b0)
disp('T_4e ='); disp(T_4e)

%% 2. MATRIKS TRANSFORMASI JOINT SPACE
disp('=== 2. TRANSFORMASI JOINT SPACE ===')
A = cell(1,4);
for i = 1:4
    A{i} = simplify(dhA(theta(i), d(i), a(i), alpha(i)));
    fprintf('A%d (frame %d -> %d) =\n', i, i-1, i); disp(A{i})
end

T = sym(eye(4));
for i = 1:4
    T = simplify(T*A{i});
    fprintf('T_0%d =\n', i); disp(T)
end

T_be = T_b0*T*T_4e;
% phi = q2 - q3 + q4, dipakai untuk menyederhanakan
T_be_phi = simplify(subs(T_be, q4, phi - q2 + q3), 'Steps', 100);
disp('T_be(q), dengan phi = q2 - q3 + q4 ='); disp(T_be_phi)
disp('T_be(q) numerik ='); disp(vpa(subs(T_be_phi, par, val), 6))

%% 3. FORWARD KINEMATICS
disp('=== 3. FORWARD KINEMATICS ===')
r = -L1*sin(q2) + L2*cos(q2-q3) + L3*cos(phi) + e*sin(phi);
h =  L1*cos(q2) + L2*sin(q2-q3) + L3*sin(phi) - e*cos(phi);
x_fk = bx - (a1 + r)*sin(q1) + d4*cos(q1);
y_fk = by + (a1 + r)*cos(q1) + d4*sin(q1);
z_fk = d1 + h;
phi_fk = q2 - q3 + q4;

fprintf('r   = '); disp(r)
fprintf('h   = '); disp(h)
fprintf('x   = '); disp(x_fk)
fprintf('y   = '); disp(y_fk)
fprintf('z   = '); disp(z_fk)
fprintf('phi = '); disp(phi_fk)

% bukti: hasil di atas sama dengan kolom posisi T_be (harus nol)
cek = simplify(T_be_phi(1:3,4) - [x_fk; y_fk; z_fk], 'Steps', 100);
disp('Selisih FK ringkas vs perkalian DH (harus nol):'); disp(cek)

%% 4. MATRIKS TRANSFORMASI TASK SPACE  T(x,y,z,phi)
disp('=== 4. TRANSFORMASI TASK SPACE ===')
X = x - bx;
Y = y - by;
rho = sigma1*sqrt(X^2 + Y^2 - d4^2);
% dari q1 = atan2(Y,X) - atan2(rho,d4):
c1 = (X*d4 + Y*rho)/(X^2 + Y^2);
s1 = (Y*d4 - X*rho)/(X^2 + Y^2);

syms C1 S1 real
T_ringkas = sym(eye(4));
T_ringkas(1,:) = [-C1, -S1*sin(phi),  S1*cos(phi), x];
T_ringkas(2,:) = [-S1,  C1*sin(phi), -C1*cos(phi), y];
T_ringkas(3,:) = [sym(0),  -cos(phi),    -sin(phi), z];
disp('T_task (C1 = cos q1, S1 = sin q1) ='); disp(T_ringkas)
disp('C1 ='); disp(c1)
disp('S1 ='); disp(s1)
disp('rho ='); disp(rho)
T_task = subs(T_ringkas, [C1 S1], [c1 s1]);
disp('T_task(x,y,z,phi) lengkap ='); disp(T_task)

% bukti: bentuk ringkas = T_be (harus nol)
cekT = simplify(subs(T_ringkas, [C1 S1 x y z], ...
       [cos(q1) sin(q1) x_fk y_fk z_fk]) - T_be_phi, 'Steps', 100);
disp('Selisih T_task vs T_be (harus nol):'); disp(cekT)

%% 5. INVERSE KINEMATICS (penurunan)
disp('=== 5. INVERSE KINEMATICS ===')
Xf = x_fk - bx;
Yf = y_fk - by;
disp('(a) X*cos(q1) + Y*sin(q1) =')
disp(simplify(Xf*cos(q1) + Yf*sin(q1), 'Steps', 100))
disp('(b) -X*sin(q1) + Y*cos(q1) =  a1 + r = rho')
disp(simplify(-Xf*sin(q1) + Yf*cos(q1), 'Steps', 100))
disp('    sehingga X^2 + Y^2 = d4^2 + rho^2')

q1_ik = atan2(Y, X) - atan2(rho, d4);

syms RHO RW HW D delta real
rw = RHO - a1 - L3*cos(phi) - e*sin(phi);
hw = z - d1 - L3*sin(phi) + e*cos(phi);
disp('(c) reduksi ke 2 link planar:')
fprintf('rw = '); disp(rw)
fprintf('hw = '); disp(hw)

rw_q = -L1*sin(q2) + L2*cos(q2-q3);
hw_q =  L1*cos(q2) + L2*sin(q2-q3);
disp('(d) rw^2 + hw^2 =')
disp(simplify(rw_q^2 + hw_q^2, 'Steps', 100))
disp('    dengan delta = -q3 - pi/2, maka cos(delta) = -sin(q3)')

D_ik     = (RW^2 + HW^2 - L1^2 - L2^2)/(2*L1*L2);
delta_ik = atan2(sigma2*sqrt(1 - D^2), D);
q2_ik    = atan2(HW, RW) - atan2(L2*sin(delta), L1 + L2*cos(delta)) - hp;
q3_ik    = -delta - hp;
q4_ik    = phi - q2 + q3;

disp('--- PERSAMAAN IK ---')
fprintf('rho   = '); disp(rho)
fprintf('q1    = '); disp(q1_ik)
fprintf('D     = '); disp(D_ik)
fprintf('delta = '); disp(delta_ik)
fprintf('q2    = '); disp(q2_ik)
fprintf('q3    = '); disp(q3_ik)
fprintf('q4    = '); disp(q4_ik)
disp('RHO = rho, RW = rw, HW = hw. Jika input matriks T: phi = atan2(-T(3,3), -T(3,2))')

%% 6. UJI NUMERIK: q -> FK -> IK
disp('=== 6. UJI ===')
fk_num = matlabFunction(subs(T_be, par, val), 'Vars', {q1,q2,q3,q4});

q_uji = deg2rad([30 20 -40 15]);
T_uji = fk_num(q_uji(1), q_uji(2), q_uji(3), q_uji(4));
phi_uji = q_uji(2) - q_uji(3) + q_uji(4);
disp('q uji (deg):'); disp(rad2deg(q_uji))
disp('T_be:'); disp(T_uji)

sol = ik_arm(T_uji(1,4), T_uji(2,4), T_uji(3,4), phi_uji, val);
disp('Solusi IK dalam limit +-90 deg (tiap baris q1..q4, deg):'); disp(rad2deg(sol))
for k = 1:size(sol,1)
    Tk = fk_num(sol(k,1), sol(k,2), sol(k,3), sol(k,4));
    fprintf('error solusi %d = %.2e\n', k, norm(Tk - T_uji));
end

%% FUNGSI LOKAL
function A = dhA(th, d, a, al)
    A = sym(eye(4));
    A(1,:) = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)];
    A(2,:) = [sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th)];
    A(3,:) = [sym(0),   sin(al),          cos(al),         d];
end

function sol = ik_arm(x, y, z, phi, v)
    % v = [bx by d1 a1 L1 L2 L3 d4 e]
    bx=v(1); by=v(2); d1=v(3); a1=v(4); L1=v(5); L2=v(6); L3=v(7); d4=v(8); e=v(9);
    wrap = @(t) atan2(sin(t), cos(t));
    sol = zeros(0,4);
    X = x - bx;  Y = y - by;
    k = X^2 + Y^2 - d4^2;
    if k < 0, return; end
    for s1 = [1 -1]
        rho = s1*sqrt(k);
        q1 = atan2(Y, X) - atan2(rho, d4);
        rw = rho - a1 - L3*cos(phi) - e*sin(phi);
        hw = z - d1 - L3*sin(phi) + e*cos(phi);
        D = (rw^2 + hw^2 - L1^2 - L2^2)/(2*L1*L2);
        if abs(D) > 1 + 1e-9, continue; end
        D = max(min(D, 1), -1);
        for s2 = [1 -1]
            delta = atan2(s2*sqrt(1 - D^2), D);
            q2 = atan2(hw, rw) - atan2(L2*sin(delta), L1 + L2*cos(delta)) - pi/2;
            q3 = -delta - pi/2;
            q4 = phi - q2 + q3;
            q = wrap([q1 q2 q3 q4]);
            if all(abs(q) <= pi/2 + 1e-9)
                sol(end+1,:) = q; %#ok<AGROW>
            end
        end
    end
end