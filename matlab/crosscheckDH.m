%% Cross-check FK robot 5 joint + EE (DH standar)
% Urutan joint: q1 yaw, q2 shoulder, q3 elbow, q4 wrist pitch, q5 wrist roll
% Satuan: mm, radian. Jalankan per section (Ctrl+Enter) atau sekaligus.
clear; clc; close all;

% ---- Parameter ----
P.d1 = 23; P.a1 = 23.5; P.a2 = 104.5; P.a3 = 207.54; P.d5 = 60;

% Offset theta supaya q = 0 = pose home (shoulder tegak, forearm + tool lurus ke depan)
OFF = deg2rad([0 90 -90 90 0]);

%% 1) Cek simbolik: DH vs rumus tertutup (yang dipakai di Arduino)
% Perlu Symbolic Math Toolbox. Lewati section ini kalau tidak ada.
syms q1 q2 q3 q4 q5 real
q = [q1 q2 q3 q4 q5];
T = eye(4);
for i = 1:5
    T = T * dh_i(i, q(i) + OFF(i), P);
end
T = simplify(T);

u = P.a1 - P.a2*sin(q2) + P.a3*cos(q2+q3) + P.d5*cos(q2+q3+q4);
w = P.d1 + P.a2*cos(q2) + P.a3*sin(q2+q3) + P.d5*sin(q2+q3+q4);
errPos = simplify([T(1,4) - u*cos(q1); T(2,4) - u*sin(q1); T(3,4) - w]);
disp('Selisih posisi (harus 0 0 0):'); disp(errPos.');

ph = q2 + q3 + q4;
errDir = simplify(T(1:3,3) - [cos(ph)*cos(q1); cos(ph)*sin(q1); sin(ph)]);
disp('Selisih arah tool z5 (harus 0 0 0):'); disp(errDir.');

%% 2) Home pose (q = 0) + plot
Tk = fk_all(zeros(1,5), OFF, P);
fprintf('EE home    = [%.2f %.2f %.2f]  (harus 291.04 0 127.50)\n', Tk{end}(1:3,4));
fprintf('arah tool  = [%.2f %.2f %.2f]  (harus 1 0 0)\n', Tk{end}(1:3,3));
figure('Name','Home pose','Position',[100 100 800 600]);
plot_robot(axes, Tk, P, 'Home pose (q = 0)');

%% 3) Pose bebas (isi sendiri sudut dalam derajat), bandingkan dengan SolidWorks
qdeg = [0 -45 30 15 0];
Tk = fk_all(deg2rad(qdeg), OFF, P);
disp('T05 ='); disp(Tk{end});
figure('Name','Pose bebas','Position',[150 150 800 600]);
plot_robot(axes, Tk, P, sprintf('q = [%s] deg', num2str(qdeg)));

%% 4) Visualisasi interaktif: geser slider q1..q5 (derajat)
slider_gui(P, OFF);

%% ---- fungsi lokal ----
function A = dh(th,d,a,al)
A = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
     sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
     0        sin(al)          cos(al)         d;
     0        0                0               1];
end

function A = dh_i(i, th, P)
%        d     a     alpha
tab = [  P.d1  P.a1   pi/2;
         0     P.a2   0;
         0     P.a3   0;
         0     0      pi/2;
         P.d5  0      0 ];
A = dh(th, tab(i,1), tab(i,2), tab(i,3));
end

function Tk = fk_all(q, off, P)
Tk = cell(1,6); Tk{1} = eye(4);
for i = 1:5
    Tk{i+1} = Tk{i} * dh_i(i, q(i) + off(i), P);
end
end

function plot_robot(ax, Tk, P, ttl)
% Link berwarna: tiap link digambar mengikuti jalur DH (d sepanjang z, lalu a sepanjang x)
[az, el] = view(ax);
if isequal([az el],[0 90]), az = 35; el = 20; end
cla(ax); hold(ax,'on');

d   = [P.d1 0 0 0 P.d5];
col = [0.85 0.33 0.10;    % link 1 oranye
       0.00 0.45 0.74;    % link 2 biru
       0.47 0.67 0.19;    % link 3 hijau
       0.49 0.18 0.56;    % link 4 (panjang nol, tidak terlihat)
       0.93 0.69 0.13];   % link 5 kuning
nama = {'Link 1 (yaw -> shoulder)','Link 2 (shoulder -> elbow)', ...
        'Link 3 (elbow -> wrist)','Link 5 (wrist -> EE, sumbu roll)'};

h = gobjects(1,5);
for i = 1:5
    o0  = Tk{i}(1:3,4);  z0 = Tk{i}(1:3,3);  o1 = Tk{i+1}(1:3,4);
    mid = o0 + d(i)*z0;
    h(i) = plot3(ax,[o0(1) mid(1) o1(1)],[o0(2) mid(2) o1(2)],[o0(3) mid(3) o1(3)], ...
                 '-','Color',col(i,:),'LineWidth',7);
end

% Joint: Ji berputar pada sumbu z dari frame i-1, ditandai di o_{i-1}
dz = [15 15 15 15 -20];   % J4 dan J5 berimpit di wrist, label dipisah
for i = 1:5
    o = Tk{i}(1:3,4);
    plot3(ax,o(1),o(2),o(3),'o','MarkerSize',10,'MarkerFaceColor','k','MarkerEdgeColor','k');
    text(ax,o(1),o(2),o(3)+dz(i),sprintf(' J%d',i),'FontWeight','bold');
end
oe = Tk{6}(1:3,4);
plot3(ax,oe(1),oe(2),oe(3),'d','MarkerSize',12,'MarkerFaceColor','r','MarkerEdgeColor','r');
text(ax,oe(1),oe(2),oe(3)+15,' EE','Color','r','FontWeight','bold');

% Triad tiap frame (x merah, y hijau, z biru)
L = 25;
for i = 1:6
    o = Tk{i}(1:3,4); R = Tk{i}(1:3,1:3);
    quiver3(ax,o(1),o(2),o(3),R(1,1)*L,R(2,1)*L,R(3,1)*L,0,'r');
    quiver3(ax,o(1),o(2),o(3),R(1,2)*L,R(2,2)*L,R(3,2)*L,0,'g');
    quiver3(ax,o(1),o(2),o(3),R(1,3)*L,R(2,3)*L,R(3,3)*L,0,'b');
end

% Lantai (z = 0 di poros atas servo base)
fill3(ax,[-150 300 300 -150],[-200 -200 200 200],[0 0 0 0],[0.8 0.8 0.8], ...
      'FaceAlpha',0.25,'EdgeColor','none');

xlim(ax,[-150 450]); ylim(ax,[-300 300]); zlim(ax,[-50 450]);
daspect(ax,[1 1 1]); grid(ax,'on'); box(ax,'on');
xlabel(ax,'X (mm)'); ylabel(ax,'Y (mm)'); zlabel(ax,'Z (mm)');
title(ax,ttl); view(ax,az,el);
legend(ax,h([1 2 3 5]),nama,'Location','northeastoutside');
end

function slider_gui(P, off)
f  = figure('Name','Slider FK','Position',[100 100 1000 650]);
ax = axes('Parent',f,'Position',[0.05 0.22 0.65 0.74]);
sl = gobjects(1,5); tx = gobjects(1,5);
for i = 1:5
    x = 0.03 + 0.19*(i-1);
    tx(i) = uicontrol(f,'Style','text','Units','normalized', ...
                      'Position',[x 0.09 0.17 0.04],'String',sprintf('q%d = 0 deg',i));
    sl(i) = uicontrol(f,'Style','slider','Units','normalized','Position',[x 0.05 0.17 0.04], ...
                      'Min',-180,'Max',180,'Value',0,'Callback',@(~,~) refresh());
end
refresh();
    function refresh()
        qd = arrayfun(@(s) s.Value, sl);
        for k = 1:5, tx(k).String = sprintf('q%d = %.0f deg',k,qd(k)); end
        plot_robot(ax, fk_all(deg2rad(qd), off, P), P, 'FK interaktif');
    end
end