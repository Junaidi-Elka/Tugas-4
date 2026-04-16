% Particle filter localization
%%
clc,clear;
% Simulation parameter
TMAX = 2;
dt = 0.01;
t = 0;

% Anchor points X, Y
ap = [[0.5, -1];
      [-1.5, 0];
      [-0.75, -2];
      [-1, 2];
      ];

% Real position, random xc, yc
xc = rand - 0.5;
yc = rand - 0.5;
RP = realPos(t, TMAX, xc, yc);

% Initialize the number of particles and their initial positions
numParticles = 100;
particlePos = rand(numParticles, 2) * 4 - 2; % Random positions within a 4x4 area

d_ac_p = zeros(length(ap), numParticles);

L_p = zeros(1, numParticles);
%%
% Loop
% v = VideoWriter('D:\S2 ITS\Semester 1\algoritma dan komputasi\particle_filter.mp4', 'MPEG-4');
% v.FrameRate = 5;
% open(v);
while t<TMAX
    %%
    t = t+dt;
    [RP, theta] = realPos(t, TMAX, xc, yc);

    % Prediction is average
    x_pred = sum(particlePos(:,1))/numParticles;
    y_pred = sum(particlePos(:,2))/numParticles;
    
    dist = sqrt((particlePos(:,1) - RP(1)).^2 + (particlePos(:,2) - RP(2)).^2);
    dist_norm = (dist - min(dist)) / (max(dist) - min(dist) + eps);

    % merah (dekat) ? hijau (jauh)
%     colors = [dist_norm, 1 - dist_norm, zeros(numParticles,1)];
    colors = [1-dist_norm, dist_norm, zeros(numParticles,1)];
    % Plot
    figure(1);
%     plot(particlePos(:,1), particlePos(:,2), 'bo', 'MarkerSize',5); % Posisi partikel
    scatter(particlePos(:,1), particlePos(:,2), 25, colors, 'filled'); % partikel berwarna
    hold on
    plot(x_pred, y_pred, 'k.', 'MarkerSize',15); % Posisi prediction
    plot(RP(1), RP(2), 'r*', 'MarkerSize',12); % posisi sebenarnya
    plot(ap(:,1), ap(:,2), 'x', 'MarkerSize',18);
    plot(xc, yc, 'b*', 'MarkerSize',5); % Posisi center
    hold off
%     frame = getframe(gcf);
%     writeVideo(v, frame);

    % RSSI anchor to key
    d_ac_k = sqrt(sum((RP-ap).^2, 2));

    % RSSI particle to key
    for i = 1:numParticles
        d_ac_p(:, i) = sqrt(sum((particlePos(i, :)-ap).^2, 2));
    end
    
    % Particle Likelyhood of each particle
%     for i = 1:numParticles
%         L_p(:, i) = 1/sum(abs(d_ac_p(:, i) - d_ac_k));
%     end
%     L_p = L_p/sum(L_p);
    L_p = zeros(numParticles,1);

    for i = 1:numParticles
        L_p(i) = 1 / (sum(abs(d_ac_p(:, i) - d_ac_k)) + eps);
    end

    % Normalisasi
    L_p = L_p / sum(L_p);

    % Resample particles based on likelihood
    indices = randsample(1:numParticles, numParticles, true, L_p);
    particlePos = particlePos(indices, :);
    
    % Update particle positions based on motion model
    for i = 1:numParticles
        % distance to center
        tocenter = sqrt(sum((particlePos(i, :)-[xc yc]).^2));
        xy_relative = (particlePos(i, :)-[xc yc]);
        theta_p = atan2(xy_relative(2), xy_relative(1));

        w = 3/4*2*pi/TMAX;

        theta_p = theta_p + w*dt;
        
        particlePos(i, 1) = xc + tocenter*cos(theta_p) + randn * 0.04;
        particlePos(i, 2) = yc + tocenter*sin(theta_p) + randn * 0.04;
    end

end
% close(v);

% Real key posisitoin moving on a circle
function [out, theta] = realPos(t, TMAX, xc, yc)
    r = 1;
    theta = 3/4*2*pi*t/TMAX;
    x = xc + r*cos(theta);
    y = yc + r*sin(theta);
    out = [x, y];
end
