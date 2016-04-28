% Plots

clear all 
close all
clc

%% t0

N = 50;

p = [10*(rand(1,N)-0.5); 10*(rand(1,N)-0.5); zeros(1,N)];
figure;
stem3(p(1,:),p(2,:),p(3,:))
xlabel('beacon_1');
ylabel('beacon_2');
zlabel('train');
title('t0');
axis([-5 5 -5 5 0 1]);

%% t1

p(3,:) = 10 + randn(1,N);
figure;
stem3(p(1,:),p(2,:),p(3,:))
xlabel('beacon_1');
ylabel('beacon_2');
zlabel('train');
title('t1');
axis([-5 5 -5 5 0 13]);


%% t2

p(1,:) = p(3,:) + 15 + randn(1,N);
figure;
stem3(p(1,:),p(2,:),p(3,:))
xlabel('beacon_1');
ylabel('beacon_2');
zlabel('train');
title('t2');
axis([0 30 -5 5 0 13]);



%% t3

p(2,:) = p(3,:) - 5 + randn(1,N);
figure;
stem3(p(1,:),p(2,:),p(3,:))
xlabel('beacon_1');
ylabel('beacon_2');
zlabel('train');
title('t3');
axis([0 30 0 10 0 13]);


%% t4

p(3,:) = p(3,:) + 20 + 2*randn(1,N);
figure;
stem3(p(1,:),p(2,:),p(3,:))
xlabel('beacon_1');
ylabel('beacon_2');
zlabel('train');
title('t4');
axis([0 30 0 10 0 35]);







