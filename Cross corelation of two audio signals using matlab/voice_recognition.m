close all;
clear;
clc;
% audio recording
Fs =16000;
Channels = 1;
bits = 16;
X1=audioread("anirudh.wav");
X2=audioread("sharanya.wav");
X3=audioread("arun.wav");
X4=audioread("nithin.wav");
X5=audioread("saicharan.wav");
X6=audioread("salma.wav");
X7=audioread("rajnesh.wav");
X8=audioread("mahesree.wav");
X9=audioread("shivakalyan.wav");
pause(5);
rt = audiorecorder(Fs,bits, Channels);
duration = 10; disp('recording started test');
recordblocking(rt,duration);
disp('audio recording stopped test');
Xt = getaudiodata(rt);

%graph the audio xt
t = 0:1/Fs:(length(Xt)-1)/Fs;
figure(1); plot(t,Xt,'LineWidth',1.5);
xlabel('time(sec)'); ylabel('Amplitude');
title('audio time domain - xt');

%Find whether they both are of same person or not
[c,lags]=xcorr(Xt,X1);
figure(2);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("anirudh");
anirudh=max(c);
[c,lags]=xcorr(Xt,X2);
figure(3);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("Sharanya");
sharanya=max(c);
[c,lags]=xcorr(Xt,X3);
figure(4);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("nithin")
nithin=max(c);
[c,lags]=xcorr(Xt,X4);
figure(5);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("arun");
arun=max(c);
[c,lags]=xcorr(Xt,X5);
figure(6);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("saicharan");
saicharan=max(c);
[c,lags]=xcorr(Xt,X6);
figure(7);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("salma");
salma=max(c);
[c,lags]=xcorr(Xt,X7);
figure(8);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("rajnesh");
rajnesh=max(c);
[c,lags]=xcorr(Xt,X8);
figure(9);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("mahesree");
mahesree=max(c);
[c,lags]=xcorr(Xt,X9);
figure(10);
stem(lags,c);
xlabel("LAGS");
ylabel("corelation");
title("shivakalyan");
shivakalyan=max(c);

maxvalues=[anirudh sharanya nithin arun saicharan salma mahesree rajnesh shivakalyan];
answer=max(maxvalues);
switch answer
    case anirudh
        disp("The voice is anirudh's");
    case sharanya
        disp("The voice is sharanya's"); 
    case nithin
        disp("The voice is nithin's"); 
    case arun
        disp("The voice is arun's"); 
    case saicharan
        disp("The voice is saicharan's"); 
    case salma
        disp("The voice is salma's"); 
    case mahesree
        disp("The voice is mahesree's"); 
    case rajnesh
        disp("The voice is rajnesh's"); 
    case shivakalyan
        disp("The voice is shivakalyan's"); 
    otherwise     
        disp("The Voice is not yet registered");
end