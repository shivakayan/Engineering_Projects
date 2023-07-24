close all;
clear;
clc;
% audio recording
Fs =4000;
Channels = 1;
bits = 16;

%recording first audio file
r1 = audiorecorder(Fs,bits, Channels);
duration = 5; disp('audio recording started x1');
recordblocking(r1,duration);
disp('audio recording stopped x1');
X1 = getaudiodata(r1);

% creating first audio file
filename = 'myvoice1.wav';
audiowrite(filename,X1,Fs);

pause(5);

%recording second audio file
r2 = audiorecorder(Fs,bits, Channels);
duration = 5; disp('audio recording started x2');
recordblocking(r2,duration);
disp('audio recording stopped x2');
X2 = getaudiodata(r2);

% creating second audio file
filename = 'myvoice2.wav';
audiowrite(filename,X2,Fs);

%listening to the  first audio sound
disp('first audio file');
sound(X1,Fs,bits);

pause(10);

%listening to second audio sound
disp('second audio file');
sound(X2,Fs,bits);

% graph the audio x1
t = 0:1/Fs:(length(X1)-1)/Fs;
subplot(2,2,1); plot(t,X1,'LineWidth',1.5);
xlabel('time(sec)'); ylabel('Amplitude');
title('audio time domain - x1(t)')
n=length(X1); 
Y1=fft(X1,n);F=0:(n-1)*Fs/n;
F_0=(-n/2:n/2-1).*(Fs/n);
Y_0=fftshift(Y1);
AY_0=abs(Y_0);
subplot(2,2,2);plot(F_0,AY_0,'LineWidth',1.5);
xlabel('Frequency(Hz)'); ylabel('Amplitude');
title('Audio frequency domain - X1(W)')

%graph the audio x2
t = 0:1/Fs:(length(X2)-1)/Fs;
subplot(2,2,3); plot(t,X2,'LineWidth',1.5);
xlabel('time(sec)'); ylabel('Amplitude');
title('audio time domain - x2(t)')
n=length(X2); 
Y2=fft(X1,n);
F_1=(-n/2:n/2-1).*(Fs/n);
Y_1=fftshift(Y2);
AY_1=abs(Y_1);
subplot(2,2,4);plot(F_1,AY_1,'LineWidth',1.5);
xlabel('Frequency(Hz)'); ylabel('Amplitude');
title('Audio frequency domain - X2(W)')

%Multiplying 2 audio signals in time domain
x3=X1.*X2;

% creating multiplied audio file
filename = 'myvoice1_2.wav';
audiowrite(filename,x3,Fs);

pause(10);

%listening to combinational sound
disp('multiplication audio file');
sound(x3,Fs,bits);

%graphing the Multiplied audio signal in time domain
t = 0:1/Fs:(length(x3)-1)/Fs;
figure;
subplot(2,1,1); plot(t,x3,'LineWidth',1.5);
xlabel('time(sec)'); ylabel('Amplitude');
title('audio signals multiplication time domain')

%Multiplying 2 audio signals in frequency domain
y3=Y1.*Y2;

%%graphing the Multiplied audio signal in Frequency domain
if(length(X1)>length(X2))
    n=length(X1);   
else
    n=length(X2);
end
F_3=(-n/2:n/2-1).*(Fs/n);
Y_3=fftshift(y3);
AY_3=abs(Y_3);
subplot(2,1,2);plot(F_3,AY_3,'LineWidth',1.5);
xlabel('Frequency(Hz)'); ylabel('Amplitude');
title('audio signals multiplication frequency domain')







