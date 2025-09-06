clear; clc;

%% TPS61288
%% Constants
Vinmax = 8.4;       % V
Vinmin = 7.4;       % V
Vout = 12;          % V
VRef = 0.6;         % V
IoutMax = 6;        % A

%% Voltage divider
R_FB_RATIO = (Vout - VRef) / VRef;

%% Peak current limit
IPeak = 15;

%% inductor Selection
eta = 0.9;  % power conversion efficiency
IDC = (Vout*IoutMax)/(Vinmin*eta);
  
% inductor current peak-to-peak
L = 2.2e-6;     % 2.2uH for the Iteration with -30% tolerance
fSW = 500e3;     % nom: 500KHz
IPP = 1/(0.7*L*(1/(Vout-Vinmin)+1/Vinmin)*fSW);
    % L is the inductor value
    % IPP is the inductor peak-to-peak ripple

% peak current
ILpeak = IDC+IPP/2;

%% output Capacitor Selection
VRippleDis = 50e-3; % 50 mV
RCESR = 0.01; % Ohm
COut = ((Vout-Vinmin)*IoutMax)/(Vout*fSW*VRippleDis);

CO = 100e-6; % 100 uF chosen
    
VrippleESR = ILpeak * RCESR;

%%  Loop stability
RO = Vout / IoutMax;
D = 1 - Vinmin / Vout;

fP = 2/(2*pi*RO*CO);

fESRZ = 1/(2*pi*RCESR*CO);

fRHPZ = (RO*(1-D)^2)/(2*pi*L);

fCmin = min(fSW / 10, fRHPZ / 5)
fC = 10e3; % selected crossover frequency

GEA = 180e-6; % S
KCOMP = 13.5; % A / V

RC = (2*pi*Vout*CO*fC)/((1-D)*VRef*GEA*KCOMP);

CC = (RO*CO)/(2*RC);

CP = (RCESR*CO)/RC;

fprintf('Inductor DC current: %.3f A\n', IDC);
fprintf('Inductor peak-to-peak ripple current: %.3f A\n', IPP);
fprintf('Peak inductor current: %.3f A\n', ILpeak);
fprintf('Minimum effective C_OUT: %.1f uF\n', COut*10^6);
fprintf('Output capacitor value: %.0f uF\n', CO*10^6);
%fprintf('VRippleESR: %.3f V\n', VrippleESR);
fprintf('Phase margin frequency: %.0f Hz\n', fP*10^-3);
fprintf('ESR zero frequency: %.0f Hz\n', fESRZ*10^-3);
fprintf('Right half-plane zero frequency: %.0f Hz\n', fRHPZ*10^-3);
fprintf('Maximum Crossover frequency: %.2f kHz\n', fCmin*10^-3);
fprintf('Crossover frequency: %.0f kHz\n', fC*10^-3);
fprintf('Compensation resistor RC: %.1f kOhm\n', RC*10^-3);
fprintf('Compensation capacitor CC: %.1f nF\n', CC*10^9);
fprintf('Compensation pole capacitor CP: %.1f pF\n', CP*10^12);