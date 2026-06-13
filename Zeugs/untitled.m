%% MATLAB-Skript: Live-Datenstrom mit korrekter Spaltenzuordnung
clear; clc; close all;

filename = 'Sturz.csv';

if ~exist(filename, 'file')
    error('Die Datei "%s" wurde nicht gefunden.', filename);
end

% CSV-Daten importieren
data = readmatrix(filename);

%% 2. Daten exakt nach deiner CSV-Struktur zuweisen
% Spalte 1: Sample (Das ist deine X-Achse / Zeitachse!)
messpunkte    = data(:, 1); 

% Spalten 2 bis 6: Die eigentlichen Y-Werte
pitchDeg      = data(:, 2); % Pitch (Ist-Winkel) -> Spalte 2
last_P_out    = data(:, 3); % P_Anteil           -> Spalte 3
last_I_out    = data(:, 4); % I_Anteil           -> Spalte 4
last_D_out    = data(:, 5); % D_Anteil           -> Spalte 5
signedRealPwm = data(:, 6); % PWM (Effektiv)     -> Spalte 6

%% 3. Grafische Auswertung (Plots erstellen)
figure('Name', 'Live-Datenstrom vom Balancier-Roboter', 'NumberTitle', 'off', 'Color', 'w');
hold on;

% Jetzt wird gegen "messpunkte" (Spalte 1) geplottet!
plot(messpunkte, pitchDeg, 'r-', 'LineWidth', 2.0);       % Ist-Winkel (Rot, durchgezogen)
plot(messpunkte, last_P_out, 'g--', 'LineWidth', 1.2);    % P-Anteil (Grün, gestrichelt)
plot(messpunkte, last_I_out, 'b:', 'LineWidth', 1.5);     % I-Anteil (Blau, gepunktet)
plot(messpunkte, last_D_out, 'm-.', 'LineWidth', 1.2);    % D-Anteil (Magenta, strichpunktiert)
plot(messpunkte, signedRealPwm, 'k-', 'LineWidth', 1.8);  % PWM Effektiv (Schwarz, durchgezogen)

%% 4. Achsenbeschriftungen und Design
grid on;
xlabel('Messpunkte (Zeit)');
ylabel('Wert');
title('Live-Datenstrom vom Balancier-Roboter', 'FontWeight', 'bold');

% Achsenbegrenzung passend zu deinem Screenshot
if max(messpunkte) > 3400
    xlim([2450, 3410]); 
end
ylim([-650, 500]); 

legend('Ist-Winkel (°)', 'P-Anteil', 'I-Anteil', 'D-Anteil', 'PWM Effektiv', ...
       'Location', 'northwest', 'Color', 'w');

set(gca, 'FontSize', 11);