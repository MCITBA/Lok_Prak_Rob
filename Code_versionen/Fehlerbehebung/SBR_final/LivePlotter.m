% MATLAB Live-Plotter & Daten-Logger für ESP32
clear; clc; close all;

comPort = "COM8"; % <-- HIER deinen Arduino COM-Port eintragen!
baudRate = 115200;

% Verbindung zum ESP32 aufbauen
fprintf("Verbinde mit %s...\n", comPort);
device = serialport(comPort, baudRate);
configureTerminator(device, "LF");
flush(device);

% Grafikfenster vorbereiten
figure('Name', 'ESP32 Realtime PID Tuning', 'NumberTitle', 'off');
h1 = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Ist-Winkel (°)');
h2 = animatedline('Color', 'g', 'LineStyle', '--', 'DisplayName', 'P-Anteil');
h3 = animatedline('Color', 'b', 'LineStyle', ':', 'DisplayName', 'I-Anteil');
h4 = animatedline('Color', 'm', 'LineStyle', '-.', 'DisplayName', 'D-Anteil');
h5 = animatedline('Color', 'k', 'LineWidth', 1.2, 'DisplayName', 'PWM Effektiv');

xlabel('Messpunkte (Zeit)');
ylabel('Wert');
title('Live-Datenstrom vom Balancier-Roboter');
grid on;
legend('Location', 'northwest');

% Daten-Speicher für unendlich langes Loggen
dataLog = [];
count = 1;

fprintf("Lese Daten... Drücke Strg+C im MATLAB-Fenster zum Stoppen.\n");

% Endlosschleife zum Plotten
while true
    try
        % Zeile vom ESP32 einlesen
        line = readline(device);
        
        if ~isempty(line)
            % Regulärer Ausdruck, um nur die Zahlen aus dem Text zu filtern
            numbers = regexp(line, '[-+]?\d*\.?\d+', 'match');
            
            if length(numbers) >= 5
                % Zahlen in Double-Werte umwandeln
                val_pitch = str2double(numbers{1});
                val_p     = str2double(numbers{2});
                val_i     = str2double(numbers{3});
                val_d     = str2double(numbers{4});
                val_pwm   = str2double(numbers{5});
                
                % Live-Kurven im Plot aktualisieren (schiebt sich NICHT aus dem Bild)
                addpoints(h1, count, val_pitch);
                addpoints(h2, count, val_p);
                addpoints(h3, count, val_i);
                addpoints(h4, count, val_d);
                addpoints(h5, count, val_pwm);
                
                % Daten im RAM sichern
                dataLog(count, :) = [count, val_pitch, val_p, val_i, val_d, val_pwm];
                count = count + 1;
                
                % Grafik alle paar Punkte auffrischen (schont die CPU)
                if mod(count, 5) == 0
                    drawnow;
                end
            end
        end
    catch
        fprintf("\nMessung abgebrochen oder USB-Kabel gezogen.\n");
        break;
    end
end

% Am Ende (nach Strg+C) die Daten als Excel/CSV speichern
fprintf("Speichere aufgezeichnete Daten in 'robot_data.csv'...\n");
T = array2table(dataLog, 'VariableNames', {'Sample', 'Pitch', 'P_Anteil', 'I_Anteil', 'D_Anteil', 'PWM'});
writetable(T, 'robot_data.csv');
fprintf("Fertig! Du kannst die Daten nun mit 'plot(robot_data.Pitch)' analysieren.\n");