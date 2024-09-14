%%% NO ES PARA EJECUTAR TODO EL SCRIPT, SINO, LINEA POR LINEA O SECCIONES
serialportlist("available") % Find the serial port that the HC-06 is connected to

%% Escribir Direcciones
write(HC_06,"ADE","string")
write(HC_06,"ATR","string")
write(HC_06,"DER","string")
write(HC_06,"IZQ","string")
%% Sección de finales
clear HC_06 %Borrar objeto
clc; clear all;

%% Sección para configurar
HC_06 = serialport('COM6',9600) % Connect to the HC-06
configureTerminator(HC_06,"CR/LF"); %Set the Terminator property to match the terminator that you specified in the Arduino code.
flush(HC_06) % Flush the serialport object to remove any old data.
configureCallback(HC_06,"terminator",@evento_HC_06); %The callback function is triggered when a new data (with the terminator) is available to be read from the Arduino.

%% Asi reconoce la funcion
%read(HC_06,10,"string") % leee cantidad de caracteres, o hasta que el tiempo agote

function evento_HC_06(HC_06, ~)
% Read the ASCII data from the serialport object.
data=readline(HC_06);
disp(strcat('Recibido:',' ',data))
end
