function [s] = NI_initialization(number_of_trucks)
%NI_INITIALIZATION Summary of this function goes here
%   Detailed explanation goes here

devices=daq.getDevices;
% Create the session
s=daq.createSession('ni');
% Adding output channels

if (number_of_trucks) >=1
    s.addAnalogOutputChannel(devices(1).ID,2,'Voltage');
    s.addAnalogOutputChannel(devices(1).ID,3,'Voltage');
end

if (number_of_trucks) >=2
    
    s.addAnalogOutputChannel(devices(1).ID,4,'Voltage');
    s.addAnalogOutputChannel(devices(1).ID,5,'Voltage');
    
end

if (number_of_trucks) >=3
    
   s.addAnalogOutputChannel(devices(1).ID,8,'Voltage');
   s.addAnalogOutputChannel(devices(1).ID,9,'Voltage');
    
end

if (number_of_trucks) >=4
    
   error('NI_initialization(), not handling for more than 3 trucks, please do it yourself.')
    
end

s.addAnalogOutputChannel(devices(1).ID,12,'Voltage');



end

