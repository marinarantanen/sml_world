function []=NI_voltage_stop(s, number_of_vehicles)
%NI_VOLTAGE_OUTPUT Send voltage to NI outputs
%   It will limit the voltage to [0,3.2]

% Must limit the voltage to [0,3.2], to not fry the Scania Truck Remote
% Control
%     v1=3.2/2;
    stop_voltage = 1.7;
    
    voltage_vector = stop_voltage*ones(1, 2*number_of_vehicles);
    voltage_vector = [voltage_vector stop_voltage];

    s.outputSingleScan(voltage_vector);

end

