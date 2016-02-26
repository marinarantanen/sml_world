function []=NI_voltage_output(s,voltage_vector)
%NI_VOLTAGE_OUTPUT Send voltage to NI outputs
%   It will limit the voltage to [0,3.2]

% Must limit the voltage to [0,3.2], to not fry the Scania Truck Remote
% Control

    new_voltage_vector = voltage_vector(1:end-1);
    
    trailer_voltage = voltage_vector(end);
    
    for i = 1:length(new_voltage_vector)/2
        
        new_voltage_vector((i-1)*2 + 1) = voltage_vector(i*2);
        new_voltage_vector(i*2) = voltage_vector((i-1)*2 + 1);
                
        new_voltage_vector((i-1)*2 + 1)=min(3.2,new_voltage_vector((i-1)*2 + 1));
        new_voltage_vector((i-1)*2 + 1)=max(0,new_voltage_vector((i-1)*2 + 1));
        new_voltage_vector(i*2)=min(3.2,new_voltage_vector(i*2));
        new_voltage_vector(i*2)=max(0,new_voltage_vector(i*2));
        
    end
    
    trailer_voltage=min(3.2,trailer_voltage);
    trailer_voltage=max(0,trailer_voltage);
    
    new_voltage_vector = [new_voltage_vector , trailer_voltage ];

%     disp(new_voltage_vector)
    
    s.outputSingleScan(new_voltage_vector);

end

