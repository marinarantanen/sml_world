
if exist('udp_receiver')

    fclose(udp_receiver)
    delete(udp_receiver)
    
end

if exist('ni_handler')

    NI_voltage_stop(ni_handler, 1)
    delete(ni_handler)
    
end
    
clear

ni_handler = NI_initialization(1);

udp_receiver = udp('', 'LocalPort', 34515);

fopen(udp_receiver)

received_package_counter = -1;



% udp_receiver

velocity = 0;
steering = 0;

while true
%     
%     disp('Rui')
    while udp_receiver.BytesAvailable ~= 0
                
        A = fread(udp_receiver, udp_receiver.BytesAvailable);
                
        message_string = strcat(strcat(A'));
        
        [token_1, message_string] = strtok(message_string, ';');
        
        datagram_number = str2num(token_1);
        
        if datagram_number < received_package_counter
            
            disp('Out of order packet, ignoring')
            continue
            
        else
            
            received_package_counter = datagram_number;
            
        end
        
        [token_2, message_string] = strtok(message_string, ';');
        [token_3, message_string] = strtok(message_string, ';');
        
        velocity = str2num(token_2);
        steering = str2num(token_3);
                       
        
    end
        
%     steering = 0;
    
    disp(['Velocity = ', num2str(velocity), ' steering = ', num2str(steering), ' Latest packet number = ', num2str(received_package_counter)])
    
%     voltage_vector = [1.8 + velocity*0.6, 1.6 - steering*0.8, 3.2];

    % For the maze scenario, not working
    voltage_vector = [1.8 + velocity*0.4, 1.6 - steering*1.2, 3.2];
    
    NI_voltage_output(ni_handler,voltage_vector)
    
    pause(0.05)
    
end