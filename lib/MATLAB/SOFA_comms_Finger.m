% Create TCP client
t = tcpclient('localhost', 12345);

% Configure the simulation of the control loop
try
    while true
        % Read the robot positions from SOFA
        data = read(t, t.BytesAvailable, 'string');
        if isempty(data)
            continue;
        end

        % THIS EXAMPLE WORKS FOR THE FINGER, FOR THE ANKLE USE THE OTHER
        % FILE SOFA_COMMS_ANKLE

        target=[50,0,50]% Target for the finger
        structPos = jsondecode(data);
        positionEndEffector=structPos.posiciones'
        disp(['positiones recibidas: ', mat2str(structPos.posiciones)]);
        control=0.05*(target-positionEndEffector) % kp=1 for the finger
        % Control Algorithm (PID or another)
        % Here you make the calculations for the control actions
        control_actions = struct('actuadores',control);  
        
        % Encode the actions in json format and send back to SOFA
        actions_json = jsonencode(control_actions);
        write(t, actions_json, 'string');
        
        %pause(0.1);  % Ajustar frecuencia de actualización (100 ms)
    end
catch ME
    disp(['Error en MATLAB: ', ME.message]);
end

% Close the socket to end
clear t;