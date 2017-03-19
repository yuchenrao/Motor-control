function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',20); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 
has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('     a: Read current sensor(ADC counts)     b: Read current sensor (mA)\n     c: Read encoder(counts)     d: Read encoder (deg)\n     e: Reset encoder     f: Set PWM (-100 to 100)\n     g: Get current gains     h: Get current gains\n     i: Set position gains     j: Get position gains\n     k: Test current control     l: Go to angle (deg)\n     m: Load step trajectory:        n: Load cubic trajectory \n        o: excute trajectory      p: Unpower the motor\n      q: Quit client     r: Get mode\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'a'
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
        case 'b'
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
        case 'c'
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
        case 'd'                 
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
        case 'e'
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
        case 'f'
            n = input('What PWM value would you like [-100 to 100]?'); % get the number to send
            fprintf(mySerial, '%d\n',n); % send the number
            if n < 0
                fprintf('PWM has been set to %d in the counterclockwise direction.\n', n);
            else
                fprintf('PWM has been set to %d in the clockwise direction.\n', n);
            end
        case 'g'
            Kpi = input('Enter your desired Kp current gain [recommended: 100]: '); % get the number to send
            fprintf(mySerial, '%f\n',Kpi); % send the number
            Kii = input('Enter your desired Ki current gain [recommended: 1]: '); % get the number to send
            fprintf(mySerial, '%f\n',Kii); % send the number
            fprintf('Sending Kp = %f and Ki = %f to the current controller.\n', Kpi, Kii);
        case 'h'
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
        case 'i'
            Kpp = input('Enter your desired Kp position gain [recommended: 2]: '); % get the number to send
            fprintf(mySerial, '%f\n',Kpp); % send the number
            Kip = input('Enter your desired Ki position gain [recommended: 0.3]: '); % get the number to send
            fprintf(mySerial, '%f\n',Kip); % send the number
            Kdp = input('Enter your desired Kd position gain [recommended: 200]: '); % get the number to send
            fprintf(mySerial, '%f\n',Kdp); % send the number
            fprintf('Sending Kp = f%, Ki = %f and Kd = f% to the position controller.\n', Kpp, Kip, Kdp);
        case 'j'
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
            %fprintf('The position controller is using Kp = f%, Ki = %f and Kd = f%.\n', Kp, Ki, Kd);
        case 'k'
            nsamples0 = fscanf(mySerial,'%d')       % first get the number of samples being sent0
            data0 = zeros(nsamples0,2);               % two values per sample:  ref and actual
            for i=1:nsamples0
              data0(i,:) = fscanf(mySerial,'%d %d'); % read in data from PIC32; assume ints, in mA
              times0(i) = (i-1)*0.2;                 % 0.2 ms between samples
            end
            if nsamples0 > 1						        
              stairs(times0,data0(:,1:2));            % plot the reference and actual
            else
              fprintf('Only 1 sample received\n')
              disp(data0);
            end     
            % compute the average error
            score = mean(abs(data0(:,1)-data0(:,2)));
            fprintf('\nAverage error: %5.1f mA\n',score);
            title(sprintf('Average error: %5.1f mA',score));
            ylabel('Current (mA)');
            xlabel('Time (ms)');
        case 'l'
            angle = input('Enter your desired motor angle in degrees: '); % get the number to send
            fprintf(mySerial, '%f\n',angle);
            fprintf('Motor moving to %d degree.\n', angle);
        case 'm'
            trajectory = input('Enter step trajectory, in sec and segrees [time1, ang1; time2, ang2; ...] '); % get the number to send           
            ref = genRef(trajectory, 'step');
            l = length(ref);
            fprintf(mySerial, '%d\n', l);
            for i = 1: l
                fprintf(mySerial, '%f\n', ref(i)); 
            end
        case 'n'
            trajectory = input('Enter step trajectory, in sec and segrees [time1, ang1; time2, ang2; ...] '); % get the number to send           
            ref = genRef(trajectory, 'cubic');
            l = length(ref);
            fprintf(mySerial, '%d\n', l);
            for i = 1: l
                fprintf(mySerial, '%f\n', ref(i));
            end
        case 'o'
            nsamples = fscanf(mySerial,'%d');      % first get the number of samples being sent0
            data = zeros(nsamples,2);               % two values per sample:  ref and actual
            for i=1:nsamples
              data(i,:) = fscanf(mySerial,'%f %f'); % read in data from PIC32; assume ints, in mA
              times(i) = i/200.0;                 % 0.2 ms between samples
            end
            if nsamples > 1						        
              stairs(times,data(:,1:2));            % plot the reference and actual
            else
              fprintf('Only 1 sample received\n')
              disp(data);
            end     
            % compute the average error
            score = mean(abs(data(:,1)-data(:,2)));
            fprintf('\nAverage error: %5.1f mA\n',score);
            title(sprintf('Average error: %5.1f mA',score));
            ylabel('Current (mA)');
            xlabel('Time (ms)');
        case 'p'
             fprintf('Set mode to IDLE\n');
        case 'q'
            has_quit = true;             % exit client
        case 'r'
            statement = fscanf(mySerial,'%s');
            fprintf(statement);
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end
