clear all;
clc;
%% Radar Specifications

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
%% User Defined Range and Velocity of target
% *%TODO* : define the target's initial position and velocity. Note : Velocity 
% remains contant

Rt = 110; % m
Vt = -20; % m/s
d_Resolution = 1; % m
c = 3e8; % m/s
max_radar_range = 200;
max_radar_velocity = 100;
%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B = c/2*d_Resolution;

Tchirp = ( 5.5 * 2 * max_radar_range ) / c;

slope = B / Tchirp;
%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

                                                          
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t));
td=zeros(1,length(t));
%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = Rt + Vt*t(i);
    td(i) = 2*r_t(i)/c;
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos( 2*pi*( fc*t(i) + 0.5*slope*t(i)^2) );
    Rx(i) = cos( 2*pi*( fc*(t(i)-td(i)) + 0.5*slope*(t(i)-td(i))^2));
        
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = (Tx(i).*Rx(i));
    
end

figure; plot(t, slope.*t, 'r--', t, slope.*(t-td), 'g-');
xlim([0 1e-5]);
legend('Signal Transmitted','Echo Signal Received');
title('Signals Used');
print('Signals','-dpng')

%% RANGE MEASUREMENT

  % *%TODO* :
%reshape the vector into Nr*Nr array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.

signal = reshape(Mix,Nr,Nd);
 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
% Call only 1D FFT
signal_fft = fft(signal,Nr)./Nr;
 % *%TODO* :
% Take the absolute value of FFT output
signal_fft = abs(signal_fft);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
signal_fft = signal_fft(1:(Nr/2));
%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 
plot(signal_fft, 'LineWidth', 4, "Color", 'r');
title( 'Range from First FFT');
xlabel('Range');
ylabel('Amplitude');
grid on;
axis ([0 200 0 1]);
print('First_FFT','-dpng')
%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT 
% on the mixed signal (beat signal) output and generate a range doppler map.You 
% will implement CFAR on the generated RDM

% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure('Name','2DFFT Surface Plot')
figure,surf(doppler_axis,range_axis,RDM);
colorbar;
title( 'Surface plot 2DFFT');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');
print('SurfacePlot_2DFFT','-dpng')
%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 10; % Tried: 10;
Td = 8;  % Tried: 4;


% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4; % Tried: 5
Gd = 4; % Tried: 2

% *%TODO* :
% offset the threshold by SNR value in dB
% Tried values {4, 6}
offset=1.4;

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells

% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.


% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR
% Normalise RDM

RDM = RDM/max(max(RDM));

% Iterate the RDM using the training and guard cells
for x = Tr+Gr+1:(Nr/2)-(Gr+Tr)
    for y = Td+Gd+1:Nd-(Gd+Td)
        noise_level = zeros(1,1);
        
        % Check aggregate noise around CUT
        % Iterate x until CUT +- Training and Guard range cells
        for i = x - (Tr+Gr): x + (Tr+Gr)
            % Iterate y until CUT +- Training and Guard doppler cells
            for j = y - (Td+Gd): y + (Td+Gd)
                % Everything but Guard cell
                if( ( abs(x -i) > Gr ) || ( abs(y-j) > Gd ) )
                    % add signal level converted to linear
                    noise_level = noise_level + db2pow(RDM(i,j));
                end
            end
        end
        
        % Convert to logarithmic using pow2db and add to offset
        threshold = pow2db(noise_level/(2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1));
        threshold = threshold + offset;
           
        %threshold = pow2db(noise_level/ ( (2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1) ));
        %threshold = pow2db(noise_level/(2*(Td+Gd+1)*2*(Tr+Gr+1)-(Gr*Gd)-1));
       
        % Cell Under Test
        CUT = RDM(x,y);
        
        % Threshold check and assign true and false accordingly        
        if( CUT <= threshold )
            RDM(x,y) = 0;
        else
            RDM(x,y) = 1;
        end
    end
end

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
RDM(RDM~=0 & RDM~=1) = 0;

% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
%figure,surf(doppler_axis,range_axis,RDM);
%colorbar;
figure('Name', 'CFAR RDM')
surf(doppler_axis,range_axis,RDM);
colorbar;
title( 'Surface plot RDM');
xlabel('Speed');
ylabel('Range');
zlabel('Normalized Amplitude');
print('CFAR_RDM_Surface_Plot','-dpng')

% Surface plot against Range
figure ('Name','Range and Amplitude');
surf(doppler_axis,range_axis,RDM);
colorbar;
title('Range and Amplitude');
xlabel('Speed');
ylabel('Range');
zlabel('Amplitude');
view(90,0);
print('Range_Amplitude_Surface_plot','-dpng')

% Surface plot against Speed
figure ('Name','Speed and Amplitude');
surf(doppler_axis,range_axis,RDM);
colorbar;
title('Speed and Amplitude');
xlabel('Speed');
ylabel('Range');
zlabel('Amplitude');
view(0,0);
print('Speed_Amplitude_Surface_Plot','-dpng')