%addpath('C:\..etc..');
% Display functions in library.
if not(libisloaded('usbhidlib'))
    loadlibrary('usbhidlib');
end
libfunctions('usbhidlib');
% NOTE: you can ignore the warning:
% "Warning: The data type 'hid_device_infoPtr' used by structure 
% hid_device_info does not exist." -- this is not true or a problem.
% (MatLab is easily confused by C code.)
% create 2 empty arrays of BYTES (uint8 in MatLab)
bOut = zeros(1,65,'uint8');
bIn = zeros(1,65,'uint8');
BufferSize = 64;
pBuffer = libpointer('uint8Ptr',zeros(BufferSize,1));
AC660 = zeros(1,64,'uint16');
AC940 = zeros(1,64,'uint16');

% HID command defines
% CMD_FLASH_LED                   1
% CMD_NORMAL_READ                 2
% CMD_DISPLAY_VOLTAGES            3
% CMD_GET_DATA                    4
% 
% CMD_GET_DATA subcommands
% SEND_DC_PART                     1
% SEND_AC660_1                     2
% SEND_AC660_2                     3
% SEND_AC940_1                     4
% SEND_AC940_2                     5

bOut(2)= 1;     % cmd  - index is 2, because MatLab arrays start at 1
                 % and the 1st element must have a value of 0 ascii for 
                 % the CCS HID code.  
calllib('usbhidlib','hid_send_64bytes',1121, 4, bOut, pBuffer, 100, 10);


bOut(2)= 2;      % Normal read 
calllib('usbhidlib','hid_send_64bytes',1121, 4, bOut, pBuffer, 100, 10);

pause(12);

bOut(2)= 4;      % get DC data 
bOut(3) = 1;
calllib('usbhidlib','hid_send_64bytes',1121, 4, bOut, pBuffer, 100, 10);
y = pBuffer.Value;
val = [y(2),y(1)];
DC660 = typecast(uint8(val), 'uint16');
val = [y(4),y(3)];
DC940 = typecast(uint8(val), 'uint16');
PIC_spo2 = sprintf("%d.%d", y(5), y(6));

% get 1st AC660 data packet
bOut(2)= 4; 
bOut(3) = 2;
calllib('usbhidlib','hid_send_64bytes',1121, 4, bOut, pBuffer, 100, 10);
y = pBuffer.Value;
j = 1;
for i = 1:32 
   val = [y(j+1),y(j)];
   AC660(i) = typecast(uint8(val), 'uint16');
   j = j + 2;
end

% get 2nd AC660 data packet
bOut(2)= 4;      
bOut(3) = 3;
calllib('usbhidlib','hid_send_64bytes',1121, 4, bOut, pBuffer, 100, 10);
y = pBuffer.Value;

j = 1;
for i = 33:64 
   val = [y(j+1),y(j)];
   AC660(i) = typecast(uint8(val), 'uint16');
   j = j + 2;
end

% get 1st AC940 data packet
bOut(2)= 4; 
bOut(3) = 4;
calllib('usbhidlib','hid_send_64bytes',1121, 4, bOut, pBuffer, 100, 10);
y = pBuffer.Value;
j = 1;
for i = 1:32 
   val = [y(j+1),y(j)];
   AC940(i) = typecast(uint8(val), 'uint16');
   j = j + 2;
end

% get 2nd AC940 data packet
bOut(2)= 4;      
bOut(3) = 5;
calllib('usbhidlib','hid_send_64bytes',1121, 4, bOut, pBuffer, 100, 10);
y = pBuffer.Value;
j = 1;
for i = 33:64 
   val = [y(j+1),y(j)];
   AC940(i) = typecast(uint8(val), 'uint16');
   j = j + 2;
end


x = linspace(0,6.3,64);
subplot(2,1,1);
plot(x,AC660);
title('AC 660 nm');
subplot(2,1,2);
plot(x,AC940);
title('AC 940 nm');
str = 'PIC value: ' + PIC_spo2;
annotation('textbox',[.9 .6 .1 .2],'String',str,'EdgeColor','none')
%unloadlibrary('usbhidlib');   % This unloads the dll when the program exits
                              % For developing the MatLab program, REM this
                              % out so it doesn't have to load the dll each
                              % time the pregram runs (saves time). 