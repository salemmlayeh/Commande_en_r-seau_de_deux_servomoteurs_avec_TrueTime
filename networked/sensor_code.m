function [exectime, data] = sensor_code(seg, data)

persistent y

switch seg
 case 1
  y = ttAnalogIn(1);
  exectime = 0.0005;
 case 2
     msg.id=1;
     msg.y=y;
  ttSendMsg(3, msg, 80); % Send message (80 bits) to node 3 (controller)
  exectime = 0.0004;
 case 3
  exectime = -1; % finished
end
