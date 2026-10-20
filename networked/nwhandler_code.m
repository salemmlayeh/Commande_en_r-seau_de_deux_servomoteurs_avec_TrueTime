function [exectime, data] = nwhandler_code(seg, data)

msg=ttGetMsg;         % Obtain sensor value
if (msg.id==1)
    data = ttGetData('controller_task');
    data.msg.y=msg.y;
    ttSetData('controller_task',data);
    ttCreateJob('controller_task');
else
    data = ttGetData('controller_task2');
    data.msg.y=msg.y;
    ttSetData('controller_task2',data);
    ttCreateJob('controller_task2');
end

exectime = -1;
