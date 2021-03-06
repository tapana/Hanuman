function MonitorTeam(dispDir)

if (nargin < 1)
  dispDir = 'y';
end

nRobots = 4;
robots = cell(nRobots,1);

teamNumber = 18;
nUpdate = 0;


while (1)
	nUpdate = nUpdate + 1;
    i=0;
	% receive udp packets
	while(UDPComm('getQueueSize') > 0)
		i = i+1;
        msg = UDPComm('receive');
       
		if ~isempty(msg)
          try
            fprintf('try prasing msg\n');
            msg = lua2mat(char(msg));
            fprintf('prasing done\n');
            msg.tReceive = time;
            fprintf('team number %d\n',msg.teamNumber);

            if (isfield(msg, 'teamNumber') && msg.teamNumber == teamNumber)
              robots{msg.id} = msg;
            end
          catch
            %disp('failed to parse')
            %disp(char(msg))
            fprintf('failed lua2mat\n');
          end
        else
           fprintf('msg empty');         
        end
	end

	% plot current robot positions
  cla;
	plot_field(gca,1);
	hold on;

	for i = 1:nRobots
		if (~isempty(robots{i}))
      plot_robot(robots{i}, [], 1, 4);
		end
	end

	% time out messages after 10 seconds
	for i = 1:nRobots
		if (~isempty(robots{i}))
			if (robots{i}.tReceive - time > 10)
				robots{i} = [];
			end
		end
	end
				

  if (dispDir == 'y')
    set(gca, 'CameraUpVector', [1, 0, 0]);
  elseif (dispDir == 'b' || dispDir == 'c')
    set(gca, 'CameraUpVector', [-1, 0, 0]);
  end

	drawnow;

end


end
