function [pc_lb, pc_id] = readLabel(file_path)
   
%% Read 
fid = fopen(file_path, 'rb'); 
label = fread(fid, [1 inf], 'uint32'); 
fclose(fid);

% ref: https://github.com/PRBonn/semantic-kitti-api/blob/77d6e9b2ddd8ec334bf97651ffa68a75345b2796/auxiliary/laserscan.py#L238
pc_lb = bitand(label, hex2dec('ffff')); % lower 16 bit 
pc_id = double(bitsrl(uint32(label), 16)); % upper 16 bit 
      
end % end of function
