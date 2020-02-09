clc; 
clear;
addpath(genpath('.'));

% set the colormap (for semantics)
semantickitti_label_colormap;

% set the colormap (for instances)
num_max_instances = 10000;
instance_colormap = round(255 * rand(num_max_instances, 3));
non_movable_color = 210;
instance_colormap = [non_movable_color, non_movable_color, non_movable_color; instance_colormap]; % the first color is reserved for the non-movable instance color.

%%
SEQ_IDX = "00";

BASE_DIR = "your/path/to/kitti/"; % change this 
DIR_SCAN = fullfile(BASE_DIR, SEQ_IDX, "velodyne");
DIR_LABEL = fullfile(BASE_DIR, SEQ_IDX, "labels");

scan_filenames = listdir(DIR_SCAN);
label_filenames = listdir(DIR_LABEL);

num_scans = length(scan_filenames);

%%
clear framecaptures_semantics;
clear framecaptures_instances;
frame_recorded_idx = 1;

num_skips = 2;
for target_idx = 1:num_scans
    if(rem(target_idx, num_skips) ~= 0)
       continue; 
    end
    
    % 0. read 
    target_idx_str = num2str(target_idx, '%06.f');
    target_scan_filename = scan_filenames{target_idx}; target_scan_filepath = fullfile(DIR_SCAN, target_scan_filename);
    target_label_filename = label_filenames{target_idx}; target_label_filepath = fullfile(DIR_LABEL, target_label_filename);

    pc = readBin(target_scan_filepath, 0);
    [pc_lb, pc_id] = readLabel(target_label_filepath); % lb: label (28 types), id: instance id 

    % 1. semantic seg
    pc_lb_set = unique(pc_lb);
    num_lb = length(pc_lb_set); 
    if(num_lb > num_LABEL_INDEXES)
        disp(strcat("- ", num2str(target_idx), ": false scan, pass it."));
        continue; 
    end

    pc_colors = zeros(pc.Count, 3);
    for ith = 1:numel(pc_lb_set)
        semantic_idx = pc_lb_set(ith);
        semantic_color = LABEL_COLORMAP_uint((semantic_idx == LABEL_INDEXES), :);
        points_idx_this_semantic = find(semantic_idx == pc_lb);
        pc_colors(points_idx_this_semantic, :) = repmat(semantic_color, numel(points_idx_this_semantic), 1);
    end
    pc.Color = uint8(pc_colors);

    % 2. instance seg
    pc_instance = pointCloud(pc.Location);

    [pc_id_set, num_each_id] = unique(pc_id);
    num_id = length(pc_id_set);

    pc_instance_colors = round(255 * 0.8 * ones(pc_instance.Count, 3));
    for ith = 1:numel(pc_id_set)
        instance_idx = pc_id_set(ith);
        instance_color = instance_colormap(instance_idx+1, :); % tmp, blue
        points_idx_this_instance = find(instance_idx == pc_id);
        pc_instance_colors(points_idx_this_instance, :) = repmat(instance_color, numel(points_idx_this_instance), 1);
    end
    pc_instance.Color = uint8(pc_instance_colors);
    
    % viz
    ptsize = 20;
    limsize = 150; zoomfactor = 6;

    figure(1); clf;
    pcshow(pc, 'MarkerSize', ptsize);
    xlim([-limsize, limsize]); ylim([-limsize, limsize]);
    zlim([-4, 15]);
    title_str = strcat("KITTI ", SEQ_IDX, ": ", num2str(target_idx)); title(title_str);
    grid off;
    zoom(zoomfactor);
    text(0, 10, 30, title_str);
    framecaptures_semantics(frame_recorded_idx) = getframe(gcf);
    
    figure(2); clf;
    pcshow(pc_instance, 'MarkerSize', ptsize);
    xlim([-limsize, limsize]); ylim([-limsize, limsize]);
    zlim([-4, 15]);
    title_str = strcat("KITTI ", SEQ_IDX, ": ", num2str(target_idx)); title(title_str);
    grid off;
    zoom(zoomfactor);
    text(0, 10, 30, title_str);
    framecaptures_instances(frame_recorded_idx) = getframe(gcf);
    
    %
    frame_recorded_idx = frame_recorded_idx + 1;
end


% a viz result writer 
writerObj = VideoWriter('semantickitti_viz.avi');
writerObj.FrameRate = 5;
open(writerObj);
for i=1:length(framecaptures_semantics)
    frame_semantic = framecaptures_semantics(i);    
    frame_instance = framecaptures_instances(i);    

    frame_semantic.cdata = [frame_semantic.cdata, frame_instance.cdata];
    writeVideo(writerObj, frame_semantic);
end
close(writerObj);







