function [s, c] = get_image_path(im_file, paper_dim, paper_orig, drawntolife)
% Input:    im_file - file name of image
%         paper_dim - [width length] of drawing paper
%        paper_orig - [x0, y0] of paper origin in world (robot) frame
% Output: s - nx3 matrix containing (x y z) coordinates of trajectory
%         c -       nx1 matrix specifying visibility of path coordinates
%         (1 if coordinate is drawn, 0 is coordinate should not be drawn
%         and is just a transition path between boundaries)
% Requires MATLAB Image Processing Toolbox

% Note: works best with completely filled in shapes

%temporary variable set
% drawntolife = drawntolifeInit();
% im_file = "images\owl2.jpg"; 
% % im_file = "images\omalley.jpg";
% paper_dim = [8.5 11];
% paper_orig = [3 -5.5];

% Define paper dimensions (for later use)
paper_wid = paper_dim(1); paper_len = paper_dim(2);
x0 = paper_orig(1); y0 = paper_orig(2);

% Convert image to binary
color_im = imread(im_file); %maybe jpg works better?
gray_im = rgb2gray(color_im);
bw_im = imbinarize(gray_im); 

% Clean up image
se = strel('disk', 1);
iopen = imopen(bw_im, se); %get rid of extraneous pixels
iclose = imclose(iopen, se); %smooth out image

% Set resolution
res = 100; %number of rows in output image
im_resized = imresize(iclose, [res, NaN]);
[im_wid, im_len] = size(im_resized); %used later

% Determine image boundaries
B = bwboundaries(im_resized);
B(1,:) = []; %boundaries always seems to create a box around the image

% Order boundaries from longest -> shortest
b_lengths = cellfun(@length, B);
[~, b_ind] = sort(b_lengths, 'descend');
B = B(b_ind, :); 

% Order boundaries next from closest -> furthest in position (in progress)
% B_ordered = cell(length(B), 1);
B_copy = repmat(B,1);
first_pts = cell2mat(cellfun(@(x) x(1, 1:2), B_copy, 'UniformOutput', false)); %first pt in boundary
first_pts = first_pts(2:end, 1:2); %remove first point
B_copy(1,:) = []; %remove first boundary

% Initialize ordered B array
B_ordered = cell(length(B), 1);
B_ordered{1} = B{1}; 
for i = 1+1:length(B)
    curr_pt = B_ordered{i-1}(end, 1:2);
    dist_vec = abs(curr_pt-first_pts(1:end, 1:2));
    dist_vec = dist_vec(1:end,1)+dist_vec(1:end,2);
    [~, index] = min(dist_vec); %find index of minimum dist. point
    next_bound = B_copy(index);
    B_ordered(i) = next_bound;
    first_pts(index, :) = []; %remove chosen point from options
    B_copy(index,:) = []; %remove chosen boundaries from options
end

% Generate pixel position matrix
boundary_num = length(B_ordered);
boundary_pt_num = sum(b_lengths); %total points in just B, change if modifying B
bn_steps = 20; %transition steps between drawing

% Test Boundary Points
% hold on
% for k = 1:length(B_ordered)
%    boundary = B_ordered{k};
%    plot(boundary(:,1), boundary(:,2),'.')
% end

% Define home point in image frame
[T_home, ~] = drawntolifeFK([0 0 0], drawntolife);
home_pt = T_home(1:2, 4); % xy home defined in robot frame
home_pt = ((home_pt'-paper_orig).*size(im_resized)./paper_dim)';
B_ordered = [B_ordered; home_pt']; %return robot home when drawing is finished

tot_points = boundary_pt_num+(boundary_num+1)*bn_steps; %-3 bc there are zeros at end after homing
s = zeros(tot_points,3); %path matrix
c = zeros(tot_points,1); %visibility matrix, 1 if drawing, 0 if not drawing

% Generate first path from home to first drawing point
first_vec = B_ordered{1}; first_pt = first_vec(1,:); % first point of drawing
s(1:bn_steps, 1:2) = (home_pt*ones(1,bn_steps) + (first_pt'-home_pt)*linspace(0,1,bn_steps))'; %add xy coord to mat
c(1:bn_steps) = zeros(bn_steps,1)'; %set as transition

pt_idx = bn_steps+1;
for k = 1:length(B_ordered)-1
   % Add boundary coordinates to path matrix
   bnd_vec = B_ordered{k}; %xy coordinates of one boundary line
   length_bnd = length(bnd_vec);
   s(pt_idx:pt_idx+length_bnd-1, 1:2) = bnd_vec; %add xy coord to mat
   c(pt_idx:pt_idx+length_bnd-1) = ones(length_bnd,1); %set visible
   
   % Generate transition path between boundary end/start points
   pt1 = bnd_vec(end,:); %end of curr boundary
   next_vec = B_ordered{k+1};
   pt2 = next_vec(1,:);
   trans_path = pt1'*ones(1,bn_steps) + (pt2-pt1)'*linspace(0,1,bn_steps);
   s(pt_idx+length_bnd:pt_idx+length_bnd+bn_steps-1, 1:2) = trans_path'; %add xy coord to mat
   c(pt_idx+length_bnd:pt_idx+length_bnd+bn_steps-1) = zeros(bn_steps,1)'; %set as transition
   
   pt_idx = pt_idx+length_bnd+bn_steps;
end

% Convert from image frame to world frame
s(1:tot_points,1) = (s(1:tot_points,1)*(paper_wid-4)/im_wid)+x0;
s(1:tot_points,2) = (s(1:tot_points,2)*(paper_len-4)/im_len)+y0;
s(1:tot_points,3) = c(1:tot_points)*-.5; %-.5 for extending pen to paper and drawing

% Test post-processed plot
% plot(s(:,1), s(:,2), '.') 

end