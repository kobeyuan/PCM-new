function [ ptslist] = findSeamPos( labImage )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%gayImg = rgb2gray(labImage);
%threshold = graythresh(gayImg);

threshold = graythresh(labImage);

bw = im2bw(labImage,threshold);

ptscell = bwboundaries(~bw);

%figure(1)
%plot(ptscell{1}(:,2), ptscell{1}(:,1));
 
ptslist = cell2mat(ptscell);

end

