function [ output_args ] = drawcurvesmark( values,start,islocal)


x = start:1:(length(values) + start - 1);
set(gca,'xtick',start:20:(length(values) + start - 1))
plot(x, values);

%axis([start,(length(values) + start - 1),0.,1.]);

hold on;

%sig = 0: pi/20: 2*pi

for i = start:1:(length(islocal) + start - 1)  
    if islocal(i-start + 1) == 1
        %%plot(i + 0.001*cos(sig), 0.01*values(i-start + 1) + 0.001 * sin(sig));

        plot(i, values(i-start + 1),'.','MarkerSize',20);
        hold on;
    end
end
  
title('Selected Frames Position');
ylabel('Area ratio','FontSize',25);
xlabel('Frame','FontSize',25);


%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


end

