%%
% This is an example of how to create a histogram plot in MATLAB&#174;.
% 
% Read about the <http://www.mathworks.com/help/matlab/ref/hist.html |hist|> function in the MATLAB documentation.
%
% For more examples, go to <http://www.mathworks.com/discovery/gallery.html MATLAB Plot Gallery>
%
% Copyright 2012-2014 The MathWorks, Inc.

% Load nucleotide data
load nucleotideData ncount

% Create the histogram using the hist function
figure
hist(ncount)
colormap summer

% Add a legend
legend('A', 'C', 'G', 'T')

% Add title and axis labels
title('Histogram of nucleotide type distribution')
xlabel('Occurrences')
ylabel('Number of sequence reads')
