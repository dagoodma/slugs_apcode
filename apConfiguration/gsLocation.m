% This is the file that configures the location of the Ground Station make
% sure that GS_location has 0 altitude since the altitude is read in base
% height. Do not modify this file, since this is under version control,
% rather copy it name it what you want and replace the line where its being
% called in InnerOuterInit.m (simulation) or mcuInit.m (for HIL and
% flight).

% GS Location. East field UCSC location
% GS_location = [0 36.9898376464844 -122.051963806152];
% GS_location = [0 19.0450708 -95.9717882];
% GS_location = [0 36.99 -122.05]; %UCSC
GS_location = [0 36.988506 -122.055308]; %UCSC



baseHeight = 100;