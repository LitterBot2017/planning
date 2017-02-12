%homography script
clear all;
close all;


in = ...
[ 216,142;
  363,151;
  507,148;
  652,159;
  795,159;
  931,161;
  1080,163;
  211,243;
  358,251;
  503,258;
  648,260;
  791,264;
  931,264;
  1076,264;
  211,394;
  352,396;
  497,400;
  642,402;
  791,409;
  933,411;
  1076,411;
  207,539;
  350,543;
  929,556;
  1076,558;
  195,684]'

img = imread('2017-02-11-184237.jpg');

figure;
imshow(img);
hold on;
scatter(in(1,:),in(2,:),30,'c');

out = ...
[ 37,30;
  37,20;
  37,10;
  37,0;
  37,-10;
  37,-20;
  37,-30;
  30,30;
  30,20;
  30,10;
  30,0;
  30,-10;
  30,-20;
  30,-30;
  20,30;
  20,20;
  20,10;
  20,0;
  20,-10;
  20,-20;
  20,-30;
  10,30;
  10,20;
  10,-20;
  10,-30;
  0,30]'

  h = homography_solve(in,out);
  
  tformd = homography_transform(in,h);
  
  figure;
  scatter(tformd(1,:),tformd(2,:),30,'c');