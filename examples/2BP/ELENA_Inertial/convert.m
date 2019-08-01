%
mission = load('mission_performance.txt');
%
rx_0 =  mission(1,2)*1000;
ry_0 =  mission(1,3)*1000;
vx_0 =  mission(1,8)*1000;
vy_0 =  mission(1,9)*1000;


x0 = [rx_0,ry_0,0,vx_0,vy_0,0]';


%x0 = [1.299345289722e+09 -1.150007503487e+09 -0.000000000000e+00 4.712765363632e+03 6.731444319561e+03 0.000000000000e+00]';
coe = orbitalElementsFromState(x0)

