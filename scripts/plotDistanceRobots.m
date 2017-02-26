NRobots=[1 2 4 8 16 20 32];
CAC=[1104120 510345 360294 236540 207097 207097 207097];
CRC=[1104120 577569 534482 451810 331629 301110 245848];
figure(1);
clf;
subplot(2,2,1);
plot(NRobots,CAC,'r+');hold on;
a=plot(NRobots,CAC,'r');
plot(NRobots,CRC,'b*');
b=plot(NRobots,CRC,'b');
title('Cave Environment','FontSize',14,'FontWeight','bold');
xlabel('Number of Robots','FontSize',12,'FontWeight','bold');
ylabel('Maximum Distance Travelled (m)','FontSize',12,'FontWeight','bold');
legend([a b],'CAC','CRC');

CAC=[1310300 688556 372524 233897 167994 167994 167519];
CRC=[1310300 727820 687368 502516 386292 388006 291406];

subplot(2,2,2);
a=plot(NRobots,CAC,'r');hold on;
b=plot(NRobots,CRC,'b');
legend([a b],'CAC','CRC');
plot(NRobots,CAC,'r+');
plot(NRobots,CRC,'b*');
title('Rural Quebec','FontSize',14,'FontWeight','bold');
xlabel('Number of Robots','FontSize',12,'FontWeight','bold');
ylabel('Maximum Distance Travelled (m)','FontSize',12,'FontWeight','bold');

CAC=[308014 136851 114076 81786.2 81786.2 81786.2 81786.2];
CRC=[308014 164209 157674 123304 107053 107053 86356.5];

subplot(2,2,3);
plot(NRobots,CAC,'r+');hold on;
a=plot(NRobots,CAC,'r');
plot(NRobots,CRC,'b*');
b=plot(NRobots,CRC,'b-');
title('Indoor Environment','FontSize',14,'FontWeight','bold');
xlabel('Number of Robots','FontSize',12,'FontWeight','bold');
ylabel('Maximum Distance Travelled (m)','FontSize',12,'FontWeight','bold');
legend([a b],'CAC','CRC');


CAC=[1408800 799549 443358 302947 302947 302947 302947];
CRC=[1408800 712279 699767 563726 420078 376449 346104];

subplot(2,2,4);
plot(NRobots,CAC,'r+');hold on;
a=plot(NRobots,CAC,'r');
plot(NRobots,CRC,'b*');
b=plot(NRobots,CRC,'b-');
title('Multi-cell Environment','FontSize',14,'FontWeight','bold');
xlabel('Number of Robots','FontSize',12,'FontWeight','bold');
ylabel('Maximum Distance Travelled (m)','FontSize',12,'FontWeight','bold');
legend([a b],'CAC','CRC');
